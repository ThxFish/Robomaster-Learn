#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/* can instance ptrs storage, used for recv callback */
// 存放所有已经注册的CAN实例指针。
// 当CAN总线产生接收中断时，CPU会遍历这个数组，挑选出 `can_handle`(属于哪个CAN外设) 和 `rx_id`(接收ID)
// 与引发中断的报文相匹配的实例，随后调用该实例绑定的回调函数 `can_module_callback` 将数据传递给上层模块（如电机解析函数）。
// @todo: 后续为每个CAN总线单独添加一个can_instance指针数组,提高回调查找的性能
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引，每次有新的传感器/电机模块发起CAN注册，此值就会自增。

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief 添加硬件过滤器以实现对特定ID报文的接收。此函数在 CANRegister() 中被自动调用。
 *
 * @note  原理讲解：
 *        CAN总线是一个广播总线，总线上所有节点都能“听到”所有的报文。为了节省单片机CPU的算力，
 *        STM32硬件FDCAN外设内置了“过滤器(Filter)”。只有我们在过滤器中注册过的 `rx_id` (接收ID)，
 *        FDCAN硬件才会自动将其放行，存入接收FIFO中，并触发中断通知CPU处理。而不相关的报文直接在硬件层被丢弃。
 *        本项目中，所有过滤通过的报文都会被统一路由到 RxFifo0 队列。
 *
 * @param _instance 指向正在被注册的CAN实例（例如某一个电机的实例）
 */
static void CANAddFilter(CANInstance *_instance)
{
    FDCAN_FilterTypeDef fdcan_filter_conf;
    static uint8_t fdcan1_std_filter_idx = 0, fdcan2_std_filter_idx = 0, fdcan3_std_filter_idx = 0; // 标准ID过滤器索引
    static uint8_t fdcan1_ext_filter_idx = 0, fdcan2_ext_filter_idx = 0, fdcan3_ext_filter_idx = 0; // 扩展ID过滤器索引

    // 根据ID类型配置过滤器
    if (_instance->id_type == CAN_ID_EXT)
    {
        fdcan_filter_conf.IdType = FDCAN_EXTENDED_ID; // 使用扩展ID(29位)
        // 根据can_handle判断是FDCAN1、FDCAN2还是FDCAN3,然后自增
        if (_instance->can_handle == &hfdcan1)
            fdcan_filter_conf.FilterIndex = fdcan1_ext_filter_idx++;
        else if (_instance->can_handle == &hfdcan2)
            fdcan_filter_conf.FilterIndex = fdcan2_ext_filter_idx++;
        else
            fdcan_filter_conf.FilterIndex = fdcan3_ext_filter_idx++;
        fdcan_filter_conf.FilterID1 = _instance->rx_id;      // 过滤ID
        fdcan_filter_conf.FilterID2 = _instance->rx_id_mask; // 使用用户配置的掩码
    }
    else
    {
        fdcan_filter_conf.IdType = FDCAN_STANDARD_ID; // 使用标准ID(11位)
        // 根据can_handle判断是FDCAN1、FDCAN2还是FDCAN3,然后自增
        if (_instance->can_handle == &hfdcan1)
            fdcan_filter_conf.FilterIndex = fdcan1_std_filter_idx++;
        else if (_instance->can_handle == &hfdcan2)
            fdcan_filter_conf.FilterIndex = fdcan2_std_filter_idx++;
        else
            fdcan_filter_conf.FilterIndex = fdcan3_std_filter_idx++;
        fdcan_filter_conf.FilterID1 = _instance->rx_id;      // 过滤ID
        fdcan_filter_conf.FilterID2 = _instance->rx_id_mask; // 使用用户配置的掩码
    }
    fdcan_filter_conf.FilterType = FDCAN_FILTER_MASK;         // 使用掩码模式
    fdcan_filter_conf.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 消息路由到RxFifo0

    HAL_FDCAN_ConfigFilter(_instance->can_handle, &fdcan_filter_conf);
}

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动FDCAN1和FDCAN2,开启FDCAN1和FDCAN2的RxFifo0接收通知
 *
 */
static void CANServiceInit()
{
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* ----------------------- two extern callable function -----------------------*/

/**
 * @brief  bsp_can 的核心功能：CAN实例注册函数
 *
 * @note   设计思路【重要】：
 *         任何上层应用（比如各种电机驱动或大疆云台传感器）若想使用 CAN，不需要直接操作 HAL 库的各种寄存器。
 *         只需要调用此涵数，并传入配置结构 `config`，声明：
 *         1. 我用哪条总线 (`can_handle`)
 *         2. 我要发哪个ID (`tx_id`)
 *         3. 我期望收哪个ID (`rx_id`)
 *         4. 收到了这个ID后，要回调我的什么代码 (`can_module_callback`，函数指针)
 *         BSP 层就会在底层准备好所有过滤器、开启中断。等有对应报文到达时，上层的 callback 就会直接拿到解析后的数据。
 *
 * @param  config: 通用的CAN初始化配置结构（见 bsp_can.h）
 * @retval CANInstance*: 返回给上层模块它所专属的CAN实例句柄，用于将来通过 `CANTransmit` 发送报文。
 */
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (!idx)
    {
        CANServiceInit(); // 第一次注册,先进行硬件初始化
        LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= CAN_MX_REGISTER_CNT) // 超过最大实例数
    {
        while (1)
            LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
    }
    for (size_t i = 0; i < idx; i++)
    { // 重复注册 | id重复
        if (can_instance[i]->rx_id == config->rx_id && can_instance[i]->can_handle == config->can_handle)
        {
            while (1)
                LOGERROR("[}bsp_can] CAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
        }
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance)); // 分配空间
    memset(instance, 0, sizeof(CANInstance));                           // 分配的空间未必是0,所以要先清空
    // 进行发送报文的配置
    instance->txconf.Identifier = config->tx_id;                                                       // 发送id
    instance->txconf.IdType = (config->id_type == CAN_ID_EXT) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID; // 根据配置选择标准或扩展ID
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME;                                                   // 发送数据帧
    instance->txconf.DataLength = FDCAN_DLC_BYTES_8;                                                   // 默认发送长度为8字节
    instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    instance->txconf.BitRateSwitch = FDCAN_BRS_OFF; // 经典CAN模式,不使用位速率切换
    instance->txconf.FDFormat = FDCAN_CLASSIC_CAN;  // 使用经典CAN格式
    instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    instance->txconf.MessageMarker = 0;
    // 设置回调函数和接收发送id
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id; // 好像没用,可以删掉
    instance->rx_id = config->rx_id;
    instance->id_type = config->id_type; // 保存ID类型
    // 设置rx_id_mask,如果用户未配置(0),则使用默认精确匹配
    if (config->rx_id_mask == 0)
        instance->rx_id_mask = (config->id_type == CAN_ID_EXT) ? 0x1FFFFFFF : 0x7FF;
    else
        instance->rx_id_mask = config->rx_id_mask;
    instance->last_rx_identifier = 0;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    CANAddFilter(instance);         // 添加CAN过滤器规则
    can_instance[idx++] = instance; // 将实例保存到can_instance中

    return instance; // 返回can实例指针
}

/* @todo 目前似乎封装过度,应该添加一个指向tx_buff的指针,tx_buff不应该由CAN instance保存 */
/* 如果让CANinstance保存txbuff,会增加一次复制的开销 */
/**
 * @brief  向 CAN 硬件 FIFO 队列填充数据，准备非阻塞式发送
 *
 * @note   原理总结：
 *         这里使用了 timeout 机制，如果连续发送导致 FDCAN 的硬件 `TxFIFO` 打满。
 *         如果不提供超时，单纯的死循环会把系统卡成假死。
 *         加入超时后，就算某总线硬件级拥堵/离线（例如被短路），只会丢包不会卡死整个 RTOS。
 *
 * @param  _instance:   调用此发送请求的上层实例 （包含了发送数据 tx_buff, 所用句柄, tx_id等）
 * @param  timeout:     如果在该毫秒数内底层FIFO仍被占满无空隙，就返回 ERROR 丢弃该包指令。
 * @retval HAL_StatusTypeDef: 当它入队成功了，不代表发完了，只是成功放到了 FDCAN 硬件发射池里。
 */
HAL_StatusTypeDef CANTransmit(CANInstance *_instance, float timeout)
{
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused)); // for cancel warning
    float dwt_start = DWT_GetTimeline_ms();
    while (HAL_FDCAN_GetTxFifoFreeLevel(_instance->can_handle) == 0) // 等待TxFifo空闲
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
        {
            LOGWARNING("[bsp_can] FDCAN TxFifo full! failed to add msg to fifo. Cnt [%d]", busy_count);
            busy_count++;
            return HAL_BUSY;
        }
    }
    wait_time = DWT_GetTimeline_ms() - dwt_start;
    // 发送消息到TxFifo
    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->can_handle, &_instance->txconf, _instance->tx_buff) != HAL_OK)
    {
        LOGWARNING("[bsp_can] FDCAN bus BUSY! cnt:%d", busy_count);
        busy_count++;
        return HAL_BUSY;
    }
    return HAL_OK; // 发送成功
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查
        while (1)
            LOGERROR("[bsp_can] CAN DLC error! check your code or wild pointer");

    // 将字节长度转换为FDCAN的DLC代码
    uint32_t dlc_codes[] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8};
    _instance->txconf.DataLength = dlc_codes[length];
}

void CANSetTxId(CANInstance *_instance, uint32_t tx_id)
{
    _instance->tx_id = tx_id;
    _instance->txconf.Identifier = tx_id;
}

/* -----------------------belows are callback definitions--------------------------*/

/**
 * @brief  FDCAN底层的接收回调（中断）核心处理中枢。无论哪个电机、裁判系统数据发来，都会走这里。
 *         这个静态函数专门用来拆解进入 RxFifo0 队列的消息。
 *
 * @note   工作原理：
 *         这里是硬件中断（ISR）层面。一旦外部有一帧CAN数据来到芯片并存入FDCAN1/2/3的RxFifo0硬件缓冲里：
 *         1. 取出这帧数据(ID，内容，长度)。
 *         2. O(N) 遍历刚才我们在 `CANRegister` 中存的那一整个 `can_instance[]` 数组。
 *         3. 通过判断：【是不是这条总线】 && 【标准帧/扩展帧能否对上】 && 【利用掩码能否匹配ID】
 *         4. 比对成功后，把接收到的纯字节流，原封不动地 memcpy() 复制到那个 模块实体 的 `rx_buff` 内存中。
 *         5. 最终通过函数指针 `can_module_callback()` 调用上层应用解析这段 `rx_buff` (例如算出现在的转速rpm)。
 *
 * @param _hcan 引发中断的当前 FDCAN 硬件总线句柄 (hfdcan1 / hfdcan2 / ...)
 * @param fifox FIFO标号(通常都是 FDCAN_IT_RX_FIFO0_NEW_MESSAGE，所以就是0)
 */
static void CANFIFOxCallback(FDCAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    FDCAN_RxHeaderTypeDef rxconf;
    uint8_t can_rx_buff[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        if (HAL_FDCAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff) != HAL_OK) // 从FIFO中获取数据
            continue;

        uint32_t rx_id = rxconf.Identifier;
        CAN_ID_Type_e rx_id_type = (rxconf.IdType == FDCAN_EXTENDED_ID) ? CAN_ID_EXT : CAN_ID_STD;

        for (size_t i = 0; i < idx; ++i)
        { // 需要同时匹配can_handle、id_type,并使用掩码匹配rx_id
            if (_hcan == can_instance[i]->can_handle &&
                rx_id_type == can_instance[i]->id_type &&
                (rx_id & can_instance[i]->rx_id_mask) == (can_instance[i]->rx_id & can_instance[i]->rx_id_mask))
            {
                if (can_instance[i]->can_module_callback != NULL) // 回调函数不为空就调用
                {
                    // FDCAN的DataLength是DLC代码,根据文档实际就是长度
                    can_instance[i]->rx_len = rxconf.DataLength; // 提取实际字节数
                    if (can_instance[i]->rx_len > 8)
                        can_instance[i]->rx_len = 8;                                        // 安全检查
                    can_instance[i]->last_rx_identifier = rx_id;                            // 保存完整的接收ID,用于解析扩展协议
                    memcpy(can_instance[i]->rx_buff, can_rx_buff, can_instance[i]->rx_len); // 消息拷贝到对应实例
                    can_instance[i]->can_module_callback(can_instance[i]);                  // 触发回调进行数据解析和处理
                }
                return;
            }
        }
    }
}

/**
 * @brief 注意,STM32H7的FDCAN每个实例有独立的FIFO
 * 下面的函数是HAL库中的回调函数,它被HAL声明为__weak,这里对其进行重载(重写)
 * 当RxFifo0有新消息(不论是FDCAN1、2还是3)时，STM32硬件会触发对应的中断，并进入这个中断处理函数。
 * 我们的代码通过识别 `hfdcan->Instance` 转交给刚刚重写的 `CANFIFOxCallback()` 进行 O(N) 遍历分发。
 */
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定FDCAN设备的消息

/**
 * @brief rx fifo callback. Once RxFifo0 receives new message, this func would be called
 *
 * @param hfdcan FDCAN handle indicate which device the message comes from
 * @param RxFifo0ITs 中断标志
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        CANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
    }
}
