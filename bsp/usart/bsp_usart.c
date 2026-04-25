/**
 * @file bsp_usart.c
 * @author neozng
 * @brief  串口bsp层的实现
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "bsp_usart.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "memory.h"

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart服务实例配置：所有调用 USARTRegister 注册了usart的传感器/射频模块信息会被保存在这个数组里 */
static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

// 串口接收缓冲区。
// @note 硬件级优化：在STM32H7中，为了配合具有极强性能的DMA（直接内存访问控制器），
//       我们将这块接收缓冲池硬编码映射到了 0x30000000 开头的 SRAM D2 区域。
//       因为H7如果使用默认的DTCM区域，DMA是无法直接访问的（或者需要很复杂的Cache一致性维护）。
uint8_t (*recv_buffs)[USART_RXBUFF_LIMIT] = (uint8_t (*)[USART_RXBUFF_LIMIT])0x30000000;

/**
 * @brief  初始化特定串口的底层服务，开启 DMA+IDLE 接收模式。
 *
 * @note   原理详解：
 *         传统单片机收串口数据是一个字节触发一次中断，CPU开销极大。
 *         这里使用了最高效的 `HAL_UARTEx_ReceiveToIdle_DMA` 函数：
 *         1. 让DMA自动搬运数据到SRAM。
 *         2. 只有当对方发完一帧数据（总线产生 IDLE 空闲帧）时，才触发一次中断让CPU去处理。
 *         这对于遥控器数据接收、裁判系统变长协议接收非常合适。
 *
 * @param _instance 上层应用的串口实例
 */
void USARTServiceInit(USARTInstance *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    // [重要填坑] HAL库机制缺陷：
    // 使用 DMA+IDLE 时，不仅接收完成、产生IDLE会进回调，DMA传输进行到一板（Half Transfer）也会进回调。
    // 这会导致一帧数据被拦腰截断处理两次。因此这里强制关闭 DMA 半传输中断。
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");

    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff = recv_buffs[idx];
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;

    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

/**
 * @brief  向串口发送数据（提供阻塞/中断/DMA三种不同级别的效率发送方案）
 *
 * @note   原理总结：
 *         在电控比赛场景下，如果是发给裁判系统的长字符串，推荐使用 `USART_TRANSFER_DMA`。
 *         如果不慎使用了阻塞发送 `USART_TRANSFER_BLOCKING`，它内部的死循环会霸占 CPU，让整个外设卡顿。
 *
 * @param _instance 发送请求实例（记录发送用的huart等）
 * @param send_buf  需要发送的数据包指针
 * @param send_size 数据包大小
 * @param mode      底层传输模式：BLOCKING（阻塞轮询，效率最低）, IT（中断发送），DMA（DMA代劳，效率最高）
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        while (1)
            ; // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
        break;
    }
}

/* 串口发送时,gstate会被设为BUSY_TX */
uint8_t USARTIsReady(USARTInstance *_instance)
{
    if (_instance->usart_handle->gState | HAL_UART_STATE_BUSY_TX)
        return 0;
    else
        return 1;
}

/**
 * @brief  接收完成核心中断回调函数：当 DMA+IDLE 事件发生时，硬件自动调用此大门口。
 *
 * @note   工作原理：这是一扇守门大闸
 *         1. 当外部射频/视觉套件往单片机UART RX发来一堆数据后停下时（总线变为空闲IDLE）。
 *         2. CPU被打断，进入了这个HAL库预挖好的虚函数（__weak）。
 *         3. 这里使用 O(N) 遍历找到注册时候填的那个实体模块。
 *         4. 如果应用层绑定了 `module_callback`（比如裁判协议拆解），就触发它。
 *         5. 因为可能对方发的是可变长度的数据，这里要在处理完后，立刻把这段内存 `recv_buff` 重新用0复位，
 *            如果不清零，下一次来的包假如比上次短，就会残留下一次包尾部的脏数据。
 *         6. 最后也是最重要的：手动开启下一轮的DMA接收等待循环。否则串口就只能收一次，下次装死。
 *
 * @param huart 发生中断和接收的底层串口硬件指针
 * @param Size 本次DMA究竟收完了多少个字节（由于是IDLE中断，往往是一个短突发数据段的具体长度）
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle)
        { // call the callback function if it is not NULL
            if (usart_instance[i]->module_callback != NULL)
            {
                usart_instance[i]->module_callback();
                memset(usart_instance[i]->recv_buff, 0, Size); // 接收结束后清空buffer,对于变长数据是必要的
            }
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            return; // break the loop
        }
    }
}

/**
 * @brief UART 底层错误异常捕获抛出点
 *
 * @note  当通信线有干扰产生波特率不匹配、奇偶校验失效或溢出(Overflow)等物理错误时。
 *        STM32的硬件会封锁后续所有的收发请求（硬件死锁）。
 *        在这个错误抛出回调内，我们必须要暴力重启整个这路 DMA 收取通道，否则它直接永远断联了。
 *
 * @param huart 出现报错的串口句柄
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            LOGWARNING("[bsp_usart] USART error callback triggered, instance idx [%d]", i);
            return;
        }
    }
}