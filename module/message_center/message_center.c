#include "message_center.h"
#include "stdlib.h"
#include "string.h"
#include "bsp_log.h"

/* message_center是fake head node,是方便链表编写的技巧,这样就不需要处理链表头的特殊情况 */
static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .first_subs = NULL,
    .next_topic_node = NULL};

static void CheckName(char *name)
{
    if (strnlen(name, MAX_TOPIC_NAME_LEN + 1) >= MAX_TOPIC_NAME_LEN)
    {
        LOGERROR("EVENT NAME TOO LONG:%s", name);
        while (1)
            ; // 进入这里说明话题名超出长度限制
    }
}

static void CheckLen(uint8_t len1, uint8_t len2)
{
    if (len1 != len2)
    {
        LOGERROR("EVENT LEN NOT SAME:%d,%d", len1, len2);
        while (1)
            ; // 进入这里说明相同话题的消息长度却不同
    }
}

/**
 * @brief  PubRegister: 发布者(某个数据或指令的源头)向系统注册一个“话题”(Topic)。
 *
 * @note   工作原理：这是 Pub-Sub 架构的核心构建块。
 *         比如 Robot_Cmd 任务想要对外广播 “底盘速度控制量”。它并不需要知道到底是哪个C文件里的底盘任务在运行。
 *         它只要调用这个函数，说：“我要建立一个名叫 'cmd_chassis' 的话题，我每次发的数据包长度是 16 个字节”。
 *         函数会从 message_center 虚拟头节点开始遍历纵向单链表：
 *         1. 如果这个话题名存在了（可能被别人先注册订阅了），就补充返回话题的指针给调用方。
 *         2. 如果不存在，就在链表尾部再 `malloc` 画出一个新的 Topic 节点。
 *          这样一纵列的话题名就建立起来了。
 *
 * @param  name      话题的唯一字符串名称
 * @param  data_len  这一个话题每次通信交互的字节长度（如 sizeof(Chassis_Cmd_s)）
 * @retval Publisher_t* 返回指向该话题大节点（纵向节点）的指针，以后用来塞数据。
 */
Publisher_t *PubRegister(char *name, uint8_t data_len)
{
    CheckName(name);
    Publisher_t *node = &message_center;
    while (node->next_topic_node) // message_center会直接跳过,不需要特殊处理,这被称作dumb_head(编程技巧)
    {
        node = node->next_topic_node;            // 切换到下一个发布者(话题)结点
        if (strcmp(node->topic_name, name) == 0) // 如果已经注册了相同的话题,直接返回结点指针
        {
            CheckLen(data_len, node->data_len);
            node->pub_registered_flag = 1;
            return node;
        }
    } // 遍历完发现尚未创建name对应的话题
    // 在链表尾部创建新的话题并初始化
    node->next_topic_node = (Publisher_t *)malloc(sizeof(Publisher_t));
    memset(node->next_topic_node, 0, sizeof(Publisher_t));
    node->next_topic_node->data_len = data_len;
    strcpy(node->next_topic_node->topic_name, name);
    node->pub_registered_flag = 1;
    return node->next_topic_node;
}

/**
 * @brief  SubRegister: 订阅侧(想要别人数据的模块) 在指定话题名下面，开辟自己的专属“收货信箱队列”。
 *
 * @note   工作原理：这构成了 Pub-Sub 的横向链表。
 *         1. 先看看有没有关于 `name` 这个话题（Publisher_t），如果没有也连带创一个。
 *         2. 为自己这个模块分配一个 `Subscriber_t` 的盒子实例。并在这个盒子里，分配 `QUEUE_SIZE`(比如3个长度) 的接收缓冲数组。
 *         3. 然后把它自己挂载到这个对应话题所在的“横向专属订阅链表”尾部 (`next_subs_queue`)。
 *         这样：横向所有的邮箱结构体（`Subscriber_t`）就串联在了某一个 `Publisher_t` 的麾下。
 *
 * @param  name     你要接听的话题名字
 * @param  data_len 这个话题下这包数据原本有多大，以此向系统索要相应的 Malloc 内存来存缓冲。
 */
Subscriber_t *SubRegister(char *name, uint8_t data_len)
{
    Publisher_t *pub = PubRegister(name, data_len); // 查找或创建该话题的发布者
    // 创建新的订阅者结点,申请内存,注意要memset因为新空间不一定是空的,可能有之前留存的垃圾值
    Subscriber_t *ret = (Subscriber_t *)malloc(sizeof(Subscriber_t));
    memset(ret, 0, sizeof(Subscriber_t));
    // 对新建的Subscriber进行初始化
    ret->data_len = data_len; // 设定数据长度
    for (size_t i = 0; i < QUEUE_SIZE; ++i)
    { // 给消息队列的每一个元素分配空间,queue里保存的实际上是数据执指针,这样可以兼容不同的数据长度
        ret->queue[i] = malloc(data_len);
    }
    // 如果是第一个订阅者,特殊处理一下,将first_subs指针指向新建的订阅者(详见文档)
    if (pub->first_subs == NULL)
    {
        pub->first_subs = ret;
        return ret;
    }
    // 若该话题已经有订阅者, 遍历订阅者链表,直到到达尾部
    Subscriber_t *sub = pub->first_subs; // 作为iterator
    while (sub->next_subs_queue)         // 遍历订阅了该话题的订阅者链表
    {
        sub = sub->next_subs_queue; // 移动到下一个订阅者,遇到空指针停下,说明到了链表尾部
    }
    sub->next_subs_queue = ret; // 把刚刚创建的订阅者接上
    return ret;
}

/**
 * @brief  SubGetMessage: 订阅者从自己的专属缓冲区里把别人发来的数据包“取出来”( Pop 操作)。
 *
 * @note   工作原理：这是一个使用取模 `% QUEUE_SIZE` 构成的最经典的“环形FIFO队列”。
 *         如果当前这个话题真的有数据(`temp_size > 0`)，
 *         就从 `front_idx`(最新的队列读出指针) 所在的地方 `memcpy` 拷贝出来一份。
 *         所以这个函数既是阻塞，也是非阻塞的：你可以配合 FreeRTOS 轮询它，如果别人没发，就返回 0 让你继续睡觉。
 *
 * @param  sub        调用刚才注册时返回的那个 `Subscriber_t` 对象（自己的私有信箱）
 * @param  data_ptr   准备用来承接这几十、几十个字节的一级或者多级结构体容器指针。
 * @retval uint8_t    取出成功返回 1；没有新数据信箱空空如也返回 0。
 */
/* 如果队列为空,会返回0;成功获取数据,返回1;后续可以做更多的修改,比如剩余消息数目等 */
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
    if (sub->temp_size == 0)
    {
        return 0;
    }
    memcpy(data_ptr, sub->queue[sub->front_idx], sub->data_len);
    sub->front_idx = (sub->front_idx++) % QUEUE_SIZE; // 队列头索引增加
    sub->temp_size--;                                 // pop一个数据,长度减1
    return 1;
}

/**
 * @brief  PubPushMessage: 由数据的源头模块调用，将整理好的报文信息推送到所有监听这话题的信箱。
 *
 * @note   工作逻辑：这是整个多任务发布订阅的灵魂广播动作。
 *         当某个话题的源头 `pub` （比如 Robot_cmd 计算好了下发指令后调用它），
 *         它不关心有几个底盘、几个发射云台连了它。它只做一件事：
 *         顺着 `next_subs_queue` 这个单链表，摸出一个接一个的接收方（`Subscriber_t`结构体）。
 *         把自身发出来的这帧总字节（`data_ptr`） ，挨个 `memcpy` 复制塞进它们每个人各自的私有接收缓冲队列(`queue`) 最后 (`back_idx`)。
 *         如果对方太忙来不及收满塞爆了怎么办？最老的包会被直接循环覆盖（抛弃）。
 *
 * @param  pub       你作为发布源所在的话题 `Publisher_t` 节点
 * @param  data_ptr  这帧消息(比如 16字节的结构体指针)的来源地址。
 * @retval 恒返回1
 */
uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr)
{
    static Subscriber_t *iter;
    iter = pub->first_subs; // iter作为订阅者指针,遍历订阅该话题的所有订阅者;如果为空说明遍历结束
    // 遍历订阅了当前话题的所有订阅者,依次填入最新消息
    while (iter)
    {
        if (iter->temp_size == QUEUE_SIZE) // 如果队列已满,则需要删除最老的数据(头部),再填入
        {
            // 队列头索引前移动,相当于抛弃前一个位置的数据,被抛弃的位置稍后会被写入新的数据
            iter->front_idx = (iter->front_idx + 1) % QUEUE_SIZE;
            iter->temp_size--; // 相当于出队,size-1
        }
        // 将Pub的数据复制到队列的尾部(最新)
        memcpy(iter->queue[iter->back_idx], data_ptr, pub->data_len);
        iter->back_idx = (iter->back_idx + 1) % QUEUE_SIZE; // 队列尾部前移
        iter->temp_size++;                                  // 入队,size+1

        iter = iter->next_subs_queue; // 访问下一个订阅者
    }
    return 1;
}