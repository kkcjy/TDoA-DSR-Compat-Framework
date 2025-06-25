#include "adhocuwb_platform_linux.h"
#include <stdlib.h>
#include <stdio.h>
apr_pool_t *pool;
uint16_t MY_UWB_ADDRESS;
SemaphoreHandle_t readyToSend;

//-----------------------------------------------------------------------------------------------------------------

/**
 * 功能：设置休眠的秒数
 */
void vTaskDelay(const TickType_t xTicksToDelay)
{
    usleep(1000000 * (xTicksToDelay));
}

/**
 * 功能：获取当前系统对应的时间，返回的是秒数
 * TODO:  使用毫秒
 */
TickType_t xTaskGetTickCount()
{
    struct timespec ts;
    // 获取当前系统时间
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
    {
        perror("clock_gettime");
        return 1;
    }
    // 打印当前系统时间
    // 时间为--秒+纳秒--
    // printf("Current system time: %ld seconds, %ld nanoseconds\n", ts.tv_sec, ts.tv_nsec);
    // 返回对应的秒数
    return ts.tv_sec;
}

/**
 * 功能：Queue队列为一个指针队列，里面存放的都是指针类型的数据
 *
 */
QueueHandle_t xQueueCreate(const uint32_t uxQueueLength,
                           const uint32_t uxItemSize)
{ // uxItemSize每个数据类型的长度
    apr_queue_t *queue;

    // 初始化 APR 库
    apr_initialize();
    atexit(apr_terminate);

    // 创建内存池
    apr_pool_create(&pool, NULL);

    // 创建队列
    apr_queue_create(&queue, uxQueueLength, uxItemSize, pool); // 修改源码后的apr_queue_create()函数

    // queue->item_size=uxItemSize;封装直接修改会报错
    return queue;
}

/**
 * 功能：销毁进程对应的内存池，直接释放内存池中的所有空间
 */
void xQueueDestroy(apr_pool_t *pool)
{

    apr_pool_destroy(pool);
    apr_terminate();
}

/**
 * 功能：实现往apr_queue_t队列之中插入一个指针
 */
BaseType_t xQueueSendFromISR(
    QueueHandle_t xQueue,
    const void *const pvItemToQueue, // 存放的是一级指针
    BaseType_t *const pxHigherPriorityTaskWoken)
{
    return xQueueSend(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken);
}

/**
 * 功能：实现往apr_queue_t队列之中插入一个指针
 * 注意：将传递进来的临时指针转换为堆内存指针存储在apr队列之中
 */
BaseType_t xQueueSend(
    QueueHandle_t xQueue,
    const void *const pvItemToQueue, // 存放的是一级指针
    BaseType_t *const pxHigherPriorityTaskWoken)
{
    unsigned int item_size = apr_queue_item_size(xQueue); // 表示apr_queue队列之中存放的元素的大小

    void *ptr = malloc(item_size);

    memcpy(ptr, pvItemToQueue, item_size);

    apr_queue_push(xQueue, ptr);

    return 1;
}

// 从队列之中获取一个元素
BaseType_t xQueueReceive(QueueHandle_t xQueue,
                         void *const pvBuffer, // pvBuffer指向一个Ranging_Message_With_Timestamp_t类型的数据
                         TickType_t xTicksToWait)
{
    // 实现free()函数的机制
    void *ptr;
    apr_queue_pop(xQueue, (void **)&ptr); // 传出的是元素地址（指针）的地址

    // 获取的ptr，可以*ptr找到对应的测距消息

    unsigned int item_size = apr_queue_item_size(xQueue);

    memcpy(pvBuffer, ptr, item_size);

    // 释放apr_queue队列之中的首地址
    free(ptr);

    return 1;
}

/**
 * 功能：创建一个互斥锁
 */
SemaphoreHandle_t xSemaphoreCreateMutex()
{

    pthread_mutex_t *mutex = malloc(sizeof(pthread_mutex_t)); // 在堆上分配内存
    if (mutex == NULL)
    {
        perror("Failed to allocate memory for mutex");
        return NULL; // 如果内存分配失败，返回NULL
    }
    if (pthread_mutex_init(mutex, NULL) != 0)
    {
        free(mutex); // 如果初始化失败，释放内存并返回NULL
        perror("fail to init mutex!\n");
        return NULL;
    }
    return mutex; // 返回新创建的互斥锁的指针
}
/**
 * 功能：实现加上互斥锁
 */
int xSemaphoreTake(SemaphoreHandle_t TfBufferMutex, TickType_t xTicksToWait)
{
    pthread_mutex_lock(TfBufferMutex);
}

/**
 * 功能：互斥锁的释放
 */
int xSemaphoreGive(SemaphoreHandle_t TfBufferMutex)
{
    pthread_mutex_unlock(TfBufferMutex);
}
/**
 * 功能：销毁一个互斥锁
 */
void xSemaphoreDestroyMutex(SemaphoreHandle_t mutex)
{
    if (mutex != NULL)
    {
        pthread_mutex_destroy(mutex); // 销毁互斥锁
        free(mutex);                  // 释放互斥锁所占用的内存
    }
    else
    {
        perror("mutex is NULL ,fail to free !\n");
    }
}

/**
 * 功能：定时器到期之后，执行的操作
 */
void alarm_handler(int sig)
{
    printf("Timer expired可以替换\n");
}

/**
 * 功能：创建一个定时器
 */
TimerHandle_t xTimerCreate()
{
    TimerHandle_t timer;
    return timer;
}

/**
 * 功能：设置一个定时器，当定时器超时时，它将触发一个信号
 */
long xTimerStart(TimerHandle_t timer, int expire_time, int repetition, void *func)
{
    // struct itimerval timer=xTimerCreate();
    struct sigaction sa;

    // 清空并设置信号处理函数
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = func;
    sigaction(SIGALRM, &sa, NULL);

    // 设置定时器的时间间隔
    timer.it_value.tv_sec = expire_time; // 定时器首次超时时间
    timer.it_value.tv_usec = 0;
    timer.it_interval.tv_sec = repetition; // 定时器周期性超时时间，设置为0表示单次定时器
    timer.it_interval.tv_usec = 0;

    return setitimer(ITIMER_REAL, &timer, NULL);
}

/**
 * 功能：实现创建一个线程，同时在一个进程之中join()等待线程的结束
 */
long xTaskCreate(void *task_funcion, void *args)
{
    pthread_t th;
    pthread_create(&th, NULL, task_funcion, args);
    printf("--xTaskCreate--\n");
    // pthread_join(th, NULL);

    return 1;
}
/**
 * 功能：获得16位的地址 TODO:需要重新修改下
 *
 */
uint16_t uwbGetAddress()
{
    return MY_UWB_ADDRESS;
}

/**
 * 功能：实现从消息之中读取timestamp
 *
 */
void dwt_readrxtimestamp(uint8_t *timestamp)
{
    for (int i = 0; i < 5; i++)
    {
        timestamp[i] = (rx_timestamp >> (8 * i)) & 0xFF;
    }
}

/**
 * 功能：实现从socket通信的测距消息之中读取tx_timestamp
 * 参数：txCallback()函数之中的变量的值
 * 说明：txCallback()函数之中，接收另外一个文件之中的变量的值
 */
void dwt_readtxtimestamp(uint8_t *timestamp)
{
    for (int i = 0; i < 5; i++)
    {
        timestamp[i] = (tx_timestamp >> (8 * i)) & 0xFF;
    }
}

/**
 * TODO:通过全局变量来实现2024-5-10
 * 功能：socket通信，实现rangingTxTask()函数之中swarm_ranging_proc将测距消息发送给control_center进程
 */
int uwbSendPacketBlock(Connection conn, UWB_Packet_t *packet)
{

    Socket_Packet_t responsePacket;
    memset(&responsePacket, 0, sizeof(responsePacket)); // 初始化responsePacket

    responsePacket.header.packetLength = sizeof(Socket_Packet_Header_t) + packet->header.length; // TODO:之前是strlen(packet)

    // responsePacket.header.packetLength = sizeof(Socket_Packet_Header_t) + strlen(responseMessage);
    memcpy(responsePacket.payload, packet, packet->header.length); // 复制响应消息到payload
    // strncpy(responsePacket.payload, responseMessage, strlen(responseMessage)); // 复制响应消息到payload

    // send back一接收到Packet，立即send给control_center进程
    send_packet(conn, &responsePacket, responsePacket.header.packetLength);
    printf("(swarm_ranging): send real message. \n");
    return 0;
}
