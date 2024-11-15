#ifndef __ADHOCUWB_IMPL_H__
#define __ADHOCUWB_IMPL_H__
#include <assert.h>
#include <cmsis_os.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "dwTypes.h"
#include "libdw3000.h"
#include "dw3000_cbll.h"

#define NO_DMA_CCM_SAFE_ZERO_INIT static
#define adhocuwb_readtxtimestamp dwt_readtxtimestamp
#define adhocuwb_readrxtimestamp dwt_readrxtimestamp
#define ASSERT assert
#define M2T(X) ((unsigned int)(X))
#define systemWaitStart() vTaskDelay(10)
#define LOG_GROUP_START(group)    // Nothing注释
#define LOG_GROUP_STOP(group)     // Nothing注释
#define LOG_ADD(type, name, value) // Nothing注释

#define ADHOC_UWB_TX_TASK_NAME "adhocuwbTxTask"
#define ADHOC_UWB_RANGING_TX_TASK_NAME "uwbRangingTxTask"
#define ADHOC_UWB_RANGING_RX_TASK_NAME "uwbRangingRxTask"
#define ADHOC_DECK_RANGING_TX_TASK_NAME "ADHOC_RANGING_TX"
#define ADHOC_DECK_RANGING_RX_TASK_NAME "ADHOC_RANGING_RX"
#define ADHOC_UWB_TASK_PRI osPriorityNormal
#define ADHOC_UWB_TASK_PRI osPriorityNormal
#define ADHOC_UWB_TASK_PRI osPriorityNormal
#define ADHOC_DECK_TASK_PRI     3


void adhocuwb_get_velocity_init();
void adhocuwb_get_velocity(float* velocityX, float* velocityY, float* velocityZ);

BaseType_t adhocuwb_xQueueSendFromISR(
    QueueHandle_t xQueue,
    const void * pvItemToQueue);


#endif
