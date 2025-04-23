#ifndef __ADHOCUWB_PLATFORM_CRAZYFLIE_H__
#define __ADHOCUWB_PLATFORM_CRAZYFLIE_H__

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "system.h"
#include "debug.h"

#define ADHOC_UWB_TX_TASK_NAME "adhocuwbTxTask"
#define ADHOC_UWB_RANGING_TX_TASK_NAME "uwbRangingTxTask"
#define ADHOC_UWB_RANGING_RX_TASK_NAME "uwbRangingRxTask"
#define ADHOC_UWB_TASK_PRI 3

#define spiDeckEndTransaction spiEndTransaction
#define spiDeckExchange spiExchange

void adhocuwb_vTaskNotifyGiveFromISR(TaskHandle_t taskHandle);
BaseType_t adhocuwb_xQueueSendFromISR( QueueHandle_t xQueue, const void * pvItemToQueue);

#endif