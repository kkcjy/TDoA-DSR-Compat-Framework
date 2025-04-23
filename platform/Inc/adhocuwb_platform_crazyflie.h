#ifndef __ADHOCUWB_PLATFORM_CRAZYFLIE_H__
#define __ADHOCUWB_PLATFORM_CRAZYFLIE_H__

#include "system.h"
#include "debug.h"

#define adhocuwb_vTaskNotifyGiveFromISR vTaskNotifyGiveFromISR
#define adhocuwb_xQueueSendFromISR xQueueSendFromISR

#define ADHOC_UWB_TX_TASK_NAME "adhocuwbTxTask"
#define ADHOC_UWB_RANGING_TX_TASK_NAME "uwbRangingTxTask"
#define ADHOC_UWB_RANGING_RX_TASK_NAME "uwbRangingRxTask"



#endif