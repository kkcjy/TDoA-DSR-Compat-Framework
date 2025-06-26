#ifndef __ADHOCUWB_PLATFORM_ADHOCUWBH7_H__
#define __ADHOCUWB_PLATFORM_ADHOCUWBH7_H__

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

#define DEBUG_PRINT printf
#define NO_DMA_CCM_SAFE_ZERO_INIT static
#define adhocuwb_readtxtimestamp dwt_readtxtimestamp
#define ASSERT assert
#define M2T(X) ((unsigned int)(X))
#define systemWaitStart() vTaskDelay(10)
#define LOG_GROUP_START(group)    // Nothing
#define LOG_GROUP_STOP(group)     // Nothing
#define LOG_ADD(type, name, value) // Nothing

#define ADHOC_UWB_TX_TASK_NAME "adhocuwbTxTask"
#define ADHOC_UWB_RANGING_TX_TASK_NAME "uwbRangingTxTask"
#define ADHOC_UWB_RANGING_RX_TASK_NAME "uwbRangingRxTask"
#define ADHOC_UWB_TASK_PRI osPriorityNormal

/* 涉及到DW3000模组选择模式，参考如下文档：
 * 1. https://seunetsi.feishu.cn/docx/Nlu9d3ndgoFnPixKRMTcsR1Yn7b , Drive DW3000 for Atherna
 * 2. https://seunetsi.feishu.cn/wiki/wikcnB0VX2BpLy8xW6eOoYOuDih , DWM3000 驱动实现
 */
#define CONFIG_ADHOCDECK_USE_UART1_PINS
//#define CONFIG_ADHOCDECK_USE_UART2_PINS
//#define CONFIG_ADHOCDECK_USE_ALT_PINS
// TODO: rename ADHOCUWBH7 to ADW3KH7C for H7 and DW3000 in the same PCB

void adhocuwb_get_velocity_init();
void adhocuwb_get_velocity(float* velocityX, float* velocityY, float* velocityZ);

BaseType_t adhocuwb_xQueueSendFromISR(
    QueueHandle_t xQueue,
    const void * pvItemToQueue);

#endif
