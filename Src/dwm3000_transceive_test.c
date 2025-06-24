#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "adhocuwb_init.h"
#include "adhocuwb_platform.h"
#include "adhocuwb_swarm_ranging.h"

#ifndef UWB_DEBUG_ENABLE
  #undef DEBUG_PRINT
  #define DEBUG_PRINT 
#endif

TaskHandle_t uwbTransceiveTaskHandle = 0;

void simpleTxCallback(void *argument) {	// 发送完数据包后的回调函数
	DEBUG_PRINT("simpleTxCallback\n");
	return;
}

void simpleRxCallback(void *argument) {	// 接收到数据包时的回调函数
	uint32_t *packet = (uint32_t *) argument;
	uint32_t a = packet[0];
	uint32_t b = packet[1];
	DEBUG_PRINT("receive: %lx,%lx\n", a, b);
	return;
}

void uwbTransceiveTask()
{
	systemWaitStart();

//	int uwbdata_tx[10] = {1,2,3,4,5,6,7,8,9};
	static uint32_t uwbdata_tx[2] = {0,0x5E02E751};
	// loop forever
	while(1)
	{
	    adhocuwb_hdw_send(uwbdata_tx, 8);
		DEBUG_PRINT("send: %lx,%lx\n", uwbdata_tx[0], uwbdata_tx[1]);
        vTaskDelay(500);
	    uwbdata_tx[0]++;
		NVIC_SetPendingIRQ(EXTI9_5_IRQn);
//		NVIC_SetPendingIRQ(EXTI15_10_IRQn);
	}
}

void uwbTransceiveInit(){
	adhocuwb_set_hdw_cbs(simpleTxCallback, simpleRxCallback);
    xTaskCreate(uwbTransceiveTask, ADHOC_UWB_TX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
                ADHOC_UWB_TASK_PRI, &uwbTransceiveTaskHandle);
}
