#ifndef __DWM3000_INIT_H__
#define __DWM3000_INIT_H__

#ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWB
  #include "stm32h7xx_ll_gpio.h"
  #include "spi_drv.h"
#endif
#include "libdw3000.h"

/* Function Switch */
#define UWB_ENABLE_PHR_EXT_MODE

#define UWB_SPEED_OF_LIGHT 299702547
#define UWB_MAX_TIMESTAMP 1099511627776  // 2**40
#define UWB_TX_ANT_DLY 16385
#define UWB_RX_ANT_DLY 16385

#define UWB_FRAME_LEN_STD 127
#define UWB_FRAME_LEN_EXT 256
//#define UWB_FRAME_LEN_EXT 1024

#define UWB_TASK_STACK_SIZE (2 * UWB_FRAME_LEN_MAX)

#ifdef UWB_ENABLE_PHR_EXT_MODE
  #define UWB_FRAME_LEN_MAX UWB_FRAME_LEN_EXT
#else
  #define UWB_FRAME_LEN_MAX UWB_FRAME_LEN_STD
#endif

#ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWB
  #define DW3000Deck_CS_GPIO_Port GPIOE
  #define DW3000Deck_CS_Pin LL_GPIO_PIN_2
#endif

#define DUMMY_BYTE 0x00

#ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWB
  extern SemaphoreHandle_t spiDeckTxComplete;
  extern SemaphoreHandle_t spiDeckRxComplete;
  extern SemaphoreHandle_t spiDeckMutex;
  extern SemaphoreHandle_t uwbIrqSemaphore;
#endif 

typedef void (*adhocuwb_hdw_cb_t)(void *);


void uwbISRTask(void *parameters);
int dw3000_init();
void adhocuwb_set_hdw_cbs(adhocuwb_hdw_cb_t txCb, adhocuwb_hdw_cb_t rxCb);
void adhocuwb_hdw_force_rx();
int adhocuwb_hdw_send(void *data, uint32_t datalen);

#endif /* __DW3000_CBLL_H__ */
