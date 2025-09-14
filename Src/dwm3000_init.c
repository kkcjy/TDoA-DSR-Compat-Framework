#include <string.h>

#ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7
  #include "stm32h7xx.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE)
  #include "stm32fxxx.h"
#endif

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "dwTypes.h"
#include "libdw3000.h"
#include "dw3000.h"
#include "dwm3000_init.h"
#include "dwm3000_transceive.h"
#include "adhocuwb_platform.h"
#include "adhocuwb_init.h"

#if defined(CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7) || defined(CONFIG_ADHOCUWB_PLATFORM_ATHENA)
  #include "main.h"
  static uint16_t MY_UWB_ADDRESS;
  #define DEBUG_PRINT printf
#endif

#ifdef CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
  #include "deck.h"
  #include "param.h"
#endif

#if defined(CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7) || defined(CONFIG_ADHOCUWB_PLATFORM_ATHENA)
  static uint16_t MY_UWB_ADDRESS;
  static bool isInit = false;
  static TaskHandle_t uwbISRTaskHandle = 0;
  static SemaphoreHandle_t uwbIrqSemaphore;
#endif

/* PHR configuration */
static dwt_config_t uwbPhrConfig = {
    5,            /* Channel number. */
    DWT_PLEN_128, /* Preamble length. Used in TX only. */
    DWT_PAC8,     /* Preamble acquisition chunk size. Used in RX only. */
    9,            /* TX preamble code. Used in TX only. */
    9,            /* RX preamble code. Used in RX only. */
    1,            /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for
                     non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,   /* Data rate. */
#ifdef UWB_ENABLE_PHR_EXT_MODE
    DWT_PHRMODE_EXT, /* Extended PHY header mode. */
#else
    DWT_PHRMODE_STD, /* Standard PHY header mode. */
#endif
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size).
                        Used in RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64, /* STS length, see allowed values in Enum dwt_sts_lengths_e
                     */
    DWT_PDOA_M0     /* PDOA mode off */
};

static dwt_txconfig_t uwbTxConfigOptions = {
    .PGcount = 0x0,
    .PGdly = 0x34,
    .power = 0xfdfdfdfd
};

#ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7
  #define DW3000Deck_Enable()          LL_GPIO_ResetOutputPin(DW3000Deck_CS_GPIO_Port, DW3000Deck_CS_Pin)
  #define DW3000Deck_Disable()         LL_GPIO_SetOutputPin(DW3000Deck_CS_GPIO_Port, DW3000Deck_CS_Pin)
#endif
#ifdef CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
  #define CS_PIN DECK_GPIO_IO1
  // LOCO deck alternative IRQ and RESET pins(IO_2, IO_4) instead of default (RX1, TX1), leaving UART1 free for use
  #ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
    #define GPIO_PIN_IRQ      DECK_GPIO_IO2
    #ifndef ADHOCDECK_ALT_PIN_RESET
      #define GPIO_PIN_RESET    DECK_GPIO_IO4
    #else
      #define GPIO_PIN_RESET 	ADHOCDECK_ALT_PIN_RESET
    #endif
    #define EXTI_PortSource EXTI_PortSourceGPIOB
    #define EXTI_PinSource    EXTI_PinSource5
    #define EXTI_LineN          EXTI_Line5
  #elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
    #define GPIO_PIN_IRQ 	  DECK_GPIO_TX2
    #define GPIO_PIN_RESET 	DECK_GPIO_RX2
    #define EXTI_PortSource EXTI_PortSourceGPIOA
    #define EXTI_PinSource 	EXTI_PinSource2
    #define EXTI_LineN 		  EXTI_Line2
  #else
    #define GPIO_PIN_IRQ      DECK_GPIO_RX1
    #define GPIO_PIN_RESET    DECK_GPIO_TX1
    #define EXTI_PortSource   EXTI_PortSourceGPIOC
    #define EXTI_PinSource    EXTI_PinSource11
    #define EXTI_LineN        EXTI_Line11
  #endif
#endif

static adhocuwb_hdw_cb_t _txCallback = 0;
static adhocuwb_hdw_cb_t _rxCallback = 0;


/************ Low level ops for libdw **********/

#define SPI_DECK_BUFFER_MAX_SIZE 300
static uint8_t spiDeckTxBuffer[SPI_DECK_BUFFER_MAX_SIZE];
static uint8_t spiDeckRxBuffer[SPI_DECK_BUFFER_MAX_SIZE];
#ifdef CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
  static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ;
#endif

static void spiDeckWrite(const void* cmd,
			size_t cmdLength,
			const void *data,
			size_t dataLength)
{
  #ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7
	  spiDeckBeginTransaction();
	  DW3000Deck_Enable();
  #endif
  #ifdef CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
    spiBeginTransaction(spiSpeed);
	  digitalWrite(CS_PIN, LOW);
  #endif 
  memcpy(spiDeckTxBuffer, cmd, cmdLength);
  memcpy(spiDeckTxBuffer + cmdLength, data, dataLength);
  spiDeckExchange(cmdLength + dataLength, spiDeckTxBuffer, spiDeckRxBuffer);
  #ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7
    DW3000Deck_Disable();
  #endif
  #ifdef CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
    digitalWrite(CS_PIN, HIGH);
  #endif 
	spiDeckEndTransaction();
}

static void spiDeckRead(const void* cmd,
			size_t cmdLength,
			void *data,
			size_t dataLength)
{
  #ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7
	  spiDeckBeginTransaction();
	  DW3000Deck_Enable();
  #endif
  #ifdef CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
    spiBeginTransaction(spiSpeed);
	digitalWrite(CS_PIN, LOW);
  #endif 
	memcpy(spiDeckTxBuffer, cmd, cmdLength);
	memset(spiDeckTxBuffer + cmdLength, DUMMY_BYTE, dataLength);
	spiDeckExchange(cmdLength + dataLength, spiDeckTxBuffer, spiDeckRxBuffer);
	memcpy(data, spiDeckRxBuffer + cmdLength, dataLength);
  #ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7
    DW3000Deck_Disable();
  #endif
  #ifdef CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
    digitalWrite(CS_PIN, HIGH);
  #endif 
	spiDeckEndTransaction();
}

#ifdef CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
void __attribute__((used)) EXTI5_Callback(void)
#elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
void __attribute__((used)) EXTI2_Callback(void)
#else
void __attribute__((used)) EXTI11_Callback(void)
#endif
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Unlock interrupt handling task
 if(uwbISRTaskHandle)
   vTaskNotifyGiveFromISR(uwbISRTaskHandle, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken) {
    portYIELD();
  }
}
#endif

static void spiDeckSetSpeed(dwSpiSpeed_t speed) { return; }

static void delayms(unsigned int delay) { vTaskDelay(delay); }

static void reset(void)
{
  #ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7
	#ifdef CONFIG_ADHOCDECK_USE_UART1_PINS
	  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15);
	  vTaskDelay(M2T(10));
	  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15);
	  vTaskDelay(M2T(10));
	#elif defined(CONFIG_ADHOCDECK_USE_UART2_PINS)
	  LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_8);
	  vTaskDelay(M2T(10));
	  LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_8);
	  vTaskDelay(M2T(10));
	#elif defined(CONFIG_ADHOCDECK_USE_ALT_PINS)
	  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
	  vTaskDelay(M2T(10));
	  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
	  vTaskDelay(M2T(10));
    #endif
  #endif

  #ifdef CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
    digitalWrite(GPIO_PIN_RESET, LOW);
    vTaskDelay(M2T(10));
    digitalWrite(GPIO_PIN_RESET, HIGH);
    vTaskDelay(M2T(10));
  #endif 
}

dwOps_t dwt_ops = {
    .spiRead = spiDeckRead,
    .spiWrite = spiDeckWrite,
    .spiSetSpeed = spiDeckSetSpeed,
    .delayms = delayms,
    .reset = reset
};

/************ Callback functions for libdw **********/
static uint8_t rxBuffer[UWB_FRAME_LEN_MAX];

static void txCallback()
{
//   DEBUG_PRINT("txCallback \n");
	if(_txCallback)	{
		_txCallback(NULL);
	}
}

static void rxCallback()
{
  uint32_t dataLength = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;

//  ASSERT(dataLength != 0 && dataLength <= UWB_FRAME_LEN_MAX);

  dwt_readrxdata(rxBuffer, dataLength - FCS_LEN, 0); /* No need to read the FCS/CRC. */

//  DEBUG_PRINT("rxCallback: data length = %lu \n", dataLength);
	if(_rxCallback)	{
		_rxCallback(rxBuffer); //TODO: make this function pass both buffer address and dataLength
	}
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rxTimeoutCallback()
{
//  DEBUG_PRINT("rxTimeoutCallback \n");
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rxErrorCallback()
{
//  DEBUG_PRINT("rxErrorCallback \n");
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

int dw3000_init()
{
  /* Need to make sure DW IC is in IDLE_RC before proceeding */
  for (int i = 0; !dwt_checkidlerc() && i < 3; i++)
  {
  }

  if (!dwt_checkidlerc())
  {
//    DEBUG_PRINT("Error: DW IC is not in IDLE_RC state \n");
    return DWT_ERROR;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    return DWT_ERROR;
  }

  if (dwt_configure(&uwbPhrConfig) == DWT_ERROR)
  {
    return DWT_ERROR;
  }

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&uwbTxConfigOptions);

  /* Configure Antenna Delay */
  dwt_setrxantennadelay(UWB_RX_ANT_DLY);
  dwt_settxantennadelay(UWB_TX_ANT_DLY);

  dwt_setrxtimeout(0xFFFFF);

  /* Set callback functions */
  dwt_setcallbacks(&txCallback, &rxCallback, &rxTimeoutCallback, &rxErrorCallback, NULL, NULL);

  /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
  dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK,
                   0, DWT_ENABLE_INT);

  /* Clearing the SPI ready interrupt */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);

  uwbIrqSemaphore = xSemaphoreCreateMutex();

  return DWT_SUCCESS;
}

uint16_t uwbGetAddress() {
  return MY_UWB_ADDRESS;
}

void uwbISRTask(void *parameters) {
  systemWaitStart();

  while (1)
  {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
    {
      do
      {
        xSemaphoreTake(uwbIrqSemaphore, portMAX_DELAY);
        dwt_isr();
        xSemaphoreGive(uwbIrqSemaphore);
      } 
      #ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7
		#ifdef CONFIG_ADHOCDECK_USE_UART1_PINS
      	  while (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) != RESET);
		#elif defined(CONFIG_ADHOCDECK_USE_UART2_PINS)
  	  	  while (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) != RESET);
  	  #elif defined(CONFIG_ADHOCDECK_USE_ALT_PINS)
		  while (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15) != RESET);
		#endif
      #endif
      #ifdef CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
        while (digitalRead(GPIO_PIN_IRQ) != 0);
      #endif
    }
  }
}

void adhocuwb_hdw_force_rx() {
	dwt_forcetrxoff();					 //关闭所有的发送和接收功能
	dwt_rxenable(DWT_START_RX_IMMEDIATE);//立即开始接收
}

int adhocuwb_hdw_send(void *data, uint32_t datalen) {
	dwt_forcetrxoff();
	dwt_writetxdata(datalen, (uint8_t *) data, 0);
	dwt_writetxfctrl(datalen + FCS_LEN, 0, 1);
	return (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_ERROR);
}

void adhocuwb_set_hdw_cbs(adhocuwb_hdw_cb_t txCb, adhocuwb_hdw_cb_t rxCb) {
	_rxCallback = rxCb;
	_txCallback = txCb;
}

#ifdef CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
static void pinInit() {
  EXTI_InitTypeDef EXTI_InitStructure;

  spiBegin();

  // Set up interrupt
  SYSCFG_EXTILineConfig(EXTI_PortSource, EXTI_PinSource);

  EXTI_InitStructure.EXTI_Line = EXTI_LineN;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
  DEBUG_PRINT("USE_ALT_PINS\n");
#elif CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  DEBUG_PRINT("USE_UART2_PINS\n");
#else
  DEBUG_PRINT("USE_UART1_PINS\n");
#endif
  // Init pins
#ifdef CONFIG_DECK_ADHOCDECK_USE_UART2_PINS
  pinMode(CS_PIN, OUTPUT);
//  pinMode(GPIO_PIN_RESET, OUTPUT); TODO: magic, don't know why, need further debugging
  pinMode(GPIO_PIN_IRQ, INPUT);
#else
  pinMode(CS_PIN, OUTPUT);
  pinMode(GPIO_PIN_RESET, OUTPUT);
  pinMode(GPIO_PIN_IRQ, INPUT);
#endif
  //Reset the DW3000 chip
  dwt_ops.reset();
}

/*********** Deck driver initialization ***************/
static void dwm3000_adhocuwb_Init(DeckInfo *info) {

  pinInit();
  if (dw3000_init() == DWT_SUCCESS) {
    xTaskCreate(uwbISRTask, ADHOC_DECK_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
      ADHOC_DECK_TASK_PRI, &uwbISRTaskHandle);
    adhocuwbInit();
    // uwbTransceiveInit();
    isInit = true;
  } else {
    isInit = false;
  }
  DEBUG_PRINT("MY_UWB_ADDRESS = %d \n", MY_UWB_ADDRESS);
}

static bool dwm3000_adhocuwb_Test() {
  if (!isInit) {
    DEBUG_PRINT("Error while initializing DWM3000_ADHOCUWB\n");
  }

  return isInit;
}

static const DeckDriver dwm3000_adhocuwb_deck = {
    .vid = 0xBC,
    .pid = 0x06,
    .name = "DWM3000_ADHOCUWB",

#ifdef CONFIG_DECK_ADHOCDECK_USE_ALT_PINS
    .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_2 | DECK_USING_IO_4,
#elif defined(CONFIG_DECK_ADHOCDECK_USE_UART2_PINS)
    .usedGpio = DECK_USING_IO_1 | DECK_USING_UART2,
#else
    .usedGpio = DECK_USING_IO_1 | DECK_USING_UART1,
#endif
    .usedPeriph = DECK_USING_SPI,
#ifdef ADHOCDECK_NO_LOW_INTERFERENCE
    .requiredLowInterferenceRadioMode = false,
#else
    .requiredLowInterferenceRadioMode = true,
#endif

    .init = dwm3000_adhocuwb_Init,
    .test = dwm3000_adhocuwb_Test,
};

DECK_DRIVER(dwm3000_adhocuwb_deck);

PARAM_GROUP_START(deck)
        PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, DWM3000, &isInit)
PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(ADHOC)
        PARAM_ADD_CORE(PARAM_UINT16 | PARAM_PERSISTENT, MY_UWB_ADDRESS, &MY_UWB_ADDRESS)
PARAM_GROUP_STOP(ADHOC)
#endif
