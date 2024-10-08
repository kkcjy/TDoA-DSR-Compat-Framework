#include "adhocuwb_impl.h"
#include "log.h"

#define CS_PIN DECK_GPIO_IO1

#define DEFAULT_RX_TIMEOUT 0xFFFFF

//以下是log.c中的部分
#define LOG_TYPE_MASK (0x0f)
static struct log_s * logs;
static int logsLen;
static uint32_t logsCrc;
static uint16_t logsCount = 0;

/* Possible variable types */
#define LOG_UINT8  1
#define LOG_UINT16 2
#define LOG_UINT32 3
#define LOG_INT8   4
#define LOG_INT16  5
#define LOG_INT32  6
#define LOG_FLOAT  7
#define LOG_FP16   8


void adhocuwb_get_velocity_init(){
	return;
}

void adhocuwb_get_velocity(float* velocityX, float* velocityY, float* velocityZ){
	*velocityX = 1.1;
	*velocityY = 22.22;
	*velocityZ = 333.333;
}

BaseType_t adhocuwb_xQueueSendFromISR(
    QueueHandle_t xQueue,
    const void * pvItemToQueue)
{
	BaseType_t retval;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Send data to the queue
	retval = xQueueSendFromISR(xQueue, pvItemToQueue, &xHigherPriorityTaskWoken);

	// Check if a higher priority task was woken and request a context switch if so
	// This macro requests a context switch to the higher priority task
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	return retval;
}

void adhocuwb_vTaskNotifyGiveFromISR(TaskHandle_t taskHandle) {
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(taskHandle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/* Public API to access log TOC from within the copter */
static logVarId_t invalidVarId = 0xffffu;


inline int logGetType(logVarId_t varid)
{
  return logs[varid].type & LOG_TYPE_MASK;
}

int logGetInt(logVarId_t varid)
{
  int valuei = 0;

  ASSERT(logVarIdIsValid(varid));

  switch(logGetType(varid))
  {
    case LOG_UINT8:
      valuei = *(uint8_t *)logs[varid].address;
      break;
    case LOG_INT8:
      valuei = *(int8_t *)logs[varid].address;
      break;
    case LOG_UINT16:
      valuei = *(uint16_t *)logs[varid].address;
      break;
    case LOG_INT16:
      valuei = *(int16_t *)logs[varid].address;
      break;
    case LOG_UINT32:
      valuei = *(uint32_t *)logs[varid].address;
      break;
    case LOG_INT32:
      valuei = *(int32_t *)logs[varid].address;
      break;
    case LOG_FLOAT:
      valuei = *(float *)logs[varid].address;
      break;
  }

  return valuei;
}

float logGetFloat(logVarId_t varid)
{
  ASSERT(logVarIdIsValid(varid));

  if (logGetType(varid) == LOG_FLOAT)
    return *(float *)logs[varid].address;

  return logGetInt(varid);
}


logVarId_t logGetVarId(const char* group, const char* name)
{
  int i;
  logVarId_t varId = invalidVarId;
  char * currgroup = "";

  for(i=0; i<logsLen; i++)
  {
    if (logs[i].type & LOG_GROUP) {
      if (logs[i].type & LOG_START) {
        currgroup = logs[i].name;
      }
    } else if ((!strcmp(group, currgroup)) && (!strcmp(name, logs[i].name))) {
      varId = (logVarId_t)i;
      return varId;
    }
  }

  return invalidVarId;
}


