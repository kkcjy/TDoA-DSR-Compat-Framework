#ifndef ADHOCUWB_SIMULATION_SUPPORT_H
#define ADHOCUWB_SIMULATION_SUPPORT_H


#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


#define         ASSERT                      assert
#define         DEBUG_PRINT                 printf
#define         DWT_TIME_UNITS              (1.0/499.2e6/128.0) 

typedef         pthread_mutex_t             *SemaphoreHandle_t;
typedef         uint32_t                    TickType_t;


SemaphoreHandle_t xSemaphoreCreateMutex();
void xSemaphoreDestroyMutex(SemaphoreHandle_t mutex);
int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t xTicksToWait);
int xSemaphoreGive(SemaphoreHandle_t mutex);

#endif