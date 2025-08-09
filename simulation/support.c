#include "support.h"


uint16_t TxCount = 0;
uint16_t RxCount = 0;
dwTime_t lastTxTimestamp;                               
dwTime_t lastRxTimestamp;                               
dwTime_t TxTimestamp;                                   
dwTime_t RxTimestamp;                                   


/* SemaphoreHandle_t */
SemaphoreHandle_t xSemaphoreCreateMutex() {
    pthread_mutex_t *mutex = malloc(sizeof(pthread_mutex_t));
    if (mutex == NULL) {
        perror("Failed to allocate memory for mutex");
        return NULL;
    }
    if (pthread_mutex_init(mutex, NULL) != 0) {
        free(mutex);
        perror("fail to init mutex!\n");
        return NULL;
    }
    return mutex;
}

void xSemaphoreDestroyMutex(SemaphoreHandle_t mutex) {
    if (mutex != NULL) {
        pthread_mutex_destroy(mutex);
        free(mutex);
    }
    else {
        perror("mutex is NULL ,fail to free !\n");
    }
}

int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t xTicksToWait) {
    return pthread_mutex_lock(mutex);
}

int xSemaphoreGive(SemaphoreHandle_t mutex) {
    return pthread_mutex_unlock(mutex);
}

/* SemaphoreHandle_t */
TickType_t xTaskGetTickCount() {
    TickType_t curTicks;
    if(TxTimestamp.full != 0) {
        if(lastTxTimestamp.full == 0) {
            lastTxTimestamp.full = TxTimestamp.full;
        }
        else {
            TxCount += TxTimestamp.full < lastTxTimestamp.full ? 1 : 0;
        }
        curTicks = (TxCount << 16) + (TxTimestamp.full >> 24);
        TxTimestamp.full = 0;
    }
    else if(RxTimestamp.full != 0) {
        if(lastRxTimestamp.full == 0) {
            lastRxTimestamp.full = RxTimestamp.full;
        }
        else {
            RxCount += RxTimestamp.full < lastRxTimestamp.full ? 1 : 0;
        }
        curTicks = (RxCount << 16) + (RxTimestamp.full >> 24);
        RxTimestamp.full = 0;
    }
    return curTicks;
}