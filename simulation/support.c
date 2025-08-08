#include "support.h"

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

