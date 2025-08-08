// #define _POSIX_C_SOURCE 199309L
#include "support.h"


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
// static void timer_signal_handler(union sigval sv) {
//     Timer_t* timer = (Timer_t*) sv.sival_ptr;
//     if (timer && timer->callback) {
//         timer->callback(timer->id);
//     }
// }

// TimerHandle_t xTimerCreate(const char* pcTimerName,
//                            TickType_t xTimerPeriodInTicks,
//                            UBaseType_t uxAutoReload,
//                            void* pvTimerID,
//                            TimerCallbackFunction_t pxCallbackFunction) {
//     Timer_t* newTimer = (Timer_t*) malloc(sizeof(Timer_t));
//     if (!newTimer) {
//         perror("malloc");
//         return NULL;
//     }
//     memset(newTimer, 0, sizeof(Timer_t));

//     strncpy(newTimer->name, pcTimerName, sizeof(newTimer->name) - 1);
//     newTimer->periodTicks = xTimerPeriodInTicks;
//     newTimer->autoReload = uxAutoReload;
//     newTimer->id = pvTimerID;
//     newTimer->callback = pxCallbackFunction;

//     struct sigevent sev;
//     memset(&sev, 0, sizeof(sev));
//     sev.sigev_notify = SIGEV_THREAD;
//     sev.sigev_value.sival_ptr = newTimer;
//     sev.sigev_notify_function = timer_signal_handler;

//     if (timer_create(CLOCK_REALTIME, &sev, &newTimer->posixTimerID) == -1) {
//         perror("timer_create");
//         free(newTimer);
//         return NULL;
//     }
//     return newTimer;
// }

// int xTimerStart(TimerHandle_t xTimer, TickType_t xTicksToWait) {
//     Timer_t* timer = (Timer_t*) xTimer;
//     if (!timer) return 0;

//     struct itimerspec its;
//     memset(&its, 0, sizeof(its));

//     its.it_value.tv_sec = timer->periodTicks / TICKS_PER_SECOND;
//     its.it_value.tv_nsec = (timer->periodTicks % TICKS_PER_SECOND) * (1000000000 / TICKS_PER_SECOND);

//     if (timer->autoReload) {
//         its.it_interval.tv_sec = its.it_value.tv_sec;
//         its.it_interval.tv_nsec = its.it_value.tv_nsec;
//     } else {
//         its.it_interval.tv_sec = 0;
//         its.it_interval.tv_nsec = 0;
//     }

//     if (timer_settime(timer->posixTimerID, 0, &its, NULL) == -1) {
//         perror("timer_settime");
//         return 0;
//     }

//     return 1;
// }

// TickType_t xTaskGetTickCount() {
//     struct timespec ts;
//     if (clock_gettime(CLOCK_MONOTONIC, &ts) == -1) {
//         perror("clock_gettime");
//         return 0;
//     }
//     return (TickType_t)(ts.tv_sec * TICKS_PER_SECOND + ts.tv_nsec / (1000000000 / TICKS_PER_SECOND));
// }