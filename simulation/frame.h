#ifndef FRAME_H
#define FRAME_H


#include <arpa/inet.h>
#include <pthread.h>
#include <netinet/in.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>


#define SWARM_RANGING_MODE
// #define DYNAMIC_SWARM_RANGING_MODE

#if defined(SWARM_RANGING_MODE)
#include "../Inc/adhocuwb_swarm_ranging.h"
#elif defined(DYNAMIC_SWARM_RANGING_MODE)
#include "../Inc/adhocuwb_dynamic_swarm_ranging.h"
#endif

#define     NODES_NUM       2
#define     ADDR_SIZE       20
#define     CENTER_ADDRESS  "CENTER"
#define     CENTER_PORT     8888
#define     MAX_LINE_LEN    256
#define     MESSAGE_SIZE    512
#define     PAYLOAD_SIZE    MESSAGE_SIZE - ADDR_SIZE - sizeof(size_t)
#define     READ_PERIOD     200
#define     REJECT_INFO     "REJECT"


const char *FILE_NAME = "./data/2025-08-06-17-48-05_.csv";


typedef enum {
    TX,                     // sender
    RX                      // receiver
} Simu_Direction_t;

typedef struct {
    uint16_t address;
    Simu_Direction_t status;
    dwTime_t timestamp;
} Line_Message_t;

typedef struct {
    char srcAddress[ADDR_SIZE];
    char payload[PAYLOAD_SIZE];
    size_t size;
} Simu_Message_t;           // message sent between center and drones

typedef struct {
    int socket;
    char address[ADDR_SIZE];
} Drone_Node_t;             // drone

typedef struct {
    Drone_Node_t node[NODES_NUM];
    int count;
    pthread_mutex_t mutex;
} Drone_Node_Set_t;         // set of drones

#endif