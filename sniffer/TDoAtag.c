#include <libusb-1.0/libusb.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../Inc/dwm3000_init.h"
#include "../Inc/adhocuwb_dynamic_swarm_ranging.h"


#define         TIMESTAMP_LIST_SIZE         3
#define         ANCHOR_SIZE                 2
#define         NULL_INDEX                  0xFF
#define         NULL_ADDRESS                0xFFFF
#define         NULL_SEQ                    0x0
#define         NULL_TIMESTAMP              0xFFFFFFFFFFU
#define         NULL_DIS                    -1
#define         MAX_PACKET_SIZE             256
#define         MAGIC_MATCH                 0xBB88
#define         VENDOR_ID                   0x0483
#define         PRODUCT_ID                  0x5740


typedef union {
    uint8_t raw[18];
    struct {
        uint32_t magic;
        uint16_t senderAddress;
        uint16_t seqNumber;
        uint16_t msgLength;
        uint64_t rxTime;
    } __attribute__((packed));
} __attribute__((packed)) Sniffer_Meta_t;

// TDoA Table Structure
typedef struct {
    dwTime_t timestamp;
    uint16_t seqNumber;
} __attribute__((packed)) Timestamp_Tuple_t;

typedef struct {
    uint16_t remoteAnchorAddress[ANCHOR_SIZE - 1];
    int16_t distance[ANCHOR_SIZE - 1];
} __attribute__((packed)) Remote_Anchor_Distance_Table_t;

typedef struct {
    uint16_t anchorAddress;
    uint8_t topIndex;
    Timestamp_Tuple_t Tx[TIMESTAMP_LIST_SIZE];
    Timestamp_Tuple_t Rx[TIMESTAMP_LIST_SIZE];
    Remote_Anchor_Distance_Table_t remoteAnchorDistanceTable;
} __attribute__((packed)) Anchor_Table_t;

typedef struct {
    Anchor_Table_t AnchorTable[ANCHOR_SIZE];
} __attribute__((packed)) Anchor_Table_Set_t;


static Anchor_Table_Set_t *AnchorTableSet;
volatile sig_atomic_t keep_running = 1;     // used for the interruption caused by Ctrl+C
int ignore_lines = 30;
int listen_lines = 0;


bool COMPARE_TIME(uint64_t time_a, uint64_t time_b) {
    uint64_t diff = (time_b - time_a) & (UWB_MAX_TIMESTAMP - 1);
    return diff < (UWB_MAX_TIMESTAMP - diff);
}

void handle_sigint(int sigint) {
    keep_running = 0;
}

void AnchorTableSetInit() {
    AnchorTableSet = (Anchor_Table_Set_t*)malloc(sizeof(Anchor_Table_Set_t));
    for(int i = 0; i < ANCHOR_SIZE; i++) {
        AnchorTableSet->AnchorTable[i].anchorAddress = NULL_ADDRESS;
        AnchorTableSet->AnchorTable[i].topIndex = NULL_INDEX;
        for(int j = 0; j < TIMESTAMP_LIST_SIZE; j++) {
            AnchorTableSet->AnchorTable[i].Tx[j].seqNumber = NULL_SEQ;
            AnchorTableSet->AnchorTable[i].Tx[j].timestamp.full = NULL_TIMESTAMP;
            AnchorTableSet->AnchorTable[i].Rx[j].seqNumber = NULL_SEQ;
            AnchorTableSet->AnchorTable[i].Rx[j].timestamp.full = NULL_TIMESTAMP;
        }
        for(int j = 0; j < ANCHOR_SIZE - 1; j++) {
            AnchorTableSet->AnchorTable[i].remoteAnchorDistanceTable.remoteAnchorAddress[j] = NULL_ADDRESS;
            AnchorTableSet->AnchorTable[i].remoteAnchorDistanceTable.distance[j] = NULL_DIS;
        }
    }
}

uint64_t processTDoAMessage(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo) {
    Ranging_Message_t *rangingMessage = &rangingMessageWithAdditionalInfo->rangingMessage;
    uint64_t RxTimestamp = rangingMessageWithAdditionalInfo->timestamp.full;

    uint8_t tableIndex = NULL_INDEX;
    for(int i = 0; i < ANCHOR_SIZE; i++) {
        if(AnchorTableSet->AnchorTable[i].anchorAddress == rangingMessage->header.srcAddress) {
            tableIndex = i;
        }
    }
    if(tableIndex == NULL_INDEX) {
        DEBUG_PRINT("Warnging: failed to find TDoA table\n");
        return NULL_TIMESTAMP;
    }

    Anchor_Table_t *AnchorTable = &AnchorTableSet->AnchorTable[tableIndex];

    /* Process Header */
    // update Rx
    uint16_t RxSeqNumber = rangingMessage->header.msgSequence;
    AnchorTable->topIndex = (AnchorTable->topIndex + 1) % TIMESTAMP_LIST_SIZE;
    AnchorTable->Rx[AnchorTable->topIndex].timestamp.full = RxTimestamp;
    AnchorTable->Rx[AnchorTable->topIndex].seqNumber = RxSeqNumber;
    AnchorTable->Tx[AnchorTable->topIndex].timestamp.full = NULL_TIMESTAMP;
    AnchorTable->Tx[AnchorTable->topIndex].seqNumber = NULL_SEQ;

    // update Tx
    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        Timestamp_Tuple_t Tx = rangingMessage->header.Txtimestamps[i];
        for(int j = 0; j < TIMESTAMP_LIST_SIZE; j++) {
            if(AnchorTable->Rx[j].seqNumber == Tx.seqNumber && AnchorTable->Tx[j].seqNumber == NULL_SEQ) {
                AnchorTable->Tx[j] = Tx;
            }
        }
    }

    /* Process Bodyunit */
    // get distance
    uint8_t bodyunitIndex = NULL_INDEX;
    uint16_t remoteAnchorAddress = NULL_ADDR;
    for(int i = 0; i < MESSAGE_BODYUNIT_SIZE; i++) {
        if(remoteAnchorAddress == NULL_ADDR) {
            bodyunitIndex = i;
            remoteAnchorAddress = rangingMessage->bodyUnits[i].address;
        }
        else if(COMPARE_TIME(rangingMessage->bodyUnits[bodyunitIndex].timestamp.full, rangingMessage->bodyUnits[i].timestamp.full)) {
            bodyunitIndex = i;
            remoteAnchorAddress = rangingMessage->bodyUnits[i].address;
        }
    }
    int16_t distance = NULL_DIS;
    for(int i = 0; i < ANCHOR_SIZE - 1; i++) {
        if(AnchorTable->remoteAnchorDistanceTable.remoteAnchorAddress[i] == remoteAnchorAddress) {
            distance = AnchorTable->remoteAnchorDistanceTable.distance[i];
        }
    }

    // get delta_Tx, delta_Rx
    uint64_t Tx_1 = NULL_TIMESTAMP;
    uint64_t Rx_1 = NULL_TIMESTAMP;
    uint64_t Tx_2 = NULL_TIMESTAMP;
    uint64_t Rx_2 = NULL_TIMESTAMP;
    uint64_t last_Tx_1 = NULL_TIMESTAMP;
    uint64_t last_Rx_1 = NULL_TIMESTAMP;
    uint64_t delta_Tx = NULL_TIMESTAMP;
    uint64_t delta_Rx = NULL_TIMESTAMP;

    // Infer the transmission time of the remote anchor from the reception time at the current anchor
    Tx_2 = (rangingMessage->bodyUnits[0].timestamp.full - (distance / VELOCITY) + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    for(int i = 0; i < ANCHOR_SIZE; i++) {
        if(AnchorTableSet->AnchorTable[i].anchorAddress == remoteAnchorAddress) {
            for(int j = 0; j < TIMESTAMP_LIST_SIZE; j++) {
                if(rangingMessage->bodyUnits[0].seqNumber == AnchorTableSet->AnchorTable[i].Rx[j].seqNumber) {
                    Rx_2 =  AnchorTableSet->AnchorTable[i].Rx[j].timestamp.full % UWB_MAX_TIMESTAMP;
                }
            }
        }
    }

    for(int i = 0; i < TIMESTAMP_LIST_SIZE; i++) {
        if(AnchorTable->Tx[i].seqNumber != NULL_SEQ && AnchorTable->Rx[i].seqNumber != NULL_SEQ && COMPARE_TIME(AnchorTable->Tx[i].timestamp.full, Tx_2) && COMPARE_TIME(AnchorTable->Rx[i].timestamp.full, Rx_2)) {
            Tx_1 = AnchorTable->Tx[i].timestamp.full;
            Rx_1 = AnchorTable->Rx[i].timestamp.full;
            uint8_t index = (i - 1 + TIMESTAMP_LIST_SIZE) % TIMESTAMP_LIST_SIZE;
            last_Tx_1 = AnchorTable->Tx[index].timestamp.full;
            last_Rx_1 = AnchorTable->Rx[index].timestamp.full;
        }
    }

    if(Tx_1 == NULL_TIMESTAMP || Rx_1 == NULL_TIMESTAMP || Tx_2 == NULL_TIMESTAMP || Rx_2 == NULL_TIMESTAMP || last_Tx_1 == NULL_TIMESTAMP || last_Rx_1 == NULL_TIMESTAMP) {
        DEBUG_PRINT("Warnging: failed to find all timestamps\n");
        return NULL_TIMESTAMP;
    }

    delta_Tx = (Tx_2 - Tx_1 + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    delta_Rx = (Rx_2 - Rx_1 + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    uint64_t adjacent_delta_Tx = (Tx_1 - last_Tx_1 + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    uint64_t adjacent_delta_Rx = (Rx_1 - last_Rx_1 + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    float alpha = (float)adjacent_delta_Rx / (float)adjacent_delta_Tx;

    uint64_t TDoA = delta_Rx - alpha * delta_Tx;

    return TDoA;
}

// Implementation of TDoA task handling
void handleTDoATask(libusb_device_handle *device_handle) {
    uint8_t buffer[MAX_PACKET_SIZE];
    uint8_t payload[MAX_PACKET_SIZE];
    uint8_t endpoint = 0x81;
    int transferred;

    while (keep_running) {
        // first reception
        int response = libusb_bulk_transfer(device_handle, endpoint, buffer, MAX_PACKET_SIZE, &transferred, 1000);
        
        // success
        if(response == 0 && transferred <= MAX_PACKET_SIZE) {
            if(listen_lines < ignore_lines) {
                if(listen_lines == 0) {
                    printf("Ignoring line %d", listen_lines);
                }
                listen_lines++;
                printf(".");
                if(listen_lines == ignore_lines) {
                    printf("\n");
                }
            }

            Sniffer_Meta_t *meta = (Sniffer_Meta_t *)buffer;

            if(meta->magic == MAGIC_MATCH && meta->msgLength <= 256) {

                response = libusb_bulk_transfer(device_handle, endpoint, payload, meta->msgLength, &transferred, 1000);
                if (response == 0 && transferred == meta->msgLength) {
                    Ranging_Message_t rangingMessage;
                    memcpy(&rangingMessage, payload, sizeof(Ranging_Message_t));

                    Ranging_Message_With_Additional_Info_t rangingMessageWithAdditionalInfo;
                    rangingMessageWithAdditionalInfo.rangingMessage = rangingMessage;
                    rangingMessageWithAdditionalInfo.timestamp.full = meta->rxTime;

                    uint64_t TDoA = processTDoAMessage(rangingMessageWithAdditionalInfo);

                    printf("TDoA: %lu, delta_dis = %f, Anchor: %u, remote Anchor: %u\n", TDoA, TDoA * VELOCITY, rangingMessage.header.srcAddress, rangingMessage.bodyUnits[0].address);
                }
                // fail
                else if(transferred > MAX_PACKET_SIZE) {
                    printf("MAX_PACKET_SIZE is small\n");
                    exit(EXIT_FAILURE);
                }
                else {
                    printf("Bulk transfer failed: %s\n", libusb_strerror(response));
                    exit(EXIT_FAILURE);
                }
            }
        }
        else if(transferred > MAX_PACKET_SIZE) {
            printf("MAX_PACKET_SIZE is small\n");
            exit(EXIT_FAILURE);
        }
        else {
            printf("Bulk transfer failed: %s\n", libusb_strerror(response));
            exit(EXIT_FAILURE);
        }
    }
}

int main() {
    libusb_device_handle *device_handle = NULL;
    libusb_context *context = NULL;

    /* Registering signal handling, initializing USB devices, and preparing log files */
    signal(SIGINT, handle_sigint);
    
    int response = libusb_init(&context);
    if (response < 0) {
        fprintf(stderr, "libusb init error\n");
        return 1;
    }

    device_handle = libusb_open_device_with_vid_pid(context, VENDOR_ID, PRODUCT_ID);
    if (!device_handle) {
        fprintf(stderr, "cannot find USB device\n");
        return 1;
    }

    libusb_set_auto_detach_kernel_driver(device_handle, 1);
    libusb_claim_interface(device_handle, 0);

    // TDoA task
    handleTDoATask(device_handle);

    libusb_release_interface(device_handle, 0);
    libusb_close(device_handle);
    libusb_exit(context);

    return 0;
}