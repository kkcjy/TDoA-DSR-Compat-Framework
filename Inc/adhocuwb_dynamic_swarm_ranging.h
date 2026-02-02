#ifndef DYNAMIC_SWARM_RANGING
#define DYNAMIC_SWARM_RANGING


#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>


#if defined(SNIFFER_STORAGE_COMPILE) || defined(SIMULATION_COMPILE)
#include "../../support.h"
#else
#include "adhocuwb_init.h"
#include "dwTypes.h"
#endif

#if !defined(SNIFFER_STORAGE_COMPILE) && !defined(SIMULATION_COMPILE)
#include "adhocuwb_platform.h"
#include "dwm3000_init.h"
#endif


/* -------------------- Define -------------------- */
/* Mode Enable */
// simulation mode
#ifdef SIMULATION_COMPILE
    #ifdef COMPENSATE_DYNAMIC_RANGING
        #define         COMPENSATE_ENABLE
    #endif
// crazyflie-firmware mode
#else
// Perform compensation after each received packet calculation
    #define         COMPENSATE_ENABLE
#endif
#define         OPTIMAL_RANGING_SCHEDULE_ENABLE                         // Enable Midpoint_Adjustment to ensure equal message-sending intervals among drones within the swarm
// #define         TDOA_COMPAT_ENABLE                                      // Fusion of DSR ranging and TDoA-based range-difference measurements

/* Ranging Constants */
#define         RANGING_PERIOD              200

/* Ranging Message */
#define         MESSAGE_TX_POOL_SIZE        3
#define         MESSAGE_BODYUNIT_SIZE       3

/* Ranging Table Set */
#define         RANGING_TABLE_SIZE          10
#define         Tr_Rr_BUFFER_SIZE           3
#define         SEND_LIST_SIZE              5
#define         RECEIVE_LIST_SIZE           RANGING_TABLE_SIZE

/* Effective Distance Range */
#define         UPPER_BOUND_DISTANCE        1000
#define         LOWER_BOUND_DISTANCE        0

/* Compensation Coefficient */
#define         SEQGAP_THRESHOLD            3
#define         DECELERATION_BOUND          15
#define         STATIC_BOUND                3

/* Queue Constants */
#define         RANGING_RX_QUEUE_SIZE       10
#define         RANGING_RX_QUEUE_ITEM_SIZE  sizeof(Ranging_Message_With_Additional_Info_t)

/* Else */
#define         TYPE_DSR                    0
#ifdef COMPENSATE_ENABLE
    #define         RANGING_MODE                "CDSR"
#else
    #define         RANGING_MODE                "DSR"
#endif
#define         CHECK_PERIOD                15
#define         CONVERGENCE_THRESHOLD       0.989f
#define         NEIGHBOR_ADDRESS_MAX        32
#define         UWB_MAX_TIMESTAMP           1099511627776
#define         VELOCITY                    0.4691763978616f

/* Index */
#define         index_t                     uint16_t
#define         table_index_t               uint8_t

/* Invalid Value */
#define         NULL_ADDRESS                0xFFFF
#define         NULL_ADDR                   0xFF
#define         NULL_DIS                    -1.0f
#define         NULL_INDEX                  0xFF
#define         NULL_SEQ                    0x0
#define         NULL_RATE                   -1.0f
#define         NULL_TIMESTAMP              0xFFFFFFFFFFU
#define         NULL_TOF                    -1.0f


/* -------------------- Base Struct -------------------- */
typedef struct {
    dwTime_t timestamp;
    uint16_t seqNumber;
} __attribute__((packed)) Timestamp_Tuple_t;

typedef enum {
    UNUSED,
    USING
} TableState;

typedef enum {
    RANGING_STATE_RESERVED,
    RANGING_STATE_S1,
    RANGING_STATE_S2,
    RANGING_STATE_S3,                                                   // Temporary internal state for initialization, never been invoked
    RANGING_STATE_S4,
    RANGING_STATE_S5,
    RANGING_STATE_S6,                                                   // Temporary internal state for distance calculation, never been invoked
    RANGING_TABLE_STATE_COUNT
} RANGING_TABLE_STATE;

typedef enum {
    RANGING_SUBSTATE_RESERVED,
    RANGING_SUBSTATE_S1,                                                // Initialize
    RANGING_SUBSTATE_S2,                                                // Swarm Ranging
    RANGING_SUBSTATE_S3,                                                // Dynamic Swarm Ranging
    RANGING_TABLE_SUBSTATE_COUNT
} RANGING_TABLE_SUBSTATE;                                               // Valid only when RANGING_TABLE_STATE is S4 ~ S6

typedef enum {
    RANGING_EVENT_TX,
    RANGING_EVENT_RX,
    RANGING_EVENT_RX_NO,
    RANGING_TABLE_EVENT_COUNT
} RANGING_TABLE_EVENT;

typedef enum {
    FIRST_CALCULATE,
    SECOND_CALCULATE
} CalculateState;


/* -------------------- Ranging Message -------------------- */
typedef struct {
    uint16_t srcAddress;                                                // Source address
    uint16_t msgSequence;                                               // Message sequence number
    Timestamp_Tuple_t Txtimestamps[MESSAGE_TX_POOL_SIZE];               // Local Tx timestamps prior to current transmission
    uint8_t msgLength;                                                  // Message length
    uint8_t type;                                                       // Message type
    uint16_t filter;                                                    // Bloom filter
} __attribute__((packed)) Ranging_Message_Header_t;

typedef union {
    struct {
        uint8_t rawtime[5];                                             // Local Rx timestamp of last received message
        uint8_t address;                                                // Neighbor address
        uint16_t seqNumber;                                             // Message sequence number of last received message
    } __attribute__((packed));
    dwTime_t timestamp;
} Ranging_Message_Body_Unit_t;

typedef struct {
    Ranging_Message_Header_t header;
    Ranging_Message_Body_Unit_t bodyUnits[MESSAGE_BODYUNIT_SIZE];
} __attribute__((packed)) Ranging_Message_t;

typedef struct {
    Ranging_Message_t rangingMessage;
    dwTime_t timestamp;                                                 // Local Rx timestamp of received message
} __attribute__((packed)) Ranging_Message_With_Additional_Info_t;


/* -------------------- Ranging Table -------------------- */
typedef struct {
    index_t topIndex;
    Timestamp_Tuple_t Txtimestamps[SEND_LIST_SIZE];
} __attribute__((packed)) SendList_t;

typedef struct {
    index_t topIndex;
    dwTime_t Rxtimestamps[RECEIVE_LIST_SIZE];
} __attribute__((packed)) ReceiveList_t;

typedef struct {
    Timestamp_Tuple_t Tr;
    Timestamp_Tuple_t Rr;
} __attribute__((packed)) Ranging_Table_Tr_Rr_Candidate_t;

typedef struct {
    index_t topIndex;                                                   // Index of latest valid (Tr,Rr) pair
    Ranging_Table_Tr_Rr_Candidate_t candidates[Tr_Rr_BUFFER_SIZE];
} __attribute__((packed)) Ranging_Table_Tr_Rr_Buffer_t;

/* Ranging Table
    +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |
    +------+------+------+------+------+------+------+
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |
    +------+------+------+------+------+------+------+
    |    EPToF    |    PToF     |
    +------+------+------+------+
    Note:   1. EPToF = ToF_eb + ToF_ep
            2. PToF = ToF_b + ToFp
*/
typedef struct {
    uint16_t neighborAddress;

    Timestamp_Tuple_t ETb;
    Timestamp_Tuple_t ERb;
    Timestamp_Tuple_t ETp;
    Timestamp_Tuple_t ERp;
    Timestamp_Tuple_t Tb;
    Timestamp_Tuple_t Rb;
    Timestamp_Tuple_t Tp;
    Timestamp_Tuple_t Rp;
    Ranging_Table_Tr_Rr_Buffer_t TrRrBuffer;
    Timestamp_Tuple_t Tf;
    Timestamp_Tuple_t Rf;
    Timestamp_Tuple_t Re;

    float PToF;                                                         // Pair of ToF
    float EPToF;

    bool continuitySign;                                                // True: sequence numbers are contiguous                | False: discontinuity detected
    bool expirationSign;                                                // True: no recent frames received from this neighbor   | False: recently active

    TableState tableState;
    RANGING_TABLE_STATE rangingState;
} __attribute__((packed)) Ranging_Table_t;

typedef struct {
    uint16_t size;                                                      // Number of neighbors
    uint32_t localSeqNumber;                                            // Local message sequence number
    SendList_t sendList;                                                // TX timestamps of messages sent to neighbors
    #ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
    ReceiveList_t receiveList;                                          // RX timestamps of messages received from neighbors (sender-independent)
    #endif
    Ranging_Table_t rangingTable[RANGING_TABLE_SIZE];                   // Per-neighbor ranging state table
    Timestamp_Tuple_t lastRxtimestamp[RANGING_TABLE_SIZE];              // Last RX timestamp per neighbor
    index_t priorityQueue[RANGING_TABLE_SIZE];                          // Circular priority queue for neighbor selection
    #ifndef SNIFFER_STORAGE_COMPILE
    SemaphoreHandle_t mutex;                                            // Concurrency protection
#endif
} __attribute__((packed)) Ranging_Table_Set_t;


/* -------------------- TDoA -------------------- */
// Anchor must remain static during experiment, tag can move arbitrarily, but only one of two modes can be selected at a time
#ifdef TDOA_COMPAT_ENABLE
// #define         ANCHOR_MODE_ENABLE
// #define         TAG_MODE_ENABLE
#define         TYPE_TDOA                   1
#define         TDOA_BROADCAST_PERIOD       50
#define         ANCHOR_SIZE                 2                           // Number of Anchors
#define         TIMESTAMP_LIST_SIZE         7
#define         TDOA_LOSS_TIME_THRESHOLD    15000000000                 // Timeout threshold to fall back to DSR mode if no TDoA frames are received


typedef struct {
    uint16_t neighborAnchorAddress;
    uint8_t topIndex;
    Timestamp_Tuple_t receivedLog[TIMESTAMP_LIST_SIZE];
    TableState tableState;
} __attribute__((packed)) Anchor_Table_t;

typedef struct {
    uint16_t localAnchorAddress;
    uint8_t topIndex;
    Timestamp_Tuple_t broadcastLog[TIMESTAMP_LIST_SIZE];
    Anchor_Table_t anchorTables[ANCHOR_SIZE - 1];
    uint32_t localSeqNumber;
    uint8_t size;
    SemaphoreHandle_t mutex;
} __attribute__((packed)) Anchor_Table_Set_t;

typedef struct {
    uint16_t anchorAddress;
    uint8_t topIndex;
    Timestamp_Tuple_t broadcastLog[TIMESTAMP_LIST_SIZE];                // Anchor's broadcast timestamps
    Timestamp_Tuple_t receiveLog[TIMESTAMP_LIST_SIZE];                  // Tag's reception timestamps
    Timestamp_Tuple_t anchorLastReceiveLog[2 * NEIGHBOR_ADDRESS_MAX]; 
    /*  [0 ... NEIGHBOR_ADDRESS_MAX - 1]                                Store the second most recent timestamp
        [NEIGHBOR_ADDRESS_MAX ... 2 * NEIGHBOR_ADDRESS_MAX - 1]         Store the most recent reception timestamp */
    TableState tableState;
} __attribute__((packed)) Tag_Table_t;

typedef struct {
    Tag_Table_t tagTables[ANCHOR_SIZE];
    uint8_t size;
    SemaphoreHandle_t mutex;
} __attribute__((packed)) Tag_Table_Set_t;


#if defined(ANCHOR_MODE_ENABLE)
    void anchorTableSetInit();
    index_t registerAnchorTable(Anchor_Table_Set_t *anchorTableSet, uint16_t neighborAnchorAddress);
    void updateBroadcastLogForAnchor(Anchor_Table_Set_t *anchorTableSet, Timestamp_Tuple_t timestampTuple);
    void updateReceivedLogForAnchor(Anchor_Table_t *anchorTable, Timestamp_Tuple_t timestampTuple);
    table_index_t findAnchorTable(Anchor_Table_Set_t *anchorTableSet, uint16_t address);
    void generateTDoAMessage(Ranging_Message_t *rangingMessage);
    void processTDoAMessageForAnchor(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo);
#elif defined(TAG_MODE_ENABLE)
    void tagTableSetInit();
    index_t registerTagTable(Tag_Table_Set_t *tagTableSet, uint16_t anchorAddress);
    void updateBroadcastLogForTag(Tag_Table_t *tagTable, Timestamp_Tuple_t timestampTuple);
    void updateReceivedLogForTag(Tag_Table_t *tagTable, Timestamp_Tuple_t timestampTuple);
    void updateAnchorLastReceiveLogForTag(Tag_Table_t *tagTable,uint16_t address, Timestamp_Tuple_t timestampTuple);
    table_index_t findTagTable(Tag_Table_Set_t *tagTableSet, uint16_t address);
    table_index_t getCollaboratorAnchor(Tag_Table_Set_t *tagTableSet, uint16_t anchorIndex, Timestamp_Tuple_t tag_P1, Timestamp_Tuple_t tag_P3, index_t *logIndex);
    void printTagTable(Tag_Table_t *tagTable);
    void processTDoAMessageForTag(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo);
    void dispatchTagClusterMode(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo);
#endif
#endif


/* -------------------- Null Struct -------------------- */
static const Timestamp_Tuple_t nullTimestampTuple = {.timestamp.full = NULL_TIMESTAMP, .seqNumber = NULL_SEQ};
static const Ranging_Table_Tr_Rr_Candidate_t nullCandidate = {.Tr.timestamp.full = NULL_TIMESTAMP, .Tr.seqNumber = NULL_SEQ, .Rr.timestamp.full = NULL_TIMESTAMP, .Rr.seqNumber = NULL_SEQ,};


/* -------------------- Base Operation -------------------- */
bool COMPARE_TIME(uint64_t time_a, uint64_t time_b);


/* -------------------- Ranging Table Set Operation -------------------- */
index_t findSendList(SendList_t *sendList, uint16_t seqNumber);
void updateSendList(SendList_t *sendList, Timestamp_Tuple_t timestampTuple);
#ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
void updateReceiveList(ReceiveList_t *receiveList, dwTime_t timestamp);
#endif

void rangingTableTr_Rr_BufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer);
void updateRangingTableTr_Buffer(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tr);
void updateRangingTableRr_Buffer(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Rr);
Ranging_Table_Tr_Rr_Candidate_t rangingTableTr_Rr_BufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tf);

void rangingTableInit(Ranging_Table_t *rangingTable);
table_index_t registerRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address);
void deregisterRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address);
table_index_t findRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address);
void fillRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf, Timestamp_Tuple_t Re);
void shiftRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, float PToF);
void replaceRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, float PToF);

void updatePriorityQueue(Ranging_Table_Set_t *rangingTableSet, int8_t shiftCount);
void rangingTableSetInit();
void checkExpirationCallback(Ranging_Table_Set_t *rangingTableSet);
int getCurrentSubstate(Ranging_Table_t *rangingTable);


/* -------------------- Calculation Function -------------------- */
float dsrRangingAlgorithm(Timestamp_Tuple_t T1, Timestamp_Tuple_t R1, Timestamp_Tuple_t T2, Timestamp_Tuple_t R2, Timestamp_Tuple_t T3, Timestamp_Tuple_t R3, float ToF12);
float srRangingAlgorithm(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);
float calculatePToF(Ranging_Table_t *rangingTable, Ranging_Table_Tr_Rr_Candidate_t candidate);
void continuousPacketLossHandler(Ranging_Table_t *rangingTable, Ranging_Table_Tr_Rr_Candidate_t candidate);


/* -------------------- Print Function -------------------- */
void printRangingMessage(Ranging_Message_t *rangingMessage);
void printSendList(SendList_t *sendList);
void printRangingTable(Ranging_Table_t *rangingTable);
void printPriorityQueue(Ranging_Table_Set_t *rangingTableSet);
void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet);
void printclassicCalculateTuple(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);
void printCalculateTuple(Timestamp_Tuple_t ETb, Timestamp_Tuple_t ERb, Timestamp_Tuple_t ETp, Timestamp_Tuple_t ERp, Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp);


/* -------------------- State Machine Operation -------------------- */
typedef void (*EventHandlerTable)(Ranging_Table_t*);


/* -------------------- Generate and Process -------------------- */
void generateDSRMessage(Ranging_Message_t *rangingMessage);
void processDSRMessage(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo);
float getCurDistance(uint16_t neighborAddress, uint64_t Rx);

#endif