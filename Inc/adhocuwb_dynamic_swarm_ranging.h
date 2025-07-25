#ifndef DYNAMIC_SWARM_RANGING
#define DYNAMIC_SWARM_RANGING


#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include "dwTypes.h"
#include "adhocuwb_platform.h"
#include "adhocuwb_init.h"
#include "dwm3000_init.h"


/* -------------------- Define -------------------- */
#define         CLASSIC_SUPPORT_ENABLE
// #define         COORDINATE_SEND_ENABLE
// #define         COMPENSATE_ENABLE
#define         STATE_MACHINE_ENABLE
// #define         PACKET_LOSS_ENABLE

#define         COMPENSATE_RATE             0.5
#define         PACKET_LOSS_RATE            0.1

#define         NULL_ADDR                   0xFFFF
#define         NULL_SEQ                    0x0
#define         NULL_TIMESTAMP              0xFFFFFFFFU
#define         NULL_INDEX                  0xFF
#define         NULL_TOF                    -1.0f
#define         NULL_DIS                    -1.0f

#define         table_index_t               uint8_t
#define         index_t                     uint16_t

#define         MESSAGE_TX_POOL_SIZE        3
#define         MESSAGE_BODY_UNIT_SIZE      2

#define         SEND_LIST_SIZE              5
#define         Tr_Rr_BUFFER_POOL_SIZE      3
#define         RANGING_TABLE_SIZE          10        

#define         UPPER_BOUND_DISTANCE        1000
#define         LOWER_BOUND_DISTANCE        0

/* Ranging Constants */
#define         RANGING_PERIOD              200
#define         RANGING_PERIOD_MIN          50   
#define         RANGING_PERIOD_MAX          500  

/* Queue Constants */
#define         RANGING_RX_QUEUE_SIZE       10
#define         RANGING_RX_QUEUE_ITEM_SIZE  sizeof(Ranging_Message_With_Additional_Info_t)

#define         CHECK_PERIOD                15
#define         UWB_MAX_TIMESTAMP           1099511627776
#define         VELOCITY                    0.4691763978616f
#define         CONVERGENCE_THRESHOLD       0.989f
#define         NEIGHBOR_ADDRESS_MAX        32

/* -------------------- Base Struct -------------------- */
typedef struct {
    dwTime_t timestamp;                      
    uint16_t seqNumber;   
} __attribute__((packed)) Timestamp_Tuple_t;            // 10 byte

typedef enum {
    UNUSED,
    USING
} TableState;

typedef enum {
    RANGING_STATE_RESERVED,
    RANGING_STATE_S1,
    RANGING_STATE_S2,
    RANGING_STATE_S3,   // RANGING_STATE_S3 is effectively a temporary state for initialization, never been invoked
    RANGING_STATE_S4,
    RANGING_STATE_S5,
    RANGING_STATE_S6,   // RANGING_STATE_S6 is effectively a temporary state for distance calculation, never been invoked
    RANGING_TABLE_STATE_COUNT
} RANGING_TABLE_STATE;

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
    uint16_t srcAddress;                // address of the source of message
    uint16_t msgSequence;               // sequence of message
    Timestamp_Tuple_t Txtimestamps[MESSAGE_TX_POOL_SIZE];
                                        // last local Txtimestamps when message is sent
    #ifdef COORDINATE_SEND_ENABLE
    Coordinate_Tuple_t TxCoordinate;    // local coordinate when message is sent
    #endif
    uint16_t filter;                    // bloom filter
    uint16_t msgLength;                 // size of message
} __attribute__((packed)) Message_Header_t;             // 8 + 10 * MESSAGE_TX_POOL_SIZE byte = 38 byte

typedef union{
    struct {
        uint8_t rawtime[5]; 
        uint8_t address;                // address of neighbor
        uint16_t seqNumber;             // last local Rxtimestamp.seqNumber when message is received
    } __attribute__((packed));
    dwTime_t timestamp;                 // last local Rxtimestamp.timestamp when message is received
} Message_Body_Unit_t;                                  // 8 byte

typedef struct {
    Message_Header_t header;
    Message_Body_Unit_t bodyUnits[MESSAGE_BODY_UNIT_SIZE];
} __attribute__((packed)) Ranging_Message_t;            // 38 + 12 * MESSAGE_BODY_UNIT_SIZE byte = 158 byte

typedef struct {
    Ranging_Message_t rangingMessage;
    dwTime_t timestamp;                 // local timestamp when message is received
    #ifdef COORDINATE_SEND_ENABLE
    Coordinate_Tuple_t RxCoordinate;    // local coordinate when message is received
    #endif
} __attribute__((packed)) Ranging_Message_With_Additional_Info_t;


/* -------------------- Ranging Table -------------------- */
typedef struct {
    index_t topIndex;
    Timestamp_Tuple_t Txtimestamps[SEND_LIST_SIZE];
    #ifdef COORDINATE_SEND_ENABLE
    Coordinate_Tuple_t TxCoordinate;    // local coordinate when message is sent
    #endif
} __attribute__((packed)) SendList_t;

typedef struct {
    Timestamp_Tuple_t Tr;
    Timestamp_Tuple_t Rr;
} __attribute__((packed)) Ranging_Table_Tr_Rr_Candidate_t;

typedef struct {
    index_t topIndex;                   // index of latest valid (Tr,Rr) pair
    Ranging_Table_Tr_Rr_Candidate_t candidates[Tr_Rr_BUFFER_POOL_SIZE];
} __attribute__((packed)) Ranging_Table_Tr_Rr_Buffer_t;

/* Ranging Table
    +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |
    +------+------+------+------+------+------+------+
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |
    +------+------+------+------+------+------+------+
    |    EPTof    |    PTof     |
    +------+------+------+------+
    Note:   1. EPTof = Tof_eb + Tof_ep = Tof_ebep
            2. PTof = Tof_b + Tofp = Tof_bp
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

    float PTof;                         // pair of Tof
    float EPTof;                        // early pair of Tof

    bool continuitySign;                // true: contiguous | false: non-contiguous
    bool expirationSign;                // true: no recent access --> expired | recent access --> not expired

    TableState tableState;              // UNUSED / USING
    #ifdef STATE_MACHINE_ENABLE
    RANGING_TABLE_STATE rangingState;   // used for state machine
    #endif
} __attribute__((packed)) Ranging_Table_t;

typedef struct {
    uint16_t size;                                          // number of neighbors
    uint32_t localSeqNumber;                                // seqNumber of message sent
    SendList_t sendList;                                    // timestamps of messages sent to neighbors
    Ranging_Table_t rangingTable[RANGING_TABLE_SIZE];
    Timestamp_Tuple_t lastRxtimestamp[RANGING_TABLE_SIZE];  
    index_t priorityQueue[RANGING_TABLE_SIZE];              // circular priority queue used for choosing neighbors to send messages
    SemaphoreHandle_t mutex;
} __attribute__((packed)) Ranging_Table_Set_t;


/* -------------------- Null Struct -------------------- */
static const Timestamp_Tuple_t nullTimestampTuple = {.timestamp.full = NULL_TIMESTAMP, .seqNumber = NULL_SEQ};
#ifdef COORDINATE_SEND_ENABLE
static const Coordinate_Tuple_t nullCoordinateTuple = {.x = -1, .y = -1, .z = -1};
#endif
static const Ranging_Table_Tr_Rr_Candidate_t nullCandidate = {.Tr.timestamp.full = NULL_TIMESTAMP, .Tr.seqNumber = NULL_SEQ, .Rr.timestamp.full = NULL_TIMESTAMP, .Rr.seqNumber = NULL_SEQ,};


/* -------------------- Base Operation -------------------- */
bool COMPARE_TIME(uint64_t time_a, uint64_t time_b);
float getDistanceReal(uint16_t neighborAddress);
void setDistanceReal(uint16_t neighborAddress, float distance);
float getDistanceCalculate(uint16_t neighborAddress);
void setDistanceCalculate(uint16_t neighborAddress, float distance);


/* -------------------- Ranging Table Set Operation -------------------- */
index_t findSendList(SendList_t *sendList, uint16_t seqNumber);
#ifdef COORDINATE_SEND_ENABLE
void updateSendList(SendList_t *sendList, Timestamp_Tuple_t timestampTuple, Coordinate_Tuple_t coordinateTuple);
#else
void updateSendList(SendList_t *sendList, Timestamp_Tuple_t timestampTuple);
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
void shiftRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, float PTof);
void replaceRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, float PTof);

void updatePriorityQueue(Ranging_Table_Set_t *rangingTableSet, int8_t shiftCount);

void rangingTableSetInit();

void checkExpiration(Ranging_Table_Set_t *rangingTableSet);


/* -------------------- Calculation Function -------------------- */
float rangingAlgorithm(Timestamp_Tuple_t T1, Timestamp_Tuple_t R1, Timestamp_Tuple_t T2, Timestamp_Tuple_t R2, Timestamp_Tuple_t T3, Timestamp_Tuple_t R3, float Tof12);
float classicCalculatePTof(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);
float calculatePTof(Ranging_Table_t *rangingTable, Ranging_Table_Tr_Rr_Candidate_t candidate);


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
Time_t generateMessage(Ranging_Message_t *rangingMessage);
void processMessage(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo);

#endif