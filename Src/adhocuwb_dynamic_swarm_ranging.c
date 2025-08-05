#include <math.h>
#include <string.h>
#include "adhocuwb_dynamic_swarm_ranging.h"

#ifndef SIMULATION_ENABLE
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "log.h"
#include "adhocuwb_init.h"
#include "adhocuwb_platform.h"
#endif

#ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7
#include "uwb_send_print.h"
#endif


static uint16_t MY_UWB_ADDRESS;
Ranging_Table_Set_t *rangingTableSet;

static QueueHandle_t rxQueue;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;

int16_t distanceReal[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_DIS};
int16_t distanceCalculate[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_DIS};

#ifdef COMPENSATE_ENABLE
static int16_t lastDistanceCalculate[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_DIS};
static int16_t lastSeqNumber[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_SEQ};
static int16_t deltaDistanceUnit[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_SEQ};
#endif


/* -------------------- Base Operation -------------------- */
// comparison of valid parts of timestamps considering cyclic overflow(time_a < time_b)
bool COMPARE_TIME(uint64_t time_a, uint64_t time_b) {
    uint64_t diff = (time_b - time_a) & (UWB_MAX_TIMESTAMP - 1);
    return diff < (UWB_MAX_TIMESTAMP - diff);
}


/* -------------------- Ranging Table Set Operation -------------------- */
// search for index of send list whose Tx seqNumber is same as seqNumber
index_t findSendList(SendList_t *sendList, uint16_t seqNumber) {
    if(seqNumber == NULL_SEQ) {
        return NULL_INDEX;
    }
    index_t index = sendList->topIndex;
    for(int i = 0; i < SEND_LIST_SIZE; i++) {
        if(sendList->Txtimestamps[index].seqNumber == seqNumber) {
            return index;
        }
        index = (index - 1 + SEND_LIST_SIZE) % SEND_LIST_SIZE;
    }
    DEBUG_PRINT("Sequence number %u not found in send list\n", seqNumber);
    return NULL_INDEX;
}

#ifdef COORDINATE_SEND_ENABLE
// update the send list with new Tx and Rx timestamps with coordinate
void updateSendList(SendList_t *sendList, Timestamp_Tuple_t timestampTuple, Coordinate_Tuple_t coordinateTuple) {
    sendList->topIndex = (sendList->topIndex + 1) % SEND_LIST_SIZE;
    sendList->Txtimestamps[sendList->topIndex] = timestampTuple;
    sendList->TxCoordinate = coordinateTuple;
}
#else
// update the send list with new Tx and Rx timestamps without coordinate
void updateSendList(SendList_t *sendList, Timestamp_Tuple_t timestampTuple) {
    sendList->topIndex = (sendList->topIndex + 1) % SEND_LIST_SIZE;
    sendList->Txtimestamps[sendList->topIndex] = timestampTuple;
}
#endif

// init rangingTableTr_Rr_Buffer
void rangingTableTr_Rr_BufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer) {
    rangingTableBuffer->topIndex = NULL_INDEX;
    for (table_index_t i = 0; i < Tr_Rr_BUFFER_SIZE; i++) {
        rangingTableBuffer->candidates[i].Tr = nullTimestampTuple;
        rangingTableBuffer->candidates[i].Rr = nullTimestampTuple;
    }
}

// update Tr in rangingTableTr_Rr_Buffer
void updateRangingTableTr_Buffer(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Tr) {
    rangingTableBuffer->candidates[rangingTableBuffer->topIndex].Tr = Tr;
}

// update Rr in rangingTableTr_Rr_Buffer
void updateRangingTableRr_Buffer(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Rr) {
    rangingTableBuffer->topIndex = (rangingTableBuffer->topIndex + 1) % Tr_Rr_BUFFER_SIZE;
    rangingTableBuffer->candidates[rangingTableBuffer->topIndex].Tr = nullTimestampTuple;
    rangingTableBuffer->candidates[rangingTableBuffer->topIndex].Rr = Rr;
}

// get candidate in rangingTableTr_Rr_Buffer based on Rp and Tf
Ranging_Table_Tr_Rr_Candidate_t rangingTableTr_Rr_BufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tf) {
    index_t index = rangingTableBuffer->topIndex;

    uint64_t leftBound = Rp.timestamp.full % UWB_MAX_TIMESTAMP;
    uint64_t rightBound = Tf.timestamp.full % UWB_MAX_TIMESTAMP;

    Ranging_Table_Tr_Rr_Candidate_t candidate = nullCandidate;

    for (int count = 0; count < Tr_Rr_BUFFER_SIZE; count++) {
        if (rangingTableBuffer->candidates[index].Tr.timestamp.full 
            && rangingTableBuffer->candidates[index].Rr.timestamp.full 
            && COMPARE_TIME(leftBound, rangingTableBuffer->candidates[index].Tr.timestamp.full)
            && COMPARE_TIME(rangingTableBuffer->candidates[index].Rr.timestamp.full, rightBound)) {      
            candidate.Tr = rangingTableBuffer->candidates[index].Tr;
            candidate.Rr = rangingTableBuffer->candidates[index].Rr;
            break;
        }
        index = (index - 1 + Tr_Rr_BUFFER_SIZE) % Tr_Rr_BUFFER_SIZE;
    }

    return candidate;
}

// init rangingTable
void rangingTableInit(Ranging_Table_t *rangingTable) {
    rangingTable->neighborAddress = NULL_ADDRESS;
    rangingTable->ETb = nullTimestampTuple;
    rangingTable->ERb = nullTimestampTuple;
    rangingTable->ETp = nullTimestampTuple;
    rangingTable->ERp = nullTimestampTuple;
    rangingTable->Tb = nullTimestampTuple;
    rangingTable->Rb = nullTimestampTuple;
    rangingTable->Tp = nullTimestampTuple;
    rangingTable->Rp = nullTimestampTuple;
    rangingTableTr_Rr_BufferInit(&rangingTable->TrRrBuffer);
    rangingTable->Tf = nullTimestampTuple;
    rangingTable->Rf = nullTimestampTuple;
    rangingTable->Re = nullTimestampTuple;

    rangingTable->PToF = NULL_TOF;
    rangingTable->EPToF = NULL_TOF;
    rangingTable->continuitySign = false;
    rangingTable->expirationSign = true;
    rangingTable->tableState = UNUSED;
    #ifdef STATE_MACHINE_ENABLE
        rangingTable->rangingState = RANGING_STATE_S1;
    #endif
}

// register a new rangingTable and return the index of rangingTable
table_index_t registerRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address) {
    if (rangingTableSet->size >= RANGING_TABLE_SIZE) {
        DEBUG_PRINT("Ranging table Set is full, cannot register new table\n");
        return NULL_INDEX;
    }

    for(table_index_t index = 0; index < RANGING_TABLE_SIZE; index++) {
        if(rangingTableSet->rangingTable[index].tableState == UNUSED) {
            rangingTableSet->rangingTable[index].neighborAddress = address;
            rangingTableSet->rangingTable[index].tableState = USING;

            // newly registered neighbor is in the initialization phase and cannot trigger calculations in the short term, so their priority is low
            rangingTableSet->priorityQueue[rangingTableSet->size] = index;
            rangingTableSet->size++;

            DEBUG_PRINT("Registered new ranging table entry: Address = %u\n", address);
            return index;
        }
    }

    ASSERT(0 && "Warning: Should not be called\n");
    return NULL_INDEX;
}

// deregister a rangingTable by address and update the priority queue
void deregisterRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address) {
    if(rangingTableSet->size == 0) {
        DEBUG_PRINT("Ranging table Set is empty, cannot deregister table\n");
        return;
    }

    table_index_t tableIdx = NULL_INDEX;
    for(table_index_t i = 0; i < RANGING_TABLE_SIZE; i++) {
        if(rangingTableSet->rangingTable[i].neighborAddress == address &&  rangingTableSet->rangingTable[i].tableState == USING) {
            tableIdx = i;
            break;
        }
    }

    if(tableIdx == NULL_INDEX) {
        ASSERT(0 && "Warning: Should not be called\n");
        return;
    }

    rangingTableInit(&rangingTableSet->rangingTable[tableIdx]);

    table_index_t queueIdx = NULL_INDEX;
    for(table_index_t i = 0; i < rangingTableSet->size; i++) {
        if(rangingTableSet->priorityQueue[i] == tableIdx) {
            queueIdx = i;
            break;
        }
    }

    if(queueIdx == NULL_INDEX) {
        ASSERT(0 && "Warning: Should not be called\n");
        return;
    }

    for(table_index_t i = queueIdx; i < rangingTableSet->size - 1; i++) {
        rangingTableSet->priorityQueue[i] = rangingTableSet->priorityQueue[i + 1];
    }
    rangingTableSet->priorityQueue[rangingTableSet->size - 1] = NULL_INDEX;
    rangingTableSet->size--;

    DEBUG_PRINT("Deregister ranging table entry: Address = %u\n", address);
}

// find the index of rangingTable by address
table_index_t findRangingTable(Ranging_Table_Set_t *rangingTableSet, uint16_t address) {
    if(rangingTableSet->size == 0) {
        // DEBUG_PRINT("Ranging table Set is empty, cannot find table\n");
        return NULL_INDEX;
    }

    for (table_index_t i = 0; i < rangingTableSet->size; i++) {
        table_index_t idx = rangingTableSet->priorityQueue[i];
        if (rangingTableSet->rangingTable[idx].neighborAddress == address && rangingTableSet->rangingTable[idx].tableState == USING) {
            return idx;
        }
    }
    // DEBUG_PRINT("Ranging table Set does not contain the address: %u\n", address);
    return NULL_INDEX;
}

/* fill Ranging Table
    +------+------+------+------+------+------+                   +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |      |                   | ETb  | ERp  |  Tb  |  Rp  |  Tr  | [Rf] |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] | [Re] |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |
    +------+------+------+------+                                 +------+------+------+------+
*/
void fillRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf, Timestamp_Tuple_t Re) {
    rangingTable->Tf = Tf;
    rangingTable->Rf = Rf;
    rangingTable->Re = Re;
}

/* shift Ranging Table
    +------+------+------+------+------+------+                   +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |                   |[ETb] |[ERp] | [Tb] | [Rp] |      |      |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |    ===>    |[ERb] |[ETp] | [Rb] | [Tp] |  Rr  |      |      |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    |    EPToF    |    PToF     |                                 |   [EPToF]   |   [PToF]    |
    +------+------+------+------+                                 +------+------+------+------+
*/
void shiftRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, float PToF) {
    rangingTable->ETb = rangingTable->Tb;
    rangingTable->ERb = rangingTable->Rb;
    rangingTable->ETp = rangingTable->Tp;
    rangingTable->ERp = rangingTable->Rp;
    rangingTable->EPToF = rangingTable->PToF;

    rangingTable->Tb = Tr;
    rangingTable->Rb = Rr;
    rangingTable->Tp = rangingTable->Tf;
    rangingTable->Rp = rangingTable->Rf;
    rangingTable->PToF = PToF;

    rangingTable->Tf = nullTimestampTuple;
    rangingTable->Rf = nullTimestampTuple;
    rangingTable->Re = nullTimestampTuple;

    // whether the table is contiguous
    if(rangingTable->ERb.seqNumber + 1 == rangingTable->Rb.seqNumber && rangingTable->ERp.seqNumber + 1 == rangingTable->Rp.seqNumber) {
        if(rangingTable->continuitySign == false) {
            rangingTable->continuitySign = true;
            // correcting PToF
            float correctPToF = classicCalculatePToF(rangingTable->ETp, rangingTable->ERp, rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp);
            rangingTable->PToF = correctPToF;
            DEBUG_PRINT("[correct PToF]: correct PToF = %f\n", (double)correctPToF);
        }
    }
    else {
        rangingTable->continuitySign = false;
    }
}

/* replace Ranging Table
    +------+------+------+------+------+------+                   +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |                   | ETb  | ERp  | [Tb] | [Rp] |  Tr  |      |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |    ===>    | ERb  | ETp  | [Rb] | [Tp] |  Rr  |      |      |
    +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
    |    EPToF    |    PToF     |                                 |    EPToF    |   [PToF]    |
    +------+------+------+------+                                 +------+------+------+------+
*/
void replaceRangingTable(Ranging_Table_t *rangingTable, Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, float PToF) {
    rangingTable->Tb = Tb;
    rangingTable->Rb = Rb;
    rangingTable->Tp = Tp;
    rangingTable->Rp = Rp;
    rangingTable->PToF = PToF;

    rangingTable->continuitySign = false;
}

// update the circular priority queue
void updatePriorityQueue(Ranging_Table_Set_t *rangingTableSet, int8_t shiftCount) {
    // no need to shift
    if(rangingTableSet->size <= 1 || shiftCount <= 0 || rangingTableSet->size == shiftCount) {
        return;
    }

    index_t *tmpQueue = malloc(sizeof(index_t) * rangingTableSet->size);
    if (!tmpQueue) {
        ASSERT(0 && "Warning: Should not be called\n");
    }

    for (index_t i = 0; i < rangingTableSet->size; i++) {
        tmpQueue[i] = rangingTableSet->priorityQueue[(i + shiftCount) % rangingTableSet->size];
    }

    for (index_t i = 0; i < rangingTableSet->size; i++) {
        rangingTableSet->priorityQueue[i] = tmpQueue[i];
    }

    free(tmpQueue);
}

// init rangingTableSet
void rangingTableSetInit() {
    rangingTableSet = (Ranging_Table_Set_t*)malloc(sizeof(Ranging_Table_Set_t));
    rangingTableSet->size = 0;
    rangingTableSet->localSeqNumber = NULL_SEQ;
    rangingTableSet->sendList.topIndex = NULL_INDEX;
    for (int i = 0; i < SEND_LIST_SIZE; i++) {
        rangingTableSet->sendList.Txtimestamps[i] = nullTimestampTuple;
    }
    #ifdef COORDINATE_SEND_ENABLE
        rangingTableSet->sendList.TxCoordinate = nullCoordinateTuple;
    #endif
    for (int i = 0; i < RANGING_TABLE_SIZE; i++) {
        rangingTableInit(&rangingTableSet->rangingTable[i]);
        rangingTableSet->lastRxtimestamp[i] = nullTimestampTuple;
        rangingTableSet->priorityQueue[i] = NULL_INDEX;
    }
    rangingTableSet->mutex = xSemaphoreCreateMutex();
}

// check expirationSign of rangingTables and deregister rangingTable expired
void checkExpiration(Ranging_Table_Set_t *rangingTableSet) {
    for(table_index_t i = rangingTableSet->size; i > 0; i--) {
        table_index_t idx = rangingTableSet->priorityQueue[i - 1];
        if(rangingTableSet->rangingTable[idx].expirationSign == true) {
            deregisterRangingTable(rangingTableSet, rangingTableSet->rangingTable[idx].neighborAddress);
        }
        else {
            rangingTableSet->rangingTable[idx].expirationSign = true;
        }
    }
}

// get substate of rangingTable
int getCurrentSubstate(Ranging_Table_t *rangingTable) {
    if (rangingTable->PToF == NULL_TOF && rangingTable->EPToF == NULL_TOF) {
        return RANGING_SUBSTATE_S1;
    } 
    else if (rangingTable->EPToF == NULL_TOF) {
        return RANGING_SUBSTATE_S2;
    } 
    else {
        return RANGING_SUBSTATE_S3;
    }
}


/* -------------------- Calculation Function -------------------- */
/* ranging algorithm
       R1   <--Db-->    T2      <--Rb-->       R3

    T1      <--Ra-->      R2    <--Da-->    T3
*/
float rangingAlgorithm(Timestamp_Tuple_t T1, Timestamp_Tuple_t R1, Timestamp_Tuple_t T2, Timestamp_Tuple_t R2, Timestamp_Tuple_t T3, Timestamp_Tuple_t R3, float ToF12) {
    uint64_t Ra = (R2.timestamp.full - T1.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    uint64_t Db = (T2.timestamp.full - R1.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    uint64_t Da = (T3.timestamp.full - R2.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    uint64_t Rb = (R3.timestamp.full - T2.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    uint64_t diffA = Ra - Da;
    uint64_t diffB = Rb - Db;

    float Ra_Da = Ra / (float)Da;
    float Rb_Db = Rb / (float)Db;

    float ToF23 = NULL_TOF;

    // meet the convergence condition
    if(Ra_Da < CONVERGENCE_THRESHOLD || Rb_Db < CONVERGENCE_THRESHOLD) {
        if(Ra_Da < Rb_Db) {
            ToF23 = (float)((diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / 2.0f - Ra * ToF12) / (float)Da;
        }
        else {
            ToF23 = (float)((diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / 2.0f - Rb * ToF12) / (float)Db;
        }

        // abnormal result
        float D = (ToF23 * VELOCITY) / 2;
        if(D < LOWER_BOUND_DISTANCE || D > UPPER_BOUND_DISTANCE) {
            DEBUG_PRINT("[rangingAlgorithm]: result is out of range.\n");
            return NULL_TOF;
        }
    }
    else {
        #ifdef CLASSIC_SUPPORT_ENABLE
            DEBUG_PRINT("[rangingAlgorithm]: not meet the convergence condition, use classic algorithm.\n");
            ToF23 = classicCalculatePToF(T1, R1, T2, R2, T3, R3);
        #else
            DEBUG_PRINT("[rangingAlgorithm]: not meet the convergence condition.\n");
        #endif
    }

    return ToF23;
}

/* timestamps for classicCalculatePToF
       Rp   <--Db-->   Tr     <--Rb-->      Rf

    Tp      <--Ra-->     Rr   <--Da-->   Tf
*/ 
float classicCalculatePToF(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {
    float curPToF = NULL_TOF;
    
    // check completeness
    if(Tp.seqNumber == NULL_SEQ || Rp.seqNumber == NULL_SEQ || Tr.seqNumber == NULL_SEQ
        || Rr.seqNumber == NULL_SEQ || Tf.seqNumber == NULL_SEQ || Rf.seqNumber == NULL_SEQ) {
        DEBUG_PRINT("Warning: Data calculation is not complete\n");
        return NULL_TOF;
    }

    int64_t Ra = (Rr.timestamp.full - Tp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Da = (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Rb = (Rf.timestamp.full - Tr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Db = (Tr.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    int64_t diffA = Ra - Da;
    int64_t diffB = Rb - Db;

    curPToF = (diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / (float)(Ra + Db + Rb + Da);

    return curPToF;
}

/* calculate the Time of Flight (ToF) based on the timestamps in the ranging table
    +------+------+------+------+------+------+
    | ETb  | ERp  |  Tb  |  Rp  |  Tr  |  Rf  |
    +------+------+------+------+------+------+------+
    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |  Re  |
    +------+------+------+------+------+------+------+
    |    EPToF    |    PToF     |
    +------+------+------+------+
*/
float calculatePToF(Ranging_Table_t *rangingTable, Ranging_Table_Tr_Rr_Candidate_t candidate) {
    Timestamp_Tuple_t Tr = candidate.Tr;
    Timestamp_Tuple_t Rr = candidate.Rr;
    Timestamp_Tuple_t Tf = rangingTable->Tf;
    Timestamp_Tuple_t Rf = rangingTable->Rf;

    float tmpPToF = NULL_TOF;
    float curPToF = NULL_TOF;
    
    // check completeness
    if(Tr.seqNumber == NULL_SEQ || Rr.seqNumber == NULL_SEQ || Tf.seqNumber == NULL_SEQ || Rf.seqNumber == NULL_SEQ) {
        /* type1
            +------+------+------+------+           +------+------+------+------+
            |  Tb  |  Rp  |  Tr  |      |           | ERp  |  Tb  |  Rp  |  Tr  |
            +------+------+------+------+   ===>    +------+------+------+------+
            |  Rb  |  Tp  |  Rr  |      |           | ETp  |  Rb  |  Tp  |  Rr  |
            +------+------+------+------+           +------+------+------+------+
        */ 
        if(Tr.seqNumber != NULL_SEQ && Rr.seqNumber != NULL_SEQ) {
            DEBUG_PRINT("[calculatePToF]: Data calculation is not complete, pair of Tf-Rf is losed\n");
            tmpPToF = rangingAlgorithm(rangingTable->ETp, rangingTable->ERp, 
                                             rangingTable->Tb, rangingTable->Rb, 
                                             rangingTable->Tp, rangingTable->Rp, (rangingTable->EPToF + rangingTable->PToF) / 2);

            if(tmpPToF != NULL_TOF) {
                curPToF = rangingAlgorithm(rangingTable->Tb, rangingTable->Rb, 
                                           rangingTable->Tp, rangingTable->Rp,
                                           Tr, Rr, tmpPToF);
                // use tmpPToF as the estimated value
                curPToF = (curPToF == NULL_TOF) ? tmpPToF : curPToF;
            }
            else {
                return NULL_TOF;
            }
        }
        /* type2
            +------+------+------+------+           +------+------+------+------+
            |  Tb  |  Rp  |      |  Rf  |           | ETb  | ERp  |  Tb  |  Rf  |
            +------+------+------+------+   ===>    +------+------+------+------+
            |  Rb  |  Tp  |      |  Tf  |           | ERb  | ETp  |  Rb  |  Tf  |
            +------+------+------+------+           +------+------+------+------+
        */
        else if(Tf.seqNumber != NULL_SEQ && Rf.seqNumber != NULL_SEQ) {
            DEBUG_PRINT("[calculatePToF]: Data calculation is not complete, pair of Tr-Rr is losed\n");
            tmpPToF = rangingAlgorithm(rangingTable->ETb, rangingTable->ERb, 
                                             rangingTable->ETp, rangingTable->ERp, 
                                             rangingTable->Tb, rangingTable->Rb, (rangingTable->EPToF + rangingTable->PToF) / 2);

            if(tmpPToF != NULL_TOF) {
                curPToF = rangingAlgorithm(rangingTable->ETp, rangingTable->ERp, 
                                           rangingTable->Tb, rangingTable->Rb,
                                           rangingTable->Tf, rangingTable->Rf, tmpPToF);
                // use tmpPToF as the estimated value
                curPToF = (curPToF == NULL_TOF) ? tmpPToF : curPToF;

                // replace pair of Tp-Rp with pair of Tf-Rf
                replaceRangingTable(rangingTable, rangingTable->Tb, rangingTable->Rb, rangingTable->Tf, rangingTable->Rf, curPToF);
            }
            else {
                return NULL_TOF;
            }
        }
        /* type3
            +------+------+------+------+      
            |  Tb  |  Rp  |      |      |          
            +------+------+------+------+   ===>    PToF
            |  Rb  |  Tp  |      |      |           
            +------+------+------+------+  
        */
        else {
            DEBUG_PRINT("[calculatePToF]: Data calculation is not complete, pair of Tf-Rf and Tr-Rr are losed\n");
            return NULL_TOF;
        }
    }

    else {
        /* type4
     
            +------+------+------+------+
            |  Tb  |  Rp  |  Tr  |  Rf  |
            +------+------+------+------+   
            |  Rb  |  Tp  |  Rr  |  Tf  | 
            +------+------+------+------+    
        */ 
        tmpPToF = rangingAlgorithm(rangingTable->Tb, rangingTable->Rb, 
                                         rangingTable->Tp, rangingTable->Rp, 
                                         Tr, Rr, rangingTable->PToF);

        if(tmpPToF != NULL_TOF) {
            curPToF = rangingAlgorithm(rangingTable->Tp, rangingTable->Rp, 
                                       Tr, Rr,
                                       rangingTable->Tf, rangingTable->Rf, tmpPToF);
            // use tmpPToF as the estimated value
            curPToF = (curPToF == NULL_TOF) ? tmpPToF : curPToF;
        }
        else {
            return NULL_TOF;
        }
    }

    return curPToF;
}

void continuousPacketLossHandler(Ranging_Table_t *rangingTable, Ranging_Table_Tr_Rr_Candidate_t candidate) {
    DEBUG_PRINT("[continuousPacketLossHandler]: A long-term continuous packet loss event occurred.\n");
    
    float curPToF_avg = (classicCalculatePToF(rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr) 
                       + classicCalculatePToF(rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr, rangingTable->Tf, rangingTable->Rf)) / 2;
    distanceCalculate[rangingTable->neighborAddress] = (curPToF_avg * VELOCITY) / 2;
    
    updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);

    shiftRangingTable(rangingTable, candidate.Tr, candidate.Rr, curPToF_avg);

    // goto S4.2
    rangingTable->ETb = nullTimestampTuple;
    rangingTable->ERb = nullTimestampTuple;
    rangingTable->EPToF = NULL_TOF;

    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S5_RX{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, RANGING_SUBSTATE_S3, curState, RANGING_SUBSTATE_S2);
}


/* -------------------- Print Function -------------------- */
void printRangingMessage(Ranging_Message_t *rangingMessage) {
    DEBUG_PRINT("\n{rangingMessage}\n");

    DEBUG_PRINT("srcAddress: %u\n", rangingMessage->header.srcAddress);
    DEBUG_PRINT("msgSequence: %u\n", rangingMessage->header.msgSequence);

    DEBUG_PRINT("[Tx]\n");
    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        DEBUG_PRINT("seqNumber: %u, timestamp: %llu\n", rangingMessage->header.Txtimestamps[i].seqNumber, rangingMessage->header.Txtimestamps[i].timestamp.full % UWB_MAX_TIMESTAMP);
    }

    DEBUG_PRINT("[Rx]:\n");
    for(int i = 0; i < MESSAGE_BODYUNIT_SIZE; i++){
        DEBUG_PRINT("address: %u, seqNumber: %u, timestamp: %llu\n", rangingMessage->bodyUnits[i].address, rangingMessage->bodyUnits[i].seqNumber, rangingMessage->bodyUnits[i].timestamp.full % UWB_MAX_TIMESTAMP);
    }
    DEBUG_PRINT("\n");
}

void printSendList(SendList_t *sendList) {
    DEBUG_PRINT("\n[sendList]\n");
    index_t index = sendList->topIndex;
    #ifdef COORDINATE_SEND_ENABLE
        DEBUG_PRINT("location: x = %u, y = %u, z = %u\n", sendList->TxCoordinate.x,  sendList->TxCoordinate.y,  sendList->TxCoordinate.z);
    #endif
    for(int i = 0; i < SEND_LIST_SIZE; i++) {
        DEBUG_PRINT("seqNumber: %u, timestamp: %llu\n", sendList->Txtimestamps[index].seqNumber, sendList->Txtimestamps[index].timestamp.full % UWB_MAX_TIMESTAMP);
        index = (index - 1 + SEND_LIST_SIZE) % SEND_LIST_SIZE;
    }
}

void printRangingTable(Ranging_Table_t *rangingTable) {
    DEBUG_PRINT("neighborAddress: %u\n", rangingTable->neighborAddress);
    DEBUG_PRINT("(ETb) seqNumber: %u, timestamp: %llu\n", rangingTable->ETb.seqNumber, rangingTable->ETb.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(ERb) seqNumber: %u, timestamp: %llu\n", rangingTable->ERb.seqNumber, rangingTable->ERb.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(ETp) seqNumber: %u, timestamp: %llu\n", rangingTable->ETp.seqNumber, rangingTable->ETp.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(ERp) seqNumber: %u, timestamp: %llu\n", rangingTable->ERp.seqNumber, rangingTable->ERp.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Tb) seqNumber: %u, timestamp: %llu\n", rangingTable->Tb.seqNumber, rangingTable->Tb.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Rb) seqNumber: %u, timestamp: %llu\n", rangingTable->Rb.seqNumber, rangingTable->Rb.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Tp) seqNumber: %u, timestamp: %llu\n", rangingTable->Tp.seqNumber, rangingTable->Tp.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Rp) seqNumber: %u, timestamp: %llu\n", rangingTable->Rp.seqNumber, rangingTable->Rp.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Tr_Rr_Buffer)\n");
    index_t index = rangingTable->TrRrBuffer.topIndex;
    for(int i = 0; i < Tr_Rr_BUFFER_SIZE; i++) {
        DEBUG_PRINT("\tseqNumber: %u, timestamp: %llu  -->  seqNumber: %u, timestamp: %llu\n", 
            rangingTable->TrRrBuffer.candidates[index].Tr.seqNumber, rangingTable->TrRrBuffer.candidates[index].Tr.timestamp.full % UWB_MAX_TIMESTAMP,
            rangingTable->TrRrBuffer.candidates[index].Rr.seqNumber, rangingTable->TrRrBuffer.candidates[index].Rr.timestamp.full % UWB_MAX_TIMESTAMP);
        index = (index - 1 + Tr_Rr_BUFFER_SIZE) % Tr_Rr_BUFFER_SIZE;
    }
    DEBUG_PRINT("(Tf) seqNumber: %u, timestamp: %llu\n", rangingTable->Tf.seqNumber, rangingTable->Tf.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Rf) seqNumber: %u, timestamp: %llu\n", rangingTable->Rf.seqNumber, rangingTable->Rf.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("(Re) seqNumber: %u, timestamp: %llu\n", rangingTable->Re.seqNumber, rangingTable->Re.timestamp.full % UWB_MAX_TIMESTAMP);
    
    DEBUG_PRINT("PToF = %f, EPToF = %f, continuitySign = %s\n", (double)rangingTable->PToF, (double)rangingTable->EPToF, rangingTable->continuitySign == true ? "true" : "false");
}

void printPriorityQueue(Ranging_Table_Set_t *rangingTableSet) {
    DEBUG_PRINT("\n[priorityQueue]\n");
    for(int i = 0; i < rangingTableSet->size; i++) {
        DEBUG_PRINT("priority: %d -> neighborAddress: %u\n", i + 1, rangingTableSet->rangingTable[rangingTableSet->priorityQueue[i]].neighborAddress);
    }
}

void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet) {
    DEBUG_PRINT("\n{rangingTableSet}\n");
    DEBUG_PRINT("size: %u\n", rangingTableSet->size);
    DEBUG_PRINT("localSeqNumber: %lu\n", rangingTableSet->localSeqNumber);

    printPriorityQueue(rangingTableSet);

    printSendList(&rangingTableSet->sendList);

    DEBUG_PRINT("\n[lastRxtimestamp]\n");
    for(int i = 0; i < rangingTableSet->size; i++) {
        DEBUG_PRINT("neighborAddress: %u, seqNumber: %u, timestamp: %llu\n", rangingTableSet->rangingTable[i].neighborAddress, rangingTableSet->lastRxtimestamp[i].seqNumber, rangingTableSet->lastRxtimestamp[i].timestamp.full % UWB_MAX_TIMESTAMP);
    }

    DEBUG_PRINT("\n[rangingTable]\n");
    for(int i = 0; i < rangingTableSet->size; i++) {
        printRangingTable(&rangingTableSet->rangingTable[rangingTableSet->priorityQueue[i]]);
    }
}

void printclassicCalculateTuple(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {
    DEBUG_PRINT("\nRp[seq=%u, ts=%llu] <--%llu--> Tr[seq=%u, ts=%llu] <--%llu--> Rf[seq=%u, ts=%llu]\n",
        Rp.seqNumber, Rp.timestamp.full % UWB_MAX_TIMESTAMP, (Tr.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP, Tr.seqNumber, Tr.timestamp.full % UWB_MAX_TIMESTAMP, (Rf.timestamp.full - Tr.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Rf.seqNumber, Rf.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("\nTp[seq=%u, ts=%llu] <--%llu--> Rr[seq=%u, ts=%llu] <--%llu--> Tf[seq=%u, ts=%llu]\n",
        Tp.seqNumber, Tp.timestamp.full % UWB_MAX_TIMESTAMP, (Rr.timestamp.full - Tp.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Rr.seqNumber, Rr.timestamp.full % UWB_MAX_TIMESTAMP, (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Tf.seqNumber, Tf.timestamp.full % UWB_MAX_TIMESTAMP);
}

void printCalculateTuple(Timestamp_Tuple_t Tb, Timestamp_Tuple_t Rb, Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp, Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr, Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf) {
    DEBUG_PRINT("\nTb[seq=%u, ts=%llu] <--%llu--> Rp[seq=%u, ts=%llu] <--%llu--> Tr[seq=%u, ts=%llu] <--%llu--> Rf[seq=%u, ts=%llu]\n",
                Tb.seqNumber,Tb.timestamp.full % UWB_MAX_TIMESTAMP,
                (Rp.timestamp.full -Tb.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Rp.seqNumber, Rp.timestamp.full % UWB_MAX_TIMESTAMP,
                (Tr.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Tr.seqNumber, Tr.timestamp.full % UWB_MAX_TIMESTAMP,
                (Rf.timestamp.full - Tr.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Rf.seqNumber, Rf.timestamp.full % UWB_MAX_TIMESTAMP);
    DEBUG_PRINT("\nRb[seq=%u, ts=%llu] <--%llu--> Tp[seq=%u, ts=%llu] <--%llu--> Rr[seq=%u, ts=%llu] <--%llu--> Tf[seq=%u, ts=%llu]\n",
                Rb.seqNumber,Rb.timestamp.full % UWB_MAX_TIMESTAMP,
                (Tp.timestamp.full -Rb.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP,Tp.seqNumber,Tp.timestamp.full % UWB_MAX_TIMESTAMP,
                (Rr.timestamp.full -Tp.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Rr.seqNumber, Rr.timestamp.full % UWB_MAX_TIMESTAMP,
                (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP ) % UWB_MAX_TIMESTAMP, Tf.seqNumber, Tf.timestamp.full % UWB_MAX_TIMESTAMP);
}


/* -------------------- State Machine Operation -------------------- */
#ifdef STATE_MACHINE_ENABLE
static void RESERVED_STUB(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[RESERVED_STUB]: Should not be called\n");
}

static void S1_TX(Ranging_Table_t *rangingTable) {
    /* Don't update Tx here since sending message is an async action, we put all Tx in sendList
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |      |      |      |  S1  |            |      |      |      |      |      |      |  S2  |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
        |      |      |      |      |      |      |      |    ===>    |      |      |      |      |      | [Tf] |      |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+   
        |             |             |                                 |             |             |
        +------+------+------+------+                                 +------+------+------+------+
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S2;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S1_TX]: S%d -> S%d\n", prevState, curState);
}

static void S1_RX(Ranging_Table_t *rangingTable) {
    /* Invalid reception of rangingMessage
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |      |      |      |  S1  |            |      |      |      |      | [Tr] | [Rf] |  S1  |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+   
        |      |      |      |      |      |      |      |            |      |      |      |      |      |      | [Re] |
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+   
        |             |             |                                 |             |             |
        +------+------+------+------+                                 +------+------+------+------+
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S1;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S1_RX{local_%u, neighbor_%u}]: S%d -> S%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curState);
}

static void S1_RX_NO(Ranging_Table_t *rangingTable) {
    /* Invalid reception of rangingMessage
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |      |      |      |  S1  |            |      |      |      |      | [Tr] |      |  S1  |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+   
        |      |      |      |      |      |      |      |            |      |      |      |      |      |      | [Re] |
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+   
        |             |             |                                 |             |             |
        +------+------+------+------+                                 +------+------+------+------+
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S1;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S1_RX_NO{local_%u, neighbor_%u}]: S%d -> S%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curState);
}

static void S2_TX(Ranging_Table_t *rangingTable) {
    /* Don't update Tx here since sending message is an async action, we put all Tx in sendList
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |      |      |      |  S2  |            |      |      |      |      |      |      |  S2  |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+   
        |      |      |      |      |      |  Tf  |      |            |      |      |      |      |      | [Tf] |      |
        +------+------+------+------+------+------+------+    <==     +------+------+------+------+------+------+------+   
        |             |             |                                 |             |             |
        +------+------+------+------+                                 +------+------+------+------+
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S2;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S2_TX]: S%d -> S%d\n", prevState, curState);
}

static void S2_RX(Ranging_Table_t *rangingTable) {
    /* Data in rangingTable shift quickly, directly skipping S3 and entering S4
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |      |      |      |  S2  |            |      |      |      |      | [Tr] | [Rf] |  S3  |            |      |      |      |  Rp  |      |      | S4.1 |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |      |      |  Tf  |      |    ===>    |      |      |      |      |      |  Tf  | [Re] |    ===>    |      |      |      |  Tp  |  Rr  |      |      |
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |             |             |                                 |             |             |                                 |             |             | 
        +------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;

    updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
    shiftRangingTable(rangingTable, nullTimestampTuple, nullTimestampTuple, NULL_TOF);

    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S2_RX{local_%u, neighbor_%u}]: S%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curState, RANGING_SUBSTATE_S1);
}

static void S2_RX_NO(Ranging_Table_t *rangingTable) {
    /* Lack of timestamp, no need to shift
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        |      |      |      |      |      |      |  S2  |            |      |      |      |      | [Tr] |      |  S2  |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+           
        |      |      |      |      |      |  Tf  |      |            |      |      |      |      |      |  Tf  | [Re] |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+           
        |             |             |                                 |             |             |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S2;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S2_RX_NO{local_%u, neighbor_%u}]: S%d -> S%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curState);
}

static void S3_TX(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[S3_TX]: Should not be called\n");
}

static void S3_RX(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[S3_RX]: Should not be called\n");
}

static void S3_RX_NO(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[S3_RX_NO]: Should not be called\n");
}

static void S4_TX(Ranging_Table_t *rangingTable) {
    /* Don't update Tx here since sending message is an async action, we put all Tx in sendList
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        |      |      |      |  Rp  |      |      | S4.1 |            |      |      |      |  Rp  |      |      | S5.1 |          
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        |      |      |      |  Tp  |  Rr  |      |      |    ===>    |      |      |      |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        |             |             |                                 |             |             |                                
        +------+------+------+------+                                 +------+------+------+------+

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  |      |      | S5.2 |          
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |    ===>    |      | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        |             |    PToF     |                                 |             |    PToF     |                                
        +------+------+------+------+                                 +------+------+------+------+ 

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |          
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+           
        |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S5;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    int curSubstate = getCurrentSubstate(rangingTable);
    DEBUG_PRINT("[S4_TX]: S%d.%d -> S%d.%d\n", prevState, curSubstate, curState, curSubstate);
}

static void S4_RX(Ranging_Table_t *rangingTable) {
    /* Invalid reception of rangingMessage
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      |      |      |  Rp  |      |      | S4.1 |            |      |      |      |  Rp  | [Tr] | [Rf] | S4.1 |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
        |      |      |      |  Tp  |  Rr  |      |      |            |      |      |      |  Tp  |  Rr  |      | [Re] |
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
        |             |             |                                 |             |             |                                
        +------+------+------+------+                                 +------+------+------+------+

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
        |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.2 |
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
        |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |            |      | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
        |             |    PToF     |                                 |             |    PToF     |                                
        +------+------+------+------+                                 +------+------+------+------+ 

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.3 |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+         
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+          
        |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;
    
    int curSubstate = getCurrentSubstate(rangingTable);
    DEBUG_PRINT("[S4_RX{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curSubstate, curState, curSubstate);
}

static void S4_RX_NO(Ranging_Table_t *rangingTable) {
    // Calculate ToF, but not shift rangingTable
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    int curSubstate = getCurrentSubstate(rangingTable);

    switch (curSubstate) {
        case RANGING_SUBSTATE_S1: {
            /* initialize               ===>    update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      |      |      |  Rp  |      |      | S4.1 |            |      |      |      |  Rp  | [Tr] |      | S4.1 |
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
                |      |      |      |  Tp  |  Rr  |      |      |            |      |      |      |  Tp  |  Rr  |      | [Re] |
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
                |             |             |                                 |             |             |                                
                +------+------+------+------+                                 +------+------+------+------+
            */
            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        case RANGING_SUBSTATE_S2: {
            /* swarm ranging            ===>    calculate and update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] |      | S4.2 |
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
                |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |            |      | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
                |             |    PToF     |                                 |             |    PToF     |                                
                +------+------+------+------+                                 +------+------+------+------+ 
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
            float curPToF = classicCalculatePToF(rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr);
            distanceCalculate[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? ((rangingTable->PToF * VELOCITY) / 2) : ((curPToF * VELOCITY) / 2);

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        case RANGING_SUBSTATE_S3: {
            /* dynamic swarm ranging    ===>    calculate and update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
                | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] |      | S4.3 |          
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+         
                | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+          
                |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                
                +------+------+------+------+                                 +------+------+------+------+ 
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
            float curPToF = calculatePToF(rangingTable, candidate);
            distanceCalculate[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? (rangingTable->PToF * VELOCITY) / 2 : (curPToF * VELOCITY) / 2;

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        default: {
            ASSERT(0 && "[S4_RX_NO]: Should not be called\n");
            break;
        }
    }

    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S4_RX_NO{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curSubstate, curState, curSubstate);
}

static void S5_TX(Ranging_Table_t *rangingTable) {
    /* Don't update Tx here since sending message is an async action, we put all Tx in sendList
        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        |      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  |      |      | S5.1 |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+           
        |      |      |      |  Tp  |  Rr  |  Tf  |      |            |      |      |      |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+           
        |             |             |                                 |             |             |                                
        +------+------+------+------+                                 +------+------+------+------+

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        |      | ERp  |  Tb  |  Rp  |      |      | S5.2 |            |      | ERp  |  Tb  |  Rp  |      |      | S5.2 |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+           
        |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |            |      | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+           
        |             |    PToF     |                                 |             |    PToF     |                                
        +------+------+------+------+                                 +------+------+------+------+ 

        +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
        | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |          
        +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+           
        | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |   
        +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+           
        |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                
        +------+------+------+------+                                 +------+------+------+------+ 
    */
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S5;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    int curSubstate = getCurrentSubstate(rangingTable);
    DEBUG_PRINT("[S5_TX]: S%d.%d -> S%d.%d\n", prevState, curSubstate, curState, curSubstate);
}

static void S5_RX(Ranging_Table_t *rangingTable) {
    // Calculate ToF, shift the rangingTable, calculation is quick, directly skipping S6 and entering S4
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    int curSubstate = getCurrentSubstate(rangingTable);

    switch (curSubstate) {
        case RANGING_SUBSTATE_S1: {
            /* initialize               ===>    calculate, update and shift
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  | [Tr] | [Rf] | S6.1 |            |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      |      |      |  Tp  |  Rr  |  Tf  |      |    ===>    |      |      |      |  Tp  |  Rr  |  Tf  | [Re] |    ===>    |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |             |             |                                 |             |             |                                 |             |    PToF     |
                +------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, rangingTable->Tf);
            float curPToF = classicCalculatePToF(rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr, rangingTable->Tf, rangingTable->Rf);
            if(curPToF != NULL_TOF) {
                distanceCalculate[rangingTable->neighborAddress] = (curPToF * VELOCITY) / 2;
            }

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);

            if(curPToF != NULL_TOF) {
                shiftRangingTable(rangingTable, candidate.Tr, candidate.Rr, curPToF);
            }
            break;
        }

        case RANGING_SUBSTATE_S2: {
            /* swarm ranging            ===>    calculate, update and shift
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      | ERp  |  Tb  |  Rp  |      |      | S5.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S6.2 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |    ===>    |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |             |    PToF     |                                 |             |    PToF     |                                 |   EPToF     |    PToF     |
                +------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, rangingTable->Tf);
            float curPToF = classicCalculatePToF(rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr, rangingTable->Tf, rangingTable->Rf);
            distanceCalculate[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? (rangingTable->PToF * VELOCITY) / 2 : (curPToF * VELOCITY) / 2;

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);

            if(curPToF != NULL_TOF) {
                shiftRangingTable(rangingTable, candidate.Tr, candidate.Rr, curPToF);
            }
            break;
        }

        case RANGING_SUBSTATE_S3: {
            /* dynamic swarm ranging    ===>    calculate, update and shift
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S6.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |    ===>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                 |   EPToF     |    PToF     |
                +------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, rangingTable->Tf);
            
            uint16_t seqGap = rangingTable->Re.seqNumber - candidate.Rr.seqNumber;
            
            // long-term continuous packet loss
            if(candidate.Tr.seqNumber != NULL_SEQ && candidate.Rr.seqNumber != NULL_SEQ && seqGap >= SEQGAP_THRESHOLD) {
                continuousPacketLossHandler(rangingTable, candidate);
                return;
            }

            // normal
            else {
                float curPToF = calculatePToF(rangingTable, candidate);
                distanceCalculate[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? (rangingTable->PToF * VELOCITY) / 2 : (curPToF * VELOCITY) / 2;

                updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);

                if(curPToF != NULL_TOF && candidate.Tr.seqNumber != NULL_SEQ && candidate.Rr.seqNumber != NULL_SEQ) {
                    shiftRangingTable(rangingTable, candidate.Tr, candidate.Rr, curPToF);
                }
            }
            break;
        }
        default: {
            ASSERT(0 && "[S5_RX]: Should not be called\n");
            break;
        }
    }

    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S5_RX{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curSubstate, curState, (curSubstate == RANGING_SUBSTATE_S3) ? RANGING_SUBSTATE_S3 : curSubstate + 1);
}

static void S5_RX_NO(Ranging_Table_t *rangingTable) {
    // Calculate ToF, but not shift rangingTable
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    int curSubstate = getCurrentSubstate(rangingTable);

    switch (curSubstate) {
        case RANGING_SUBSTATE_S1: {
            /* initialize               ===>    update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  | [Tr] |      | S5.1 |
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
                |      |      |      |  Tp  |  Rr  |  Tf  |      |            |      |      |      |  Tp  |  Rr  |  Tf  | [Re] |
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
                |             |             |                                 |             |             |                                
                +------+------+------+------+                                 +------+------+------+------+
            */
            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        case RANGING_SUBSTATE_S2: {
            /* swarm ranging            ===>    calculate and update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] |      | S4.2 |
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
                |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |            |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |   
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
                |             |    PToF     |                                 |             |    PToF     |                                
                +------+------+------+------+                                 +------+------+------+------+ 
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
            float curPToF = classicCalculatePToF(rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr);
            distanceCalculate[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? (rangingTable->PToF * VELOCITY) / 2 : (curPToF * VELOCITY) / 2;

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        case RANGING_SUBSTATE_S3: {
            /* dynamic swarm ranging    ===>    calculate and update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
                | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] |      | S4.3 |          
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+         
                | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |   
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+          
                |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                
                +------+------+------+------+                                 +------+------+------+------+ 
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
            float curPToF = calculatePToF(rangingTable, candidate);
            distanceCalculate[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? (rangingTable->PToF * VELOCITY) / 2 : (curPToF * VELOCITY) / 2;

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        default: {
            ASSERT(0 && "[S5_RX_NO]: Should not be called\n");
            break;
        }
    }

    rangingTable->rangingState = RANGING_STATE_S5;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    DEBUG_PRINT("[S5_RX_NO{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curSubstate, curState, curSubstate);
}

static void S6_TX(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[S6_TX]: Should not be called\n");
}

static void S6_RX(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[S6_RX]: Should not be called\n");
}

static void S6_RX_NO(Ranging_Table_t *rangingTable) {
    ASSERT(0 && "[S6_RX_NO]: Should not be called\n");
}

/* State Machine
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      |      |      |      | [Tr] | [Rf] |  S1  |            |      |      |      |      |      |      |  S1  |            |      |      |      |      | [Tr] |      |  S1  |
+------+------+------+------+------+------+------+     Rx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      |      |      |      |      |      | [Re] |    <==>    |      |      |      |      |      |      |      |    <==>    |      |      |      |      |      |      | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |             |                                 |             |             |                                 |             |             |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
                                                                                          |
                                                                                          |  Tx
                                                                                          |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      |      |      |      |      |      |  S2  |            |      |      |      |      |      |      |  S2  |            |      |      |      |      | [Tr] |      |  S2  |
+------+------+------+------+------+------+------+     Tx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      |      |      |      |      | [Tf] |      |    <==>    |      |      |      |      |      | [Tf] |      |    <==>    |      |      |      |      |      |  Tf  | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |             |                                 |             |             |                                 |             |             |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+
                                                                                          |
                                                                                          |  Rx
                                                                                          |
                                                              +------+------+------+------+------+------+------+
                                                              |      |      |      |      | [Tr] | [Rf] |  S3  |
                                                              +------+------+------+------+------+------+------+
                                                              |      |      |      |      |      |  Tf  | [Re] |
                                                              +------+------+------+------+------+------+------+
                                                              |             |             |
                                                              +------+------+------+------+
                                                                                          |
                                                                                          |  shift
                                                                                          |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      |      |      |  Rp  | [Tr] | [Rf] | S4.1 |            |      |      |      |  Rp  |      |      | S4.1 |            |      |      |      |  Rp  | [Tr] |      | S4.1 |
+------+------+------+------+------+------+------+     Rx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      |      |      |  Tp  |  Rr  |      | [Re] |    <==>    |      |      |      |  Tp  |  Rr  |      |      |    <==>    |      |      |      |  Tp  |  Rr  |      | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |             |                                 |             |             |                                 |             |             |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+    
                                                                                          |
                                                                                          |  Tx
                                                                                          |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  |      |      | S5.1 |            |      |      |      |  Rp  | [Tr] |      | S5.1 |
+------+------+------+------+------+------+------+     Tx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+
|      |      |      |  Tp  |  Rr  | [Tf] |      |    <==>    |      |      |      |  Tp  |  Rr  | [Tf] |      |    <==>    |      |      |      |  Tp  |  Rr  | [Tf] | [Re] |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
|             |             |                                 |             |             |                                 |             |             |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+    
                                                                                          |
                                                                                          |  Rx
                                                                                          |
                                                              +------+------+------+------+------+------+------+          
                                                              |      |      |      |  Rp  | [Tr] | [Rf] | S6.1 |         
                                                              +------+------+------+------+------+------+------+  
                                                              |      |      |      |  Tp  |  Rr  |  Tf  | [Re] |  
                                                              +------+------+------+------+------+------+------+         
                                                              |             |             |                               
                                                              +------+------+------+------+                             
                                                                                          |                                                 continuous packet loss
                                                                                          |  shift                  +-------------------------------------------------------------+
                                                                                          |                         |                                                             |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+<---+       +------+------+------+------+------+------+------+    |
|      | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.2 |            |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] |      | S4.2 |    |
+------+------+------+------+------+------+------+     Rx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+    |
|      | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |    <==>    |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |    <==>    |      | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |    |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
|             |    PTof     |                                 |             |    PTof     |                                 |             |    PTof     |                         |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+                         |
                                                                                          |                                                                                       |
                                                                                          |  Tx                                                                                   |
                                                                                          |                                                                                       |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
|      | ERp  |  Tb  |  Rp  |      |      | S5.2 |            |      | ERp  |  Tb  |  Rp  |      |      | S5.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] |      | S5.2 |    |
+------+------+------+------+------+------+------+     Tx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+    |
|      | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |    <==>    |      | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |    <==>    |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |    |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
|             |    PTof     |                                 |             |    PTof     |                                 |             |    PTof     |                         |  
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+                         |
                                                                                          |                                                                                       |
                                                                                          |  Rx                                                                                   |
                                                                                          |                                                                                       |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              |      | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S6.2 |                                                                  |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              |      | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |                                                                  |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              |             |    PTof     |                                                                                       |
                                                              +------+------+------+------+                                                                                       |
                                                                                          |                                                 normal case                                               |
                                                                                          |  shift                  +-------------------------------------------------------------+
                                                                                          |                         |                                                             |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+<---+       +------+------+------+------+------+------+------+    |
| ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] |      | S4.3 |    |
+------+------+------+------+------+------+------+     Rx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+    |
| ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |    <==>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |    <==>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |    |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
|    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |                                 |    EPTof    |    PTof     |                         |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+                         |
                                                                                          |                                                                                       |
                                                                                          |  Tx                                                                                   |
                                                                                          |                                                                                       |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
| ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |            | ETb  | ERp  |  Tb  |  Rp  |      |      | S5.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] |      | S5.3 |    |
+------+------+------+------+------+------+------+     Tx     +------+------+------+------+------+------+------+   RX_NO    +------+------+------+------+------+------+------+    |
| ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |    <==>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  | [Tf] |      |    <==>    | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |    |
+------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+    |
|    EPTof    |    PTof     |                                 |             |    PTof     |                                 |    EPTof    |    PTof     |                         |
+------+------+------+------+                                 +------+------+------+------+                                 +------+------+------+------+                         |
                                                                                          |                                                                                       | 
                                                                                          |  Rx                                                                                   |
                                                                                          |                                                                                       |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              | ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S6.3 |                                                                  |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              | ERb  | ETp  |  Rb  |  Tp  |  Rr  |  Tf  | [Re] |                                                                  |
                                                              +------+------+------+------+------+------+------+                                                                  |
                                                              |    EPTof    |    PTof     |                                                                                       |
                                                              +------+------+------+------+                                                                                       |
                                                                                          |  shift                                                                                |
                                                                                          +---------------------------------------------------------------------------------------+ 
*/

static EventHandlerTable EVENT_HANDLER[RANGING_TABLE_STATE_COUNT][RANGING_TABLE_EVENT_COUNT] = {
    {RESERVED_STUB, RESERVED_STUB, RESERVED_STUB},
    {S1_TX, S1_RX, S1_RX_NO},
    {S2_TX, S2_RX, S2_RX_NO},
    {S3_TX, S3_RX, S3_RX_NO},
    {S4_TX, S4_RX, S4_RX_NO},
    {S5_TX, S5_RX, S5_RX_NO},
    {S6_TX, S6_RX, S6_RX_NO}
};

void RangingTableEventHandler(Ranging_Table_t *rangingTable, RANGING_TABLE_EVENT event) {
    ASSERT((rangingTable->rangingState < RANGING_TABLE_STATE_COUNT) && "Warning: Should not be called\n");
    ASSERT((event < RANGING_TABLE_EVENT_COUNT) && "Warning: Should not be called\n");
    EVENT_HANDLER[rangingTable->rangingState][event](rangingTable);
}
#endif


/* -------------------- Generate and Process -------------------- */
Time_t generateDSRMessage(Ranging_Message_t *rangingMessage) {
    Time_t taskDelay = M2T(RANGING_PERIOD);

    int8_t bodyUnitCount = 0;     
    rangingTableSet->localSeqNumber++;
    rangingMessage->header.filter = 0;

    /* generate bodyUnit */
    while(bodyUnitCount < MESSAGE_BODYUNIT_SIZE && bodyUnitCount < rangingTableSet->size) {
        index_t index = rangingTableSet->priorityQueue[bodyUnitCount];

        // update timestamp field first to avoid memory overwrite
        rangingMessage->bodyUnits[bodyUnitCount].timestamp = rangingTableSet->lastRxtimestamp[index].timestamp;
        rangingMessage->bodyUnits[bodyUnitCount].seqNumber = rangingTableSet->lastRxtimestamp[index].seqNumber;
        rangingMessage->bodyUnits[bodyUnitCount].address = rangingTableSet->rangingTable[index].neighborAddress;

        rangingMessage->header.filter |= 1 << (rangingTableSet->rangingTable[index].neighborAddress % 16);
        
        #ifdef STATE_MACHINE_ENABLE
            RangingTableEventHandler(&rangingTableSet->rangingTable[index], RANGING_EVENT_TX);
        #endif

        bodyUnitCount++;
    }

    // update priority queue
    updatePriorityQueue(rangingTableSet, bodyUnitCount);

    // fill in empty info
    while(bodyUnitCount < MESSAGE_BODYUNIT_SIZE) {
        rangingMessage->bodyUnits[bodyUnitCount].timestamp.full = NULL_TIMESTAMP;
        rangingMessage->bodyUnits[bodyUnitCount].seqNumber = NULL_SEQ;
        rangingMessage->bodyUnits[bodyUnitCount].address = NULL_ADDR;
        bodyUnitCount++;
    }

    /* generate header */
    rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
    rangingMessage->header.msgSequence = rangingTableSet->localSeqNumber;
    index_t index_Tx = rangingTableSet->sendList.topIndex;
    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        if(rangingTableSet->sendList.Txtimestamps[index_Tx].seqNumber != NULL_SEQ) {
            rangingMessage->header.Txtimestamps[i] = rangingTableSet->sendList.Txtimestamps[index_Tx];
            index_Tx = (index_Tx - 1 + SEND_LIST_SIZE) % SEND_LIST_SIZE;
        }
        else {
            rangingMessage->header.Txtimestamps[i] = nullTimestampTuple;
        }
    }
    #ifdef COORDINATE_SEND_ENABLE
        rangingMessage->header.TxCoordinate = rangingTableSet->sendList.TxCoordinate;
    #endif
    rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Ranging_Message_Body_Unit_t) * bodyUnitCount;

    // clear expired rangingTable
    if(rangingTableSet->localSeqNumber % CHECK_PERIOD == 0) {
        checkExpiration(rangingTableSet);
    }

    // DEBUG_PRINT("[generate]\n");
    // printRangingMessage(rangingMessage);

    return taskDelay;
}

void processDSRMessage(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo) {
    Ranging_Message_t *rangingMessage = &rangingMessageWithAdditionalInfo->rangingMessage;

    // DEBUG_PRINT("[process]\n");
    // printRangingMessage(rangingMessage);

    uint16_t neighborAddress = rangingMessage->header.srcAddress;

    if(neighborAddress > NEIGHBOR_ADDRESS_MAX) {
        DEBUG_PRINT("[processDSRMessage]: neighborAddress > NEIGHBOR_ADDRESS_MAX\n");
        return;
    }

    index_t neighborIndex = findRangingTable(rangingTableSet, neighborAddress);

    // handle new neighbor
    if(neighborIndex == NULL_INDEX) {
        neighborIndex = registerRangingTable(rangingTableSet, neighborAddress);
        if(neighborIndex == NULL_INDEX) {
            DEBUG_PRINT("Warning: Failed to register new neighbor.\n");
            return;
        }
    }

    Ranging_Table_t *rangingTable = &rangingTableSet->rangingTable[neighborIndex];

    Timestamp_Tuple_t Re = nullTimestampTuple;
    Re.timestamp = rangingMessageWithAdditionalInfo->timestamp;
    Re.seqNumber = rangingMessage->header.msgSequence;
    rangingTableSet->lastRxtimestamp[neighborIndex] = Re;
    
    #ifdef COORDINATE_SEND_ENABLE
        Coordinate_Tuple_t TxCoordinate = rangingMessage->header.TxCoordinate;
        Coordinate_Tuple_t RxCoordinate = rangingMessageWithAdditionalInfo->RxCoordinate;
        float Dx = (RxCoordinate.x - TxCoordinate.x);
        float Dy = (RxCoordinate.y - TxCoordinate.y);
        float Dz = (RxCoordinate.z - TxCoordinate.z);
        float distance = sqrtf(Dx * Dx + Dy * Dy + Dz * Dz);
        distanceReal[rangingTable->neighborAddress] = distance;
    #endif

    /* process bodyUnit */
    Timestamp_Tuple_t Rf = nullTimestampTuple;
    if (rangingMessage->header.filter & (1 << (uwbGetAddress() % 16))) {
        uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Ranging_Message_Body_Unit_t);
        for(int i = 0; i < bodyUnitCount; i++) {
            if(rangingMessage->bodyUnits[i].address == (uint8_t)uwbGetAddress()) {
                Rf.timestamp = rangingMessage->bodyUnits[i].timestamp;
                Rf.seqNumber = rangingMessage->bodyUnits[i].seqNumber;
                break;
            }
        }
    }

    Timestamp_Tuple_t Tf = nullTimestampTuple;
    index_t index_Tf = NULL_INDEX;
    index_Tf = findSendList(&rangingTableSet->sendList, Rf.seqNumber);
    if(index_Tf != NULL_INDEX) {
        Tf = rangingTableSet->sendList.Txtimestamps[index_Tf];
    }

    fillRangingTable(rangingTable, Tf, Rf, Re);

    /* process header */
    index_t index_Rr = rangingTable->TrRrBuffer.topIndex;
    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        // find corresponding Tr according to Rr to get a valid Tr-Rr pair, this approach may help when experiencing continuous packet loss
        if(rangingMessage->header.Txtimestamps[i].timestamp.full != NULL_TIMESTAMP 
           && rangingTable->TrRrBuffer.candidates[index_Rr].Rr.timestamp.full != NULL_TIMESTAMP 
           && rangingMessage->header.Txtimestamps[i].seqNumber == rangingTable->TrRrBuffer.candidates[index_Rr].Rr.seqNumber) {
            // Rr is updated in updateRangingTableRr_Buffer
            updateRangingTableTr_Buffer(&rangingTable->TrRrBuffer, rangingMessage->header.Txtimestamps[i]);
            break;
           }
    }

    if(Tf.timestamp.full != NULL_TIMESTAMP && Rf.timestamp.full != NULL_TIMESTAMP && Rf.seqNumber != rangingTable->Rp.seqNumber) {
        // RX
        RangingTableEventHandler(rangingTable, RANGING_EVENT_RX);
    }
    else {
        // RX_NO
        RangingTableEventHandler(rangingTable, RANGING_EVENT_RX_NO);
    }

    // print result
    if(distanceCalculate[rangingTable->neighborAddress] != NULL_DIS) {
        #ifdef COMPENSATE_ENABLE
        /* For swarm ranging, the calculated distance value has a certain lag.
                ···Tb      <--Pb-->      Rp   <--Db-->   Tr      <--Rb-->      Rf

                ···   Rb   <--Pa-->   Tp      <--Ra-->      Rr   <--Da-->   Tf                    Re
            Continuous scenes:
                ToF_p = ToF_r - ΔT1, ToF_f = ToF_r + ΔT2
                The ranging process from p to r to f to e can be regarded as uniform motion.
                ∵ ΔT1 and ΔT2 are close to zero, the ToF at different stages can be approximated as:
                    ToF_p = ToF_r - ΔT
                    ToF_f = ToF_r + ΔT

                        Ra * Rb - Da * Db
                ToF = —————————————————————
                        Ra + Rb + Da + Db
                        (ToF_p + ToF_r + Db) * (ToF_r + ToF_f + Da) - Da * Db
                    = —————————————————————————————————————————————————————————
                                        2(Da + Db) + 4 * ToF_r
                        (2 * ToF_r - ΔT + Db) * (2 * ToF_r + ΔT + Da) - Da * Db
                    = ———————————————————————————————————————————————————————————
                                        2(Da + Db) + 4 * ToF_r
                        2 * (Da + Db) * ToF_r + 4 * ToF_r^2 + (Db - Da) * ΔT - ΔT^2
                    = ———————————————————————————————————————————————————————————————
                                            2(Da + Db) + 4 * ToF_r
                                    (Db - Da) - ΔT
                    = ToF_r + —————————————————————————— * ΔT
                                2(Da + Db) + 4 * ToF_r
                As ΔT approaches zero, Tof ≈ Tof_r.

                To make the ranging results closer to the true values, a compensation mechanism is adopted:
                If Rr.seqNumber + 1 == Re.seqNumber (continuous), then:
                    De = Dr + ΔD = ToF_r * velocity + ΔD ≈ ToF * velocity + (distanceCalculate - lastDistanceCalculate)
            
            Discontinuous scenes:
                If packet loss occurs, use linear extrapolation:
                    seqGap = Re.seqNumber - Rr.seqNumber
                    ΔD = deltaDistanceUnit * seqGap
                    De = Dr + ΔD = ToF_r * velocity + ΔD ≈ ToF * velocity + deltaDistanceUnit * seqGap

            therefore: 
            seqGap = Re.seqNumber - Rr.seqNumber
            distanceCompensate = deltaDistanceUnit * seqGap
            deltaDistanceUnit = (distanceCalculate - lastDistanceCalculate) / seqGap
            distance = distanceCalculate + distanceCompensate

            In scenarios with high uncertainty, it is recommended to select a coefficient COMPENSATE_RATE <1 for partial compensation
            instead of full compensation. Additionally, when seqGap exceeds a preset threshold(SEQGAP_THRESHOLD), linear extrapolation 
            may introduce significant errors in non-uniform motion scenarios. In such cases, the compensation mechanism should be disabled,
            and the system should revert to raw measurements.

            Compensation Algorithm:
            1. Initialization:
                if(lastSeqNumber == NULL_SEQ || lastDistanceCalculate == NULL_DIS) {
                    lastDistanceCalculate = distanceCalculate;
                    lastSeqNumber = rangingMessage->header.msgSequence;
                    distance = distanceCalculate;
                }
            2. First compensation:
                else if(deltaDistanceUnit == NULL_DIS) {
                    seqGap = rangingMessage->header.msgSequence - lastSeqNumber;
                    deltaDistanceUnit = (distanceCalculate - lastDistanceCalculate) / seqGap;
                    lastDistanceCalculate = distanceCalculate;
                    lastSeqNumber = rangingMessage->header.msgSequence;
                    distance = distanceCalculate;
                }
            3. Normal compensation:
                else {
                    seqGap = rangingMessage->header.msgSequence - lastSeqNumber;
                    distanceCompensate = deltaDistanceUnit * seqGap;
                    deltaDistanceUnit = (distanceCalculate - lastDistanceCalculate) / seqGap;
                    lastDistanceCalculate = distanceCalculate;
                    lastSeqNumber = rangingMessage->header.msgSequence;
                    if(seqGap < SEQGAP_THRESHOLD) {
                        distance = distanceCalculate + distanceCompensate * COMPENSATE_RATE;
                    }
                    else {
                        // If packet loss is severe, disable compensation
                        distance = distanceCalculate;
                    }
                }
            return distance;
        */
            if(lastSeqNumber[rangingTable->neighborAddress] == NULL_SEQ || lastDistanceCalculate[rangingTable->neighborAddress] == NULL_DIS) {
                lastDistanceCalculate[rangingTable->neighborAddress] = distanceCalculate[rangingTable->neighborAddress];
                lastSeqNumber[rangingTable->neighborAddress] = rangingMessage->header.msgSequence;
                DEBUG_PRINT("[local_%u<-neighbor_%u]: ModifiedD = %f", MY_UWB_ADDRESS, rangingTable->neighborAddress, (double)distanceCalculate[rangingTable->neighborAddress]);
            }
            else if(deltaDistanceUnit[rangingTable->neighborAddress] == NULL_DIS) {
                uint16_t seqGap = rangingMessage->header.msgSequence - lastSeqNumber[rangingTable->neighborAddress];
                deltaDistanceUnit[rangingTable->neighborAddress] = (distanceCalculate[rangingTable->neighborAddress] - lastDistanceCalculate[rangingTable->neighborAddress]) / seqGap;
                lastDistanceCalculate[rangingTable->neighborAddress] = distanceCalculate[rangingTable->neighborAddress];
                lastSeqNumber[rangingTable->neighborAddress] = rangingMessage->header.msgSequence;
                DEBUG_PRINT("[local_%u<-neighbor_%u]: ModifiedD = %f", MY_UWB_ADDRESS, rangingTable->neighborAddress, (double)distanceCalculate[rangingTable->neighborAddress]);
            }
            else {
                uint16_t seqGap = rangingMessage->header.msgSequence - lastSeqNumber[rangingTable->neighborAddress];
                float distanceCompensate = deltaDistanceUnit[rangingTable->neighborAddress] * seqGap;
                deltaDistanceUnit[rangingTable->neighborAddress] = (distanceCalculate[rangingTable->neighborAddress] - lastDistanceCalculate[rangingTable->neighborAddress]) / seqGap;
                lastDistanceCalculate[rangingTable->neighborAddress] = distanceCalculate[rangingTable->neighborAddress];
                lastSeqNumber[rangingTable->neighborAddress] = rangingMessage->header.msgSequence;
                if(seqGap < SEQGAP_THRESHOLD) {
                    float distance = distanceCalculate[rangingTable->neighborAddress] + distanceCompensate * COMPENSATE_RATE;                    
                    DEBUG_PRINT("[local_%u<-neighbor_%u]: ModifiedD = %f", MY_UWB_ADDRESS, rangingTable->neighborAddress, (double)distance);
                } 
                else {
                    // If packet loss is severe, disable compensation
                    DEBUG_PRINT("[local_%u<-neighbor_%u]: ModifiedD = %f", MY_UWB_ADDRESS, rangingTable->neighborAddress, (double)distanceCalculate[rangingTable->neighborAddress]);
                }
            }
        #else
            DEBUG_PRINT("[local_%u<-neighbor_%u]: ModifiedD = %f", MY_UWB_ADDRESS, rangingTable->neighborAddress, (double)distanceCalculate[rangingTable->neighborAddress]);
        #endif

        #ifdef COORDINATE_SEND_ENABLE
            DEBUG_PRINT(", TrueD = %f", distanceReal[rangingTable->neighborAddress]);
        #endif
        DEBUG_PRINT(", time = %llu\n", Re.timestamp.full % UWB_MAX_TIMESTAMP);
    }
    else {
        DEBUG_PRINT("[local_%u]: calculation failed\n", MY_UWB_ADDRESS);
    }

    rangingTableSet->rangingTable[neighborIndex].expirationSign = false;

    // printRangingTableSet(rangingTableSet);
}


#ifndef SIMULATION_ENABLE
/* -------------------- Call back -------------------- */
static void uwbRangingTxTask(void *parameters) {
    systemWaitStart();

    UWB_Packet_t txPacketCache;
    txPacketCache.header.srcAddress = uwbGetAddress();
    txPacketCache.header.destAddress = UWB_DEST_ANY;
    txPacketCache.header.type = UWB_RANGING_MESSAGE;
    txPacketCache.header.length = 0;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t*)&txPacketCache.payload;

    while (true) {
        xSemaphoreTake(rangingTableSet->mutex, portMAX_DELAY);

        // DEBUG_PRINT("[uwbRangingTxTask]: Acquired mutex, generating ranging message\n");
        Time_t taskDelay = RANGING_PERIOD;

        generateDSRMessage(rangingMessage);
        
        txPacketCache.header.seqNumber++;
        txPacketCache.header.length = sizeof(UWB_Packet_Header_t) + rangingMessage->header.msgLength;
        
        uwbSendPacketBlock(&txPacketCache);

        xSemaphoreGive(rangingTableSet->mutex);

        vTaskDelay(taskDelay);
    }
}

static void uwbRangingRxTask(void *parameters) {
    systemWaitStart();

    Ranging_Message_With_Additional_Info_t rxPacketCache;

    while (true) {
        vTaskDelay(M2T(1));
        if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
            xSemaphoreTake(rangingTableSet->mutex, portMAX_DELAY);

            processDSRMessage(&rxPacketCache);
            
            xSemaphoreGive(rangingTableSet->mutex);
        }
    }
}

void rangingTxCallback(void *parameters) {
    UWB_Packet_t *packet = (UWB_Packet_t*)parameters;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t*)packet->payload;

    dwTime_t txTime;
    dwt_readtxtimestamp((uint8_t*)&txTime.raw);

    Timestamp_Tuple_t timestamp = {.timestamp = txTime, .seqNumber = rangingMessage->header.msgSequence};
    updateSendList(&rangingTableSet->sendList, timestamp);
}

void rangingRxCallback(void *parameters) {
    #ifdef PACKET_LOSS_ENABLE
        int randnum = rand() % 100;
        if(randnum < (int)(PACKET_LOSS_RATE * 100)) {
            return;
        }
    #endif

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    UWB_Packet_t *packet = (UWB_Packet_t*)parameters;

    dwTime_t rxTime;
    dwt_readrxtimestamp((uint8_t*)&rxTime.raw);

    Ranging_Message_With_Additional_Info_t rxMessageWithTimestamp;
    rxMessageWithTimestamp.timestamp = rxTime;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t*)packet->payload;
    rxMessageWithTimestamp.rangingMessage = *rangingMessage;

    xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
}

void rangingInit() {
    MY_UWB_ADDRESS = uwbGetAddress();

    srand(MY_UWB_ADDRESS);

    rangingTableSetInit(&rangingTableSet);

    rxQueue = xQueueCreate(RANGING_RX_QUEUE_SIZE, RANGING_RX_QUEUE_ITEM_SIZE);
    
    listener.type = UWB_RANGING_MESSAGE;
    listener.rxQueue = NULL;
    listener.rxCb = rangingRxCallback;
    listener.txCb = rangingTxCallback;
    uwbRegisterListener(&listener);

    xTaskCreate(uwbRangingTxTask, ADHOC_UWB_RANGING_TX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
                ADHOC_UWB_TASK_PRI, &uwbRangingTxTaskHandle);
    xTaskCreate(uwbRangingRxTask, ADHOC_UWB_RANGING_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL,
                ADHOC_UWB_TASK_PRI, &uwbRangingRxTaskHandle);
}


LOG_GROUP_START(Ranging)
LOG_ADD(LOG_INT16, distTo00, distanceCalculate + 0)
LOG_ADD(LOG_INT16, distTo01, distanceCalculate + 1)
LOG_ADD(LOG_INT16, distTo02, distanceCalculate + 2)
LOG_ADD(LOG_INT16, distTo03, distanceCalculate + 3)
LOG_ADD(LOG_INT16, distTo04, distanceCalculate + 4)
LOG_ADD(LOG_INT16, distTo05, distanceCalculate + 5)
LOG_ADD(LOG_INT16, distTo06, distanceCalculate + 6)
LOG_ADD(LOG_INT16, distTo07, distanceCalculate + 7)
LOG_ADD(LOG_INT16, distTo08, distanceCalculate + 8)
LOG_ADD(LOG_INT16, distTo09, distanceCalculate + 9)
LOG_ADD(LOG_INT16, distTo10, distanceCalculate + 10)
LOG_ADD(LOG_INT16, distTo11, distanceCalculate + 11)
LOG_ADD(LOG_INT16, distTo12, distanceCalculate + 12)
LOG_ADD(LOG_INT16, distTo13, distanceCalculate + 13)
LOG_ADD(LOG_INT16, distTo14, distanceCalculate + 14)
LOG_ADD(LOG_INT16, distTo15, distanceCalculate + 15)
LOG_ADD(LOG_INT16, distTo16, distanceCalculate + 16)
LOG_ADD(LOG_INT16, distTo17, distanceCalculate + 17)
LOG_ADD(LOG_INT16, distTo18, distanceCalculate + 18)
LOG_ADD(LOG_INT16, distTo19, distanceCalculate + 19)
LOG_ADD(LOG_INT16, distTo20, distanceCalculate + 20)
LOG_ADD(LOG_INT16, distTo21, distanceCalculate + 21)
LOG_ADD(LOG_INT16, distTo22, distanceCalculate + 22)
LOG_ADD(LOG_INT16, distTo23, distanceCalculate + 23)
LOG_ADD(LOG_INT16, distTo24, distanceCalculate + 24)
LOG_ADD(LOG_INT16, distTo25, distanceCalculate + 25)
LOG_ADD(LOG_INT16, distTo26, distanceCalculate + 26)
LOG_ADD(LOG_INT16, distTo27, distanceCalculate + 27)
LOG_ADD(LOG_INT16, distTo28, distanceCalculate + 28)
LOG_ADD(LOG_INT16, distTo29, distanceCalculate + 29)
LOG_GROUP_STOP(Ranging)
#endif