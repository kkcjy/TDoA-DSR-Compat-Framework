#include "adhocuwb_dynamic_swarm_ranging.h"

#ifndef SIMULATION_COMPILE
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "adhocuwb_init.h"
#include "adhocuwb_platform.h"
#endif

#ifdef  CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
#include "log.h"
#endif

#ifdef CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7
#include "uwb_send_print.h"
#endif

/*
OPTIMAL_RANGING_SCHEDULE_ENABLE     -> Enable Midpoint_Adjustment to ensure equal message-sending intervals among drones within the swarm
COMPENSATE_ENABLE                   -> Perform compensation after each received packet calculation
getCurDistance                      -> Obtain the real-time distance at any moment based on compensation
*/

static uint16_t MY_UWB_ADDRESS;
Ranging_Table_Set_t *rangingTableSet;

#ifdef SIMULATION_COMPILE
extern const char* localAddress;
#else
static QueueHandle_t rxQueue;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;
#endif

static int16_t d_curr[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_DIS};
static int16_t d_prev[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_DIS};

#ifdef COMPENSATE_ENABLE
// used for compensation
static float d_temp[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_DIS};
static int16_t d_his_avg[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_DIS};
static int16_t last_Seq[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_SEQ};
static int16_t last_SeqGap[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_SEQ};
static bool turning_state[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = false};
static float compensation_factor = NULL_RATE;
// used for getting current distance
static uint64_t ONCE_Rr[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_TIMESTAMP};
static uint64_t ONCE_Tf[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_TIMESTAMP};
static uint64_t ONCE_Re[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = NULL_TIMESTAMP};
static bool compensated[NEIGHBOR_ADDRESS_MAX + 1] = {[0 ... NEIGHBOR_ADDRESS_MAX] = false};
#endif

#ifdef TDOA_COMPAT_ENABLE
/* Attention (cm)
1. The first row and first column are left unused because anchor addresses cannot be zero.
2. The matrix is indexed directly by anchor addresses to obtain distances, so it is recommended to assign smaller address values to anchors, or enlarge anchor_distance_matrix if needed.
3. Each distance entry must be filled twice to maintain symmetry (d_ij = d_ji).
*/
float anchor_distance_matrix[ANCHOR_SIZE + 1][ANCHOR_SIZE + 1] = {
    {NULL_DIS, NULL_DIS    , NULL_DIS},
    {NULL_DIS, 10.0        , NULL_DIS},
    {NULL_DIS, NULL_DIS    , 10.0    }};

#if defined(ANCHOR_MODE_ENABLE)
Anchor_Table_Set_t *anchorTableSet;
#elif defined(TAG_MODE_ENABLE)
Tag_Table_Set_t *tagTableSet;
#endif
static atomic_int ranging_cluster;     // indicates whether the TAG is in DSR-cluster or TDoA-cluster
dwTime_t lastTDoAReceptionTime;
#endif

#ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
static int8_t rangingPeriodFineTune = 0;


void Midpoint_Adjustment(dwTime_t curTxDwtime, ReceiveList_t *receiveList) {

    uint32_t ErrorAllowedinMs = 2;

    uint64_t minRxLast = UWB_MAX_TIMESTAMP;
    uint64_t minRxNext = UWB_MAX_TIMESTAMP;
    
    if (rangingTableSet->size ==0)
    	return;

    uint64_t TicksPerPeriod = (uint64_t)RANGING_PERIOD / (DWT_TIME_UNITS * 1000);
    uint64_t minRxExpected = TicksPerPeriod / rangingTableSet->size;
    uint64_t minRxError = (uint64_t)ErrorAllowedinMs / (DWT_TIME_UNITS * 1000);

    for (int i = receiveList->topIndex; 
                (i - 1 + RECEIVE_LIST_SIZE) % RECEIVE_LIST_SIZE != receiveList->topIndex; 
                i = (i - 1 + RECEIVE_LIST_SIZE) % RECEIVE_LIST_SIZE) {

        if(receiveList->Rxtimestamps[i].full == NULL_TIMESTAMP) {
            break;
        }
        
        /*
        +-------+------+-------+-------+-------+------+-------+-------+
        |  RX0  |  TX  |  RX1  |  RX2  |  RX3  |  TX  |  RX4  |  RX5  |
        +-------+------+-------+-------+-------+------+-------+-------+
                                           ^      ^       ^
                                         last    now    next
        */
        uint64_t curTxtimestamp = curTxDwtime.full % UWB_MAX_TIMESTAMP;
        uint64_t RxTimeStamp = receiveList->Rxtimestamps[i].full % UWB_MAX_TIMESTAMP;
        uint64_t diffTimestampLast = (curTxtimestamp - RxTimeStamp + UWB_MAX_TIMESTAMP) % (UWB_MAX_TIMESTAMP);
        uint64_t diffTimestampNext = (TicksPerPeriod + RxTimeStamp - curTxtimestamp + UWB_MAX_TIMESTAMP) % (UWB_MAX_TIMESTAMP);

        if(diffTimestampLast > TicksPerPeriod) {
            break;
        }

        minRxLast = diffTimestampLast < minRxLast ? diffTimestampLast : minRxLast;
        minRxNext = diffTimestampNext < minRxNext ? diffTimestampNext : minRxNext;
    }
    if(abs((int64_t)minRxExpected - (int64_t)minRxLast) <= minRxError)  {
        rangingPeriodFineTune = 0;
    }
    else if (minRxLast < minRxNext) {
        rangingPeriodFineTune = +1;
    }
    else if (minRxLast > minRxNext) {
        rangingPeriodFineTune = -1;
    }
    else if(minRxLast == UWB_MAX_TIMESTAMP && minRxNext == UWB_MAX_TIMESTAMP && rangingTableSet->size > 0) {
        rangingPeriodFineTune = -1;
    }
}
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

// update the send list with new Tx and Rx timestamps without coordinate
void updateSendList(SendList_t *sendList, Timestamp_Tuple_t timestampTuple) {
    sendList->topIndex = (sendList->topIndex + 1) % SEND_LIST_SIZE;
    sendList->Txtimestamps[sendList->topIndex] = timestampTuple;
}

#ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
void updateReceiveList(ReceiveList_t *receiveList, dwTime_t timestamp) {
    receiveList->topIndex = (receiveList->topIndex + 1) % RECEIVE_LIST_SIZE;
    receiveList->Rxtimestamps[receiveList->topIndex] = timestamp;
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
    rangingTable->rangingState = RANGING_STATE_S1;
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

            // DEBUG_PRINT("Registered new ranging table entry: Address = %u\n", address);
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

    #ifdef COMPENSATE_ENABLE
        d_prev[address] = NULL_DIS;
        d_his_avg[address] = NULL_DIS;
        d_temp[address] = NULL_DIS;
        last_Seq[address] = NULL_SEQ;
        last_SeqGap[address] = NULL_SEQ;
        turning_state[address] = false;
        compensation_factor = NULL_RATE;
        ONCE_Rr[address] = NULL_TIMESTAMP;
        ONCE_Tf[address] = NULL_TIMESTAMP;
        ONCE_Re[address] = NULL_TIMESTAMP;
        compensated[address] = false;
    #endif

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
            // DEBUG_PRINT("[correct PToF]: correct PToF = %f\n", (float)correctPToF);
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
    #ifdef SIMULATION_COMPILE
        MY_UWB_ADDRESS = (uint16_t)strtoul(localAddress, NULL, 10);
    #endif

    rangingTableSet = (Ranging_Table_Set_t*)malloc(sizeof(Ranging_Table_Set_t));
    rangingTableSet->size = 0;
    rangingTableSet->localSeqNumber = NULL_SEQ;
    rangingTableSet->sendList.topIndex = NULL_INDEX;
    for (int i = 0; i < SEND_LIST_SIZE; i++) {
        rangingTableSet->sendList.Txtimestamps[i] = nullTimestampTuple;
    }

    #ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
        rangingTableSet->receiveList.topIndex = NULL_INDEX;
        for (int i = 0; i < RECEIVE_LIST_SIZE; i++) {
            rangingTableSet->receiveList.Rxtimestamps[i].full = NULL_TIMESTAMP;
        }
    #endif

    for (int i = 0; i < RANGING_TABLE_SIZE; i++) {
        rangingTableInit(&rangingTableSet->rangingTable[i]);
        rangingTableSet->lastRxtimestamp[i] = nullTimestampTuple;
        rangingTableSet->priorityQueue[i] = NULL_INDEX;
    }
    rangingTableSet->mutex = xSemaphoreCreateMutex();
}

// check expirationSign of rangingTables and deregister rangingTable expired
void checkExpirationCallback(Ranging_Table_Set_t *rangingTableSet) {
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
    }
    else {
        #ifdef CLASSIC_SUPPORT_ENABLE
            // DEBUG_PRINT("[rangingAlgorithm]: not meet the convergence condition, use classic algorithm.\n");
            ToF23 = classicCalculatePToF(T1, R1, T2, R2, T3, R3);
        #else
            DEBUG_PRINT("[rangingAlgorithm]: not meet the convergence condition.\n");
        #endif
    }

    // abnormal result
    float D = (ToF23 * VELOCITY) / 2;
    if(D < LOWER_BOUND_DISTANCE || D > UPPER_BOUND_DISTANCE) {
        // DEBUG_PRINT("[rangingAlgorithm]: dist = %f out of range.\n", D);
        return NULL_TOF;
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
        // DEBUG_PRINT("Warning: Data calculation is not complete\n");
        return NULL_TOF;
    }

    int64_t Ra = (Rr.timestamp.full - Tp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Da = (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Rb = (Rf.timestamp.full - Tr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
    int64_t Db = (Tr.timestamp.full - Rp.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;

    int64_t diffA = Ra - Da;
    int64_t diffB = Rb - Db;

    curPToF = (diffA * Rb + diffA * Db + diffB * Ra + diffB * Da) / (float)(Ra + Db + Rb + Da);

    float D = (curPToF * VELOCITY) / 2;
    if(D < LOWER_BOUND_DISTANCE || D > UPPER_BOUND_DISTANCE) {
        // DEBUG_PRINT("[classicCalculatePToF]: dist = %f out of range.\n", D);
        return NULL_TOF;
    }

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
    Timestamp_Tuple_t Re = rangingTable->Re;

    float tmpPToF = NULL_TOF;
    float curPToF = NULL_TOF;
    
    // check completeness
    if(Tr.seqNumber == NULL_SEQ || Rr.seqNumber == NULL_SEQ || Tf.seqNumber == NULL_SEQ || Rf.seqNumber == NULL_SEQ) {
        #ifdef COMPENSATE_ENABLE
            compensation_factor = NULL_RATE;
        #endif

        /* type1
            +------+------+------+------+           +------+------+------+------+
            |  Tb  |  Rp  |  Tr  |      |           | ERp  |  Tb  |  Rp  |  Tr  |
            +------+------+------+------+   ===>    +------+------+------+------+
            |  Rb  |  Tp  |  Rr  |      |           | ETp  |  Rb  |  Tp  |  Rr  |
            +------+------+------+------+           +------+------+------+------+
        */ 
        if(Tr.seqNumber != NULL_SEQ && Rr.seqNumber != NULL_SEQ) {
            // DEBUG_PRINT("[calculatePToF]: Data calculation is not complete, pair of Tf-Rf is losed\n");
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
                return rangingTable->PToF;
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
            // DEBUG_PRINT("[calculatePToF]: Data calculation is not complete, pair of Tr-Rr is losed\n");
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
                return rangingTable->PToF;
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
            // DEBUG_PRINT("[calculatePToF]: Data calculation is not complete, pair of Tf-Rf and Tr-Rr are losed\n");
            return rangingTable->PToF;
        }
    }

    // normal
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
            #ifdef COMPENSATE_ENABLE
                ONCE_Rr[rangingTable->neighborAddress] = Rr.timestamp.full;
                ONCE_Tf[rangingTable->neighborAddress] = Tf.timestamp.full;
                ONCE_Re[rangingTable->neighborAddress] = Re.timestamp.full;
                uint64_t diffReRr = (Re.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
                uint64_t diffTfRr = (Tf.timestamp.full - Rr.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
                if(Rr.timestamp.full > Tf.timestamp.full || Tf.timestamp.full > Re.timestamp.full) {
                    compensation_factor = NULL_RATE;
                }
                else {
                    compensation_factor = (float)(diffReRr - diffTfRr / 2.0) / (float)diffReRr;
                }
            #endif
        }
        else {
            #ifdef COMPENSATE_ENABLE
                compensation_factor = NULL_RATE;
            #endif
            return rangingTable->PToF;
        }
    }
    return curPToF;
}

void continuousPacketLossHandler(Ranging_Table_t *rangingTable, Ranging_Table_Tr_Rr_Candidate_t candidate) {
    // DEBUG_PRINT("[continuousPacketLossHandler]: A long-term continuous packet loss event occurred.\n");
    
    float curPToF_avg = (classicCalculatePToF(rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr) 
                       + classicCalculatePToF(rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr, rangingTable->Tf, rangingTable->Rf)) / 2;
    d_curr[rangingTable->neighborAddress] = (curPToF_avg * VELOCITY) / 2;
    
    updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);

    shiftRangingTable(rangingTable, candidate.Tr, candidate.Rr, curPToF_avg);

    // goto S4.2
    rangingTable->ETb = nullTimestampTuple;
    rangingTable->ERb = nullTimestampTuple;
    rangingTable->EPToF = NULL_TOF;

    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    //DEBUG_PRINT("[S5_RX{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, RANGING_SUBSTATE_S3, curState, RANGING_SUBSTATE_S2);
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
    // DEBUG_PRINT("size: %u\n", rangingTableSet->size);
    DEBUG_PRINT("localSeqNumber: %lu\n", rangingTableSet->localSeqNumber);

    // printPriorityQueue(rangingTableSet);

    printSendList(&rangingTableSet->sendList);

    DEBUG_PRINT("\n[lastRxtimestamp]\n");
    for(int i = 0; i < rangingTableSet->size; i++) {
        DEBUG_PRINT("neighborAddress: %u, seqNumber: %u, timestamp: %llu\n", rangingTableSet->rangingTable[i].neighborAddress, rangingTableSet->lastRxtimestamp[i].seqNumber, rangingTableSet->lastRxtimestamp[i].timestamp.full % UWB_MAX_TIMESTAMP);
    }

    // DEBUG_PRINT("\n[rangingTable]\n");
    // for(int i = 0; i < rangingTableSet->size; i++) {
    //     printRangingTable(&rangingTableSet->rangingTable[rangingTableSet->priorityQueue[i]]);
    // }
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

    //DEBUG_PRINT("[S1_TX]: S%d -> S%d\n", prevState, curState);
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

    //DEBUG_PRINT("[S1_RX{local_%u, neighbor_%u}]: S%d -> S%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curState);
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

    //DEBUG_PRINT("[S1_RX_NO{local_%u, neighbor_%u}]: S%d -> S%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curState);
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

    //DEBUG_PRINT("[S2_TX]: S%d -> S%d\n", prevState, curState);
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

    //DEBUG_PRINT("[S2_RX{local_%u, neighbor_%u}]: S%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curState, RANGING_SUBSTATE_S1);
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

    //DEBUG_PRINT("[S2_RX_NO{local_%u, neighbor_%u}]: S%d -> S%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curState);
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
    //DEBUG_PRINT("[S4_TX]: S%d.%d -> S%d.%d\n", prevState, curSubstate, curState, curSubstate);
}

static void S4_RX(Ranging_Table_t *rangingTable) {
    // Calculate ToF, but not shift rangingTable
    RANGING_TABLE_STATE prevState = rangingTable->rangingState;
    int curSubstate = getCurrentSubstate(rangingTable);

    switch (curSubstate) {
        case RANGING_SUBSTATE_S1: {
            /* initialize               ===>    update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+
                |      |      |      |  Rp  |      |      | S4.1 |            |      |      |      |  Rp  | [Tr] | [Rf] | S4.1 |
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
                |      | ERp  |  Tb  |  Rp  |      |      | S4.2 |            |      | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.2 |
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+
                |      | ETp  |  Rb  |  Tp  |  Rr  |      |      |            |      | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+
                |             |    PToF     |                                 |             |    PToF     |                                
                +------+------+------+------+                                 +------+------+------+------+ 
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
            float curPToF = classicCalculatePToF(rangingTable->Tb, rangingTable->Rb, rangingTable->Tp, rangingTable->Rp, candidate.Tr, candidate.Rr);
            d_curr[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? ((rangingTable->PToF * VELOCITY) / 2) : ((curPToF * VELOCITY) / 2);

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        case RANGING_SUBSTATE_S3: {
            /* dynamic swarm ranging    ===>    calculate and update
                +------+------+------+------+------+------+------+            +------+------+------+------+------+------+------+      
                | ETb  | ERp  |  Tb  |  Rp  |      |      | S4.3 |            | ETb  | ERp  |  Tb  |  Rp  | [Tr] | [Rf] | S4.3 |          
                +------+------+------+------+------+------+------+    ===>    +------+------+------+------+------+------+------+         
                | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      |      |            | ERb  | ETp  |  Rb  |  Tp  |  Rr  |      | [Re] |   
                +------+------+------+------+------+------+------+    <===    +------+------+------+------+------+------+------+          
                |    EPToF    |    PToF     |                                 |    EPToF    |    PToF     |                                
                +------+------+------+------+                                 +------+------+------+------+ 
            */
            Ranging_Table_Tr_Rr_Candidate_t candidate = rangingTableTr_Rr_BufferGetCandidate(&rangingTable->TrRrBuffer, rangingTable->Rp, nullTimestampTuple);
            float curPToF = calculatePToF(rangingTable, candidate);
            d_curr[rangingTable->neighborAddress] = (curPToF * VELOCITY) / 2;

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
    
    //DEBUG_PRINT("[S4_RX{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curSubstate, curState, curSubstate);
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
            d_curr[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? ((rangingTable->PToF * VELOCITY) / 2) : ((curPToF * VELOCITY) / 2);

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
            d_curr[rangingTable->neighborAddress] = (curPToF * VELOCITY) / 2;

            updateRangingTableRr_Buffer(&rangingTable->TrRrBuffer, rangingTable->Re);
            break;
        }

        default: {
            ASSERT(0 && "[S4_RX]: Should not be called\n");
            break;
        }
    }

    rangingTable->rangingState = RANGING_STATE_S4;
    RANGING_TABLE_STATE curState = rangingTable->rangingState;

    //DEBUG_PRINT("[S4_RX_NO{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curSubstate, curState, curSubstate);
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
    //DEBUG_PRINT("[S5_TX]: S%d.%d -> S%d.%d\n", prevState, curSubstate, curState, curSubstate);
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
                d_curr[rangingTable->neighborAddress] = (curPToF * VELOCITY) / 2;
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
            d_curr[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? (rangingTable->PToF * VELOCITY) / 2 : (curPToF * VELOCITY) / 2;

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
                d_curr[rangingTable->neighborAddress] = (curPToF * VELOCITY) / 2;

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

    //DEBUG_PRINT("[S5_RX{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curSubstate, curState, (curSubstate == RANGING_SUBSTATE_S3) ? RANGING_SUBSTATE_S3 : curSubstate + 1);
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
            d_curr[rangingTable->neighborAddress] = (curPToF == NULL_TOF) ? (rangingTable->PToF * VELOCITY) / 2 : (curPToF * VELOCITY) / 2;

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
            d_curr[rangingTable->neighborAddress] = (curPToF * VELOCITY) / 2;

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

    //DEBUG_PRINT("[S5_RX_NO{local_%u, neighbor_%u}]: S%d.%d -> S%d.%d\n", MY_UWB_ADDRESS, rangingTable->neighborAddress, prevState, curSubstate, curState, curSubstate);
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
                                                                                          |                                                 normal case                           |
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


/* -------------------- Generate and Process -------------------- */
void generateDSRMessage(Ranging_Message_t *rangingMessage) {
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
        
        RangingTableEventHandler(&rangingTableSet->rangingTable[index], RANGING_EVENT_TX);

        bodyUnitCount++;
    }

    // update priority queue
    updatePriorityQueue(rangingTableSet, bodyUnitCount);

    rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Ranging_Message_Body_Unit_t) * bodyUnitCount;

    rangingMessage->header.type = TYPE_DSR;

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

    // clear expired rangingTable
    if(rangingTableSet->localSeqNumber % CHECK_PERIOD == 0) {
        checkExpirationCallback(rangingTableSet);
    }

    // DEBUG_PRINT("[generate]\n");
    // printRangingMessage(rangingMessage);
}

void processDSRMessage(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo) {
    Ranging_Message_t *rangingMessage = &rangingMessageWithAdditionalInfo->rangingMessage;

    // DEBUG_PRINT("[process]\n");
    // printRangingMessage(rangingMessage);

    if(rangingMessage->header.type != TYPE_DSR) {
        return;
    }

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
    
    /* process bodyUnit */
    Timestamp_Tuple_t Rf = nullTimestampTuple;
    if (rangingMessage->header.filter & (1 << (MY_UWB_ADDRESS % 16))) {
        uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Ranging_Message_Body_Unit_t);
        for(int i = 0; i < bodyUnitCount; i++) {
            if(rangingMessage->bodyUnits[i].address == (uint8_t)MY_UWB_ADDRESS) {
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
        // find corresponding Rr according to Tr to get a valid Tr-Rr pair, this approach may help when experiencing continuous packet loss
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

    /*  
            A   ···      Rp   <--Db-->   Tr      <--Rb-->      Rf

            B   ···   Tp      <--Ra-->      Rr   <--Da-->   Tf                    Re

                                          [SR]   [DSR-IC]                        [DSR]

        禁用场景:
        1. 连续丢包场景下, 历史计算值的参考性对于当下结果借鉴性不大, 为避免可能引入的额外误差
        2. 在相对静止场景下, 系统本身的计算值已经较好, 无需补偿引入额外噪声, 反而可能影响结果
        3. 在减速或者转向的场景下, 补偿结果可能会对当前的运动状态起到副作用
    */

    #ifdef COMPENSATE_ENABLE
        if(d_prev[neighborAddress] == NULL_DIS || last_Seq[neighborAddress] == NULL_SEQ) {
            d_prev[neighborAddress] = d_curr[neighborAddress];
            d_his_avg[neighborAddress] = d_curr[neighborAddress];
            last_Seq[neighborAddress] = rangingMessage->header.msgSequence;
            DEBUG_PRINT("[local_%u <- neighbor_%u]: %s dist = %f", MY_UWB_ADDRESS, neighborAddress, RANGING_MODE, (double)d_curr[neighborAddress]);
        }
        else if(last_SeqGap[neighborAddress] == NULL_SEQ) {
            turning_state[neighborAddress] = (d_curr[neighborAddress] - d_prev[neighborAddress]) >= 0;
            last_SeqGap[neighborAddress] = rangingMessage->header.msgSequence - last_Seq[neighborAddress];
            d_prev[neighborAddress] = d_curr[neighborAddress];
            d_his_avg[neighborAddress] = (d_his_avg[neighborAddress] + d_curr[neighborAddress]) / 2;
            last_Seq[neighborAddress] = rangingMessage->header.msgSequence;
            DEBUG_PRINT("[local_%u <- neighbor_%u]: %s dist = %f", MY_UWB_ADDRESS, neighborAddress, RANGING_MODE, (double)d_curr[neighborAddress]);
        }
        else {
            // calculate d_temp
            uint16_t seqGap = rangingMessage->header.msgSequence - last_Seq[neighborAddress];
            uint16_t total_SeqGap = seqGap + last_SeqGap[neighborAddress];
            float d_delta = d_curr[neighborAddress] - d_prev[neighborAddress];
            d_temp[neighborAddress] = ((float)seqGap / total_SeqGap) * (2 * d_delta);
            float d_comp = d_temp[neighborAddress] * compensation_factor;

            // judge
            bool judge_static = abs(d_curr[neighborAddress] - d_his_avg[neighborAddress]) < STATIC_BOUND;
            bool judge_deceleration = abs(d_curr[neighborAddress] / seqGap) - abs(d_prev[neighborAddress] / last_SeqGap[neighborAddress]) > DECELERATION_BOUND;
            bool judge_turning = ((d_curr[neighborAddress] - d_prev[neighborAddress]) >= 0) ^ turning_state[neighborAddress];

            // update
            turning_state[neighborAddress] = (d_curr[neighborAddress] - d_prev[neighborAddress]) >= 0;
            last_SeqGap[neighborAddress] = seqGap;
            d_prev[neighborAddress] = d_curr[neighborAddress];
            d_his_avg[neighborAddress] = (d_his_avg[neighborAddress] + d_curr[neighborAddress]) / 2;
            last_Seq[neighborAddress] = rangingMessage->header.msgSequence;

            if(compensation_factor != NULL_RATE) {
                float d_corr = (float)d_curr[neighborAddress] + d_comp;
                if(seqGap > SEQGAP_THRESHOLD) {
                    if(seqGap > UINT16_MAX / 2) {
                        last_SeqGap[neighborAddress] = 0;
                    }
                    compensated[neighborAddress] = false;
                    DEBUG_PRINT("[local_%u <- neighbor_%u]: %s dist = %f", MY_UWB_ADDRESS, neighborAddress, RANGING_MODE, (double)d_curr[neighborAddress]);
                }
                else if(judge_static) {
                    compensated[neighborAddress] = false;
                    DEBUG_PRINT("[local_%u <- neighbor_%u]: %s dist = %f", MY_UWB_ADDRESS, neighborAddress, RANGING_MODE, (double)d_curr[neighborAddress]);
                }
                else if(judge_deceleration || judge_turning) {
                    compensated[neighborAddress] = false;
                    DEBUG_PRINT("[local_%u <- neighbor_%u]: %s dist = %f", MY_UWB_ADDRESS, neighborAddress, RANGING_MODE, (double)d_curr[neighborAddress]);
                }
                else {
                    compensated[neighborAddress] = true;
                    DEBUG_PRINT("[local_%u <- neighbor_%u]: %s dist = %f", MY_UWB_ADDRESS, neighborAddress, RANGING_MODE, (double)d_corr);
                }
                compensation_factor = NULL_RATE;
            }
            else {
                DEBUG_PRINT("[local_%u <- neighbor_%u]: %s dist = %f", MY_UWB_ADDRESS, neighborAddress, RANGING_MODE, (double)d_curr[neighborAddress]);
                compensated[neighborAddress] = false;
            }
        }
    #else
        d_prev[neighborAddress] = d_curr[neighborAddress];
        DEBUG_PRINT("[local_%u <- neighbor_%u]: %s dist = %f", MY_UWB_ADDRESS, neighborAddress, RANGING_MODE, (double)d_curr[neighborAddress]);
    #endif

    DEBUG_PRINT(", time = %llu\n", Re.timestamp.full % UWB_MAX_TIMESTAMP);

    rangingTableSet->rangingTable[neighborIndex].expirationSign = false;

    // printRangingTableSet(rangingTableSet);
}

float getCurDistance(uint16_t neighborAddress, uint64_t Rt) {
    index_t neighborIndex = findRangingTable(rangingTableSet, neighborAddress);

    if (neighborIndex == NULL_INDEX) {
        ASSERT(0 && "[getCurDistance]: Please check neighborAddress, it was not found in rangingTableSet\n");
    }

    #ifdef COMPENSATE_ENABLE
        if(compensated[neighborAddress] == false) {
            return (float)d_prev[neighborAddress];
        }
        else if(d_temp[neighborAddress] == NULL_DIS) {
            return (float)d_prev[neighborAddress];
        }
        else if(Rt == NULL_TIMESTAMP) {
            return (float)d_prev[neighborAddress];
        }

        uint64_t diffRxRr = (Rt - ONCE_Rr[neighborAddress] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
        uint64_t diffReRr = (ONCE_Re[neighborAddress] - ONCE_Rr[neighborAddress] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
        uint64_t diffTfRr = (ONCE_Tf[neighborAddress] - ONCE_Rr[neighborAddress] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
        compensation_factor = (float)(diffRxRr - diffTfRr / 2.0) / (float)diffReRr;

        return (float)d_prev[neighborAddress] + d_temp[neighborAddress] * compensation_factor;
    #else
        return (float)d_prev[neighborAddress];
    #endif
}


/* -------------------- TDoA -------------------- */
#ifdef TDOA_COMPAT_ENABLE
#if defined(ANCHOR_MODE_ENABLE)
void anchorTableSetInit() {
    anchorTableSet = (Anchor_Table_Set_t*)malloc(sizeof(Anchor_Table_Set_t));
    anchorTableSet->localAnchorAddress = MY_UWB_ADDRESS;
    anchorTableSet->topIndex = NULL_INDEX;
    for(int i = 0; i < TIMESTAMP_LIST_SIZE; i++) {
        anchorTableSet->broadcastLog[i] = nullTimestampTuple;
    }
    for(int i = 0; i < ANCHOR_SIZE - 1; i++) {
        anchorTableSet->anchorTables[i].neighborAnchorAddress = NULL_ADDRESS;
        anchorTableSet->anchorTables[i].topIndex = NULL_INDEX;
        for(int j = 0; j < TIMESTAMP_LIST_SIZE; j++) {
            anchorTableSet->anchorTables[i].receivedLog[j] = nullTimestampTuple;
        }
        anchorTableSet->anchorTables[i].tableState = UNUSED;
    }
    anchorTableSet->localSeqNumber = NULL_SEQ;
    anchorTableSet->size = 0;
    anchorTableSet->mutex = xSemaphoreCreateMutex();
}

index_t registerAnchorTable(Anchor_Table_Set_t *anchorTableSet, uint16_t neighborAnchorAddress) {
    if(anchorTableSet->size >= ANCHOR_SIZE - 1) {
        DEBUG_PRINT("Anchor table Set is full, cannot register new table\n");
        return NULL_INDEX;
    }

    for (int index = 0; index < ANCHOR_SIZE - 1; index++) {
        if(anchorTableSet->anchorTables[index].tableState == UNUSED) {
            anchorTableSet->anchorTables[index].neighborAnchorAddress = neighborAnchorAddress;
            anchorTableSet->anchorTables[index].tableState = USING;            
            anchorTableSet->size++;
            return index;
        }
    }

    ASSERT(0 && "Warning: Should not be called\n");
    return NULL_INDEX;
}

void updateBroadcastLogForAnchor(Anchor_Table_Set_t *anchorTableSet, Timestamp_Tuple_t timestampTuple) {
    anchorTableSet->topIndex = (anchorTableSet->topIndex + 1) % TIMESTAMP_LIST_SIZE;
    anchorTableSet->broadcastLog[anchorTableSet->topIndex] = timestampTuple;
}

void updateReceivedLogForAnchor(Anchor_Table_t *anchorTable, Timestamp_Tuple_t timestampTuple) {
    anchorTable->topIndex = (anchorTable->topIndex + 1) % TIMESTAMP_LIST_SIZE;
    anchorTable->receivedLog[anchorTable->topIndex] = timestampTuple;
}

table_index_t findAnchorTable(Anchor_Table_Set_t *anchorTableSet, uint16_t address) {
    if(anchorTableSet->size == 0) {
        // DEBUG_PRINT("Anchor table Set is empty, cannot find table\n");
        return NULL_INDEX;
    }

    for (table_index_t index = 0; index < ANCHOR_SIZE - 1; index++) {
        if (anchorTableSet->anchorTables[index].neighborAnchorAddress == address && anchorTableSet->anchorTables[index].tableState == USING) {
            return index;
        }
    }
    // DEBUG_PRINT("Anchor table Set does not contain the address: %u\n", address);
    return NULL_INDEX;
}

void generateTDoAMessage(Ranging_Message_t *rangingMessage) {
    int8_t bodyUnitCount = 0;
    anchorTableSet->localSeqNumber++;

    /* generate bodyUnit */
    // select the nearest anchors first (if the number of available anchors is large, a priority mechanism may be required to further refine the selection)
    index_t tableIndex = 0;
    while (bodyUnitCount < MESSAGE_BODYUNIT_SIZE && tableIndex < ANCHOR_SIZE - 1) {
        if(anchorTableSet->anchorTables[tableIndex].tableState == USING) {
            rangingMessage->bodyUnits[bodyUnitCount].timestamp = anchorTableSet->anchorTables[tableIndex].receivedLog[anchorTableSet->anchorTables->topIndex].timestamp;
            rangingMessage->bodyUnits[bodyUnitCount].seqNumber = anchorTableSet->anchorTables[tableIndex].receivedLog[anchorTableSet->anchorTables->topIndex].seqNumber;
            rangingMessage->bodyUnits[bodyUnitCount].address = anchorTableSet->anchorTables[tableIndex].neighborAnchorAddress;
            bodyUnitCount++;
        }
        tableIndex++;
    }

    rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Ranging_Message_Body_Unit_t) * bodyUnitCount;
    rangingMessage->header.type = TYPE_TDOA;

    // broadcast (16 bits)
    rangingMessage->header.filter = NULL_ADDRESS;

    // fill in empty info
    while(bodyUnitCount < MESSAGE_BODYUNIT_SIZE) {
        rangingMessage->bodyUnits[bodyUnitCount].timestamp.full = NULL_TIMESTAMP;
        rangingMessage->bodyUnits[bodyUnitCount].seqNumber = NULL_SEQ;
        rangingMessage->bodyUnits[bodyUnitCount].address = NULL_ADDR;
        bodyUnitCount++;
    }

    /* generate header */
    rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
    rangingMessage->header.msgSequence = anchorTableSet->localSeqNumber;
    index_t TxIndex = anchorTableSet->topIndex;
    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        if(anchorTableSet->broadcastLog[TxIndex].seqNumber != NULL_SEQ) {
            rangingMessage->header.Txtimestamps[i] = anchorTableSet->broadcastLog[TxIndex];
            TxIndex = (TxIndex - 1 + TIMESTAMP_LIST_SIZE) % TIMESTAMP_LIST_SIZE;
        }
        else {
            rangingMessage->header.Txtimestamps[i] = nullTimestampTuple;
        }
    }
}

void processTDoAMessageForAnchor(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo) {
    Ranging_Message_t *rangingMessage = &rangingMessageWithAdditionalInfo->rangingMessage;

    if(rangingMessage->header.type != TYPE_TDOA) {
        return;
    }

    uint16_t anchorAddress = rangingMessage->header.srcAddress;

    if(anchorAddress > NEIGHBOR_ADDRESS_MAX) {
        DEBUG_PRINT("[processTDoAMessageForAnchor]: anchorAddress > NEIGHBOR_ADDRESS_MAX\n");
        return;
    }

    index_t anchorIndex = findAnchorTable(anchorTableSet, anchorAddress);

    // handle new anchor
    if(anchorIndex == NULL_INDEX) {
        anchorIndex = registerAnchorTable(anchorTableSet, anchorAddress);
        if(anchorIndex == NULL_INDEX) {
            DEBUG_PRINT("Warning: Failed to register new anchor.\n");
            return;
        }
    }

    Anchor_Table_t *anchorTable = &anchorTableSet->anchorTables[anchorIndex];
    Timestamp_Tuple_t timestampTuple = nullTimestampTuple;
    timestampTuple.timestamp.full = rangingMessageWithAdditionalInfo->timestamp.full % UWB_MAX_TIMESTAMP;
    timestampTuple.seqNumber = rangingMessage->header.msgSequence;
    updateReceivedLogForAnchor(anchorTable, timestampTuple);
}

#elif defined(TAG_MODE_ENABLE)
void tagTableSetInit() {
    tagTableSet = (Tag_Table_Set_t*)malloc(sizeof(Tag_Table_Set_t));
    for(int i = 0; i < ANCHOR_SIZE; i++) {
        tagTableSet->tagTables[i].anchorAddress = NULL_ADDRESS;
        tagTableSet->tagTables[i].topIndex = NULL_INDEX;
        for(int j = 0; j < TIMESTAMP_LIST_SIZE; j++) {
            tagTableSet->tagTables[i].broadcastLog[j] = nullTimestampTuple;
            tagTableSet->tagTables[i].receiveLog[j] = nullTimestampTuple;
        }
        for(int j = 0; j < 2 * NEIGHBOR_ADDRESS_MAX; j++) {
            tagTableSet->tagTables[i].anchorLastReceiveLog[j] = nullTimestampTuple;
        }
        tagTableSet->tagTables[i].tableState = UNUSED;
    }
    tagTableSet->size = 0;
    tagTableSet->mutex = xSemaphoreCreateMutex();
}

index_t registerTagTable(Tag_Table_Set_t *tagTableSet, uint16_t anchorAddress) {
    if(tagTableSet->size >= ANCHOR_SIZE) {
        DEBUG_PRINT("Tag table Set is full, cannot register new table\n");
        return NULL_INDEX;
    }

    for(int index = 0; index < ANCHOR_SIZE; index++) {
        if(tagTableSet->tagTables[index].tableState == UNUSED) {
            tagTableSet->tagTables[index].anchorAddress = anchorAddress;
            tagTableSet->tagTables[index].tableState = USING;
            tagTableSet->size++;
            return index;
        }
    }

    ASSERT(0 && "Warning: Should not be called\n");
    return NULL_INDEX;
}

void updateBroadcastLogForTag(Tag_Table_t *tagTable, Timestamp_Tuple_t timestampTuple) {
    for(int i = 0; i < TIMESTAMP_LIST_SIZE; i++) {
        if(tagTable->receiveLog[i].seqNumber == timestampTuple.seqNumber && tagTable->broadcastLog[i].seqNumber == NULL_SEQ) {
            tagTable->broadcastLog[i] = timestampTuple;
        }
    }
}

void updateReceivedLogForTag(Tag_Table_t *tagTable, Timestamp_Tuple_t timestampTuple) {
    tagTable->topIndex = (tagTable->topIndex + 1) % TIMESTAMP_LIST_SIZE;
    tagTable->receiveLog[tagTable->topIndex] = timestampTuple;
    tagTable->broadcastLog[tagTable->topIndex] = nullTimestampTuple;
}

void updateAnchorLastReceiveLogForTag(Tag_Table_t *tagTable,uint16_t address, Timestamp_Tuple_t timestampTuple) {
    if(address > NEIGHBOR_ADDRESS_MAX) {
        ASSERT(0 && "Warning: Anchor address exceeds NEIGHBOR_ADDRESS_MAX\n");
    }
    tagTable->anchorLastReceiveLog[address] = tagTable->anchorLastReceiveLog[address + NEIGHBOR_ADDRESS_MAX];
    tagTable->anchorLastReceiveLog[address + NEIGHBOR_ADDRESS_MAX] = timestampTuple;
}

table_index_t findTagTable(Tag_Table_Set_t *tagTableSet, uint16_t address) {
    if(tagTableSet->size == 0) {
        // DEBUG_PRINT("Tag table Set is empty, cannot find table\n");
        return NULL_INDEX;
    }

    for (table_index_t index = 0; index < ANCHOR_SIZE; index++) {
        if(tagTableSet->tagTables[index].anchorAddress == address && tagTableSet->tagTables[index].tableState == USING) {
            return index;
        }
    }
    // DEBUG_PRINT("Tag table Set does not contain the address: %u\n", address);
    return NULL_INDEX;
}

table_index_t getCollaboratorAnchor(Tag_Table_Set_t *tagTableSet, uint16_t anchorIndex) {
    if(tagTableSet->size < 2) {
        DEBUG_PRINT("At least 2 Anchors are needed to calculate TDoA\n");
        return NULL_INDEX;
    }

    // timestamp of last packet received from this anchor
    uint64_t baseTimestamp = tagTableSet->tagTables[anchorIndex].receiveLog[(tagTableSet->tagTables[anchorIndex].topIndex - 1 + TIMESTAMP_LIST_SIZE) % TIMESTAMP_LIST_SIZE].timestamp.full;
    if(baseTimestamp == NULL_TIMESTAMP) {
        DEBUG_PRINT("Initialization in progress");
        return NULL_INDEX;
    }

    index_t collaboratorIndex = NULL_INDEX;
    dwTime_t collaboratorTimestamp;
    for(int i = 0; i < ANCHOR_SIZE; i++) {
        if(collaboratorIndex == NULL_INDEX && tagTableSet->tagTables[i].tableState == USING) {
            index_t lastReceiveIndex = (tagTableSet->tagTables[i].topIndex - 1 + TIMESTAMP_LIST_SIZE) % TIMESTAMP_LIST_SIZE;

            if(COMPARE_TIME(tagTableSet->tagTables[i].receiveLog[lastReceiveIndex].timestamp.full, baseTimestamp)) {
                collaboratorIndex = i;
                collaboratorTimestamp = tagTableSet->tagTables[i].receiveLog[lastReceiveIndex].timestamp;
            }
        }
        else if(collaboratorIndex != NULL_INDEX && tagTableSet->tagTables[i].tableState == USING) {
            index_t lastReceiveIndex = (tagTableSet->tagTables[i].topIndex - 1 + TIMESTAMP_LIST_SIZE) % TIMESTAMP_LIST_SIZE;

            if(COMPARE_TIME(tagTableSet->tagTables[i].receiveLog[lastReceiveIndex].timestamp.full, baseTimestamp) 
            && COMPARE_TIME(collaboratorTimestamp.full, tagTableSet->tagTables[i].receiveLog[lastReceiveIndex].timestamp.full)) {
                collaboratorIndex = i;
                collaboratorTimestamp = tagTableSet->tagTables[i].receiveLog[lastReceiveIndex].timestamp;
            }
        }
    }
    return collaboratorIndex;
}

void processTDoAMessageForTag(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo) {
    Ranging_Message_t *rangingMessage = &rangingMessageWithAdditionalInfo->rangingMessage;

    if(rangingMessage->header.type != TYPE_TDOA) {
        return;
    }
    
    uint16_t anchorAddress = rangingMessage->header.srcAddress;

    if(anchorAddress > NEIGHBOR_ADDRESS_MAX) {
        DEBUG_PRINT("[processTDoAMessageForTag]: anchorAddress > NEIGHBOR_ADDRESS_MAX\n");
        return;
    }

    index_t anchorIndex = findTagTable(tagTableSet, anchorAddress);

    // handle new anchor
    if(anchorIndex == NULL_INDEX) {
        anchorIndex = registerTagTable(tagTableSet, anchorAddress);
        if(anchorIndex == NULL_INDEX) {
            DEBUG_PRINT("Warning: Failed to register new anchor.\n");
            return;
        }
    }

    Tag_Table_t *tagTable = &tagTableSet->tagTables[anchorIndex];

    // update receiveLog
    Timestamp_Tuple_t receiveTimestampTuple = nullTimestampTuple;
    receiveTimestampTuple.timestamp.full = rangingMessageWithAdditionalInfo->timestamp.full % UWB_MAX_TIMESTAMP;
    receiveTimestampTuple.seqNumber = rangingMessage->header.msgSequence;
    updateReceivedLogForTag(tagTable, receiveTimestampTuple);

    // update broadcastLog
    for(int i = 0; i < MESSAGE_TX_POOL_SIZE; i++) {
        if(rangingMessage->header.Txtimestamps[i].seqNumber != NULL_SEQ) {
            Timestamp_Tuple_t broadcastTimestampTuple = nullTimestampTuple;
            broadcastTimestampTuple.timestamp.full = rangingMessage->header.Txtimestamps[i].timestamp.full % UWB_MAX_TIMESTAMP;
            broadcastTimestampTuple.seqNumber = rangingMessage->header.Txtimestamps[i].seqNumber;
            updateBroadcastLogForTag(tagTable, broadcastTimestampTuple);
        }
    }

    // calculate TDoA
    if(tagTableSet->size < 2) {
        DEBUG_PRINT("At least 2 Anchors are needed to calculate TDoA\n");
        return;
    }

    table_index_t collaboratorIndex = getCollaboratorAnchor(tagTableSet, anchorIndex);
    if(collaboratorIndex != NULL_INDEX) {
        Tag_Table_t collaboratorTable = tagTableSet->tagTables[collaboratorIndex];
        /*
        collaborator    -----------*------------------------------------------------
                           /       |\                /        \                /
                          /        | \              /          \              /
        anchor          -*---------+--*------------*--------------------------------
                          \            \            \            \            \
                           \            \            \            \            \
        tag             ----*------------*------------*-----------------------------
                            P1           P2           P3           P4           P5 (current)
        */

        Timestamp_Tuple_t anchor_P3 = tagTable->broadcastLog[(tagTable->topIndex - 1 + TIMESTAMP_LIST_SIZE) % TIMESTAMP_LIST_SIZE];
        Timestamp_Tuple_t tag_P3 = tagTable->receiveLog[(tagTable->topIndex - 1 + TIMESTAMP_LIST_SIZE) % TIMESTAMP_LIST_SIZE];

        Timestamp_Tuple_t collaborator_P2 = collaboratorTable.broadcastLog[(collaboratorTable.topIndex - 1 + TIMESTAMP_LIST_SIZE) % TIMESTAMP_LIST_SIZE];
        Timestamp_Tuple_t anchor_P2 = tagTable->anchorLastReceiveLog[collaboratorTable.anchorAddress];
        Timestamp_Tuple_t tag_P2 = collaboratorTable.receiveLog[(collaboratorTable.topIndex - 1 + TIMESTAMP_LIST_SIZE) % TIMESTAMP_LIST_SIZE];

        Timestamp_Tuple_t anchor_P1 = tagTable->broadcastLog[(tagTable->topIndex - 2 + TIMESTAMP_LIST_SIZE) % TIMESTAMP_LIST_SIZE];
        Timestamp_Tuple_t tag_P1 = tagTable->receiveLog[(tagTable->topIndex - 2 + TIMESTAMP_LIST_SIZE) % TIMESTAMP_LIST_SIZE];

        DEBUG_PRINT("P3  anchor: ts=%u  seq=%u\n", anchor_P3.timestamp, anchor_P3.seqNumber);
        DEBUG_PRINT("P3     tag: ts=%u  seq=%u\n", tag_P3.timestamp,    tag_P3.seqNumber);

        DEBUG_PRINT("P2  collab: ts=%u  seq=%u\n", collaborator_P2.timestamp, collaborator_P2.seqNumber);
        DEBUG_PRINT("P2  anchor: ts=%u  seq=%u\n", anchor_P2.timestamp,       anchor_P2.seqNumber);
        DEBUG_PRINT("P2     tag: ts=%u  seq=%u\n", tag_P2.timestamp,           tag_P2.seqNumber);

        DEBUG_PRINT("P1  anchor: ts=%u  seq=%u\n", anchor_P1.timestamp, anchor_P1.seqNumber);
        DEBUG_PRINT("P1     tag: ts=%u  seq=%u\n", tag_P1.timestamp,    tag_P1.seqNumber);

        float TDoA = NULL_DIS;
        if(anchor_P3.seqNumber == NULL_SEQ || tag_P3.seqNumber == NULL_SEQ || collaborator_P2.seqNumber == NULL_SEQ ||
        anchor_P2.seqNumber == NULL_SEQ || tag_P2.seqNumber == NULL_SEQ || anchor_P1.seqNumber == NULL_SEQ || tag_P1.seqNumber == NULL_SEQ) {
            DEBUG_PRINT("Insufficient data for calculation\n");
        }
        else if(anchor_P3.seqNumber != tag_P3.seqNumber || collaborator_P2.seqNumber != anchor_P2.seqNumber ||
                anchor_P2.seqNumber != tag_P2.seqNumber || anchor_P1.seqNumber != tag_P1.seqNumber) {
            DEBUG_PRINT("Computation aborted: sequence numbers misaligned\n");
        }
        else {
            uint64_t Rx1_delta = (tag_P3.timestamp.full - tag_P1.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
            uint64_t Tx1_delta = (anchor_P3.timestamp.full - anchor_P1.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
            float alpha = (float)Rx1_delta / (float)Tx1_delta;

            uint64_t Rx_delta = (tag_P3.timestamp.full - tag_P2.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP;
            uint64_t Tx_delta = (anchor_P3.timestamp.full - anchor_P2.timestamp.full + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP + anchor_distance_matrix[tagTable->anchorAddress][collaboratorTable.anchorAddress] / VELOCITY;

            TDoA = Rx_delta - alpha * Tx_delta;
        }
        DEBUG_PRINT("TDoA dist diff = %f, time = %llu\n", (double)TDoA, rangingMessageWithAdditionalInfo->timestamp.full % UWB_MAX_TIMESTAMP);
    }

    // update anchorLastReceiveLog
    int bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Ranging_Message_Body_Unit_t);
    for(int i = 0; i < bodyUnitCount; i++) {
        Timestamp_Tuple_t anchorLastReceiveTimestampTuple = nullTimestampTuple;
        anchorLastReceiveTimestampTuple.timestamp.full = rangingMessage->bodyUnits[i].timestamp.full % UWB_MAX_TIMESTAMP;
        anchorLastReceiveTimestampTuple.seqNumber = rangingMessage->bodyUnits[i].seqNumber;
        updateAnchorLastReceiveLogForTag(tagTable, rangingMessage->bodyUnits[i].address, anchorLastReceiveTimestampTuple);
    }
}

void dispatchTagClusterMode(Ranging_Message_With_Additional_Info_t *rangingMessageWithAdditionalInfo) {
    Ranging_Message_t rangingMessage = rangingMessageWithAdditionalInfo->rangingMessage;
    uint8_t messageType = rangingMessage.header.type;

    // 1> DSR clustered
    if(atomic_load(&ranging_cluster) == TYPE_DSR) {
        if(messageType == TYPE_DSR) {
            xSemaphoreTake(rangingTableSet->mutex, portMAX_DELAY);
            processDSRMessage(rangingMessageWithAdditionalInfo);
            xSemaphoreGive(rangingTableSet->mutex);
        }
        else if(messageType == TYPE_TDOA) {
            // DSR -> TDoA
            atomic_store(&ranging_cluster, TYPE_TDOA);
            lastTDoAReceptionTime = rangingMessageWithAdditionalInfo->timestamp;
            xSemaphoreTake(tagTableSet->mutex, portMAX_DELAY);
            rangingTableSetInit(rangingTableSet);
            processTDoAMessageForTag(rangingMessageWithAdditionalInfo);
            xSemaphoreGive(tagTableSet->mutex);
        }
    }

    // 2> TDoA clustered
    else if(atomic_load(&ranging_cluster) == TYPE_TDOA) {
        if(messageType == TYPE_DSR) {
            // TDoA -> DSR
            if(COMPARE_TIME(lastTDoAReceptionTime.full + TDOA_LOSS_TIME_THRESHOLD, rangingMessageWithAdditionalInfo->timestamp.full)) {
                atomic_store(&ranging_cluster, TYPE_DSR);
                xSemaphoreTake(rangingTableSet->mutex, portMAX_DELAY);
                tagTableSetInit(tagTableSet);
                processDSRMessage(rangingMessageWithAdditionalInfo);
                xSemaphoreGive(rangingTableSet->mutex);
            }
        }
        else if(messageType == TYPE_TDOA) {
            lastTDoAReceptionTime = rangingMessageWithAdditionalInfo->timestamp;
            xSemaphoreTake(tagTableSet->mutex, portMAX_DELAY);
            processTDoAMessageForTag(rangingMessageWithAdditionalInfo);
            xSemaphoreGive(tagTableSet->mutex);
        }
    }

    // 3> error clustered
    else {
        ASSERT(0 && "Warning: Tag is only valid in DSR-clustered or TDoA-clustered mode\n"); 
    }
}
#endif
#endif


#ifndef SIMULATION_COMPILE
/* -------------------- Call back -------------------- */
static void uwbRangingTxTask(void *parameters) {
    systemWaitStart();

    UWB_Packet_t txPacketCache;
    txPacketCache.header.srcAddress = MY_UWB_ADDRESS;
    txPacketCache.header.destAddress = UWB_DEST_ANY;
    txPacketCache.header.type = UWB_RANGING_MESSAGE;
    txPacketCache.header.length = 0;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t*)&txPacketCache.payload;

    #if defined(TDOA_COMPAT_ENABLE) && defined(TAG_MODE_ENABLE)
        atomic_init(&ranging_cluster, TYPE_DSR);
    #endif

    while (true) {
        /*
        pure DSR MODE:
            drones  ->  supports both Tx and Rx
        TDOA_COMPAT_MODE:
            anchor  ->  periodically broadcasts synchronization packets
            tag     1>  remains in DSR mode when broadcast packets are absent for an extended period
                    2>  upon receiving a broadcast, Tx is disabled and the node transitions to TDoA mode
        */

        #ifndef TDOA_COMPAT_ENABLE
            /* pure DSR MDOE */
            xSemaphoreTake(rangingTableSet->mutex, portMAX_DELAY);

            // DEBUG_PRINT("[uwbRangingTxTask]: Acquired mutex, generating ranging message\n");

            generateDSRMessage(rangingMessage);
            // temp
            printRangingMessage(rangingMessage);
            
            txPacketCache.header.seqNumber++;
            txPacketCache.header.length = sizeof(UWB_Packet_Header_t) + rangingMessage->header.msgLength;
            
            // push into txQueue
            uwbSendPacketBlock(&txPacketCache);

            xSemaphoreGive(rangingTableSet->mutex);

            Time_t rangingPeriod = RANGING_PERIOD;

            #ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
                rangingPeriod += rangingPeriodFineTune;
                rangingPeriodFineTune = 0;
            #endif

            vTaskDelay(rangingPeriod);

        #else
            /* TDOA_COMPAT_MODE - anchor */
            #if defined(ANCHOR_MODE_ENABLE)
                xSemaphoreTake(anchorTableSet->mutex, portMAX_DELAY);
                generateTDoAMessage(rangingMessage);
                // temp
                printRangingMessage(rangingMessage);
                
                txPacketCache.header.seqNumber++;
                txPacketCache.header.length = sizeof(UWB_Packet_Header_t) + rangingMessage->header.msgLength;
                
                // push into txQueue
                uwbSendPacketBlock(&txPacketCache);
                
                xSemaphoreGive(anchorTableSet->mutex);
                
                Time_t broadcastPeriod = TDOA_BROADCAST_PERIOD;
                
                vTaskDelay(broadcastPeriod);

            /* TDOA_COMPAT_MODE - tag */
            #elif defined(TAG_MODE_ENABLE)
                // 1> DSR clustered
                if(atomic_load(&ranging_cluster) == TYPE_DSR) {
                    DEBUG_PRINT("***ranging_cluster == TYPE_DSR***\n");
                    xSemaphoreTake(rangingTableSet->mutex, portMAX_DELAY);
                    generateDSRMessage(rangingMessage);
                    // temp
                    printRangingMessage(rangingMessage);
                    
                    txPacketCache.header.seqNumber++;
                    txPacketCache.header.length = sizeof(UWB_Packet_Header_t) + rangingMessage->header.msgLength;
                    
                    // push into txQueue
                    uwbSendPacketBlock(&txPacketCache);

                    xSemaphoreGive(rangingTableSet->mutex);

                    Time_t rangingPeriod = RANGING_PERIOD;

                    #ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
                        rangingPeriod += rangingPeriodFineTune;
                        rangingPeriodFineTune = 0;
                    #endif

                    vTaskDelay(rangingPeriod);
                }

                // 2> TDoA clustered
                else if(atomic_load(&ranging_cluster) == TYPE_TDOA) {
                    DEBUG_PRINT("***ranging_cluster == TYPE_TDOA***\n");
                    Time_t rangingPeriod = RANGING_PERIOD;
                    vTaskDelay(rangingPeriod);
                }

                // 3> error clustered
                else {
                    ASSERT(0 && "Warning: Tag is only valid in DSR-clustered or TDoA-clustered mode\n"); 
                }

            #else
                ASSERT(0 && "Warning: Exactly one of ANCHOR_MODE_ENABLE and TAG_MODE_ENABLE must be enabled\n");    
            #endif
        #endif
    }
}

static void uwbRangingRxTask(void *parameters) {
    systemWaitStart();

    Ranging_Message_With_Additional_Info_t rxPacketCache;

    while (true) {
        vTaskDelay(M2T(1));
        if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
            /*
            pure DSR MODE:
                drones  ->  supports both Tx and Rx
            TDOA_COMPAT_MODE:
                anchor  ->  periodically broadcasts synchronization packets
                tag     1>  remains in DSR mode when broadcast packets are absent for an extended period
                        2>  upon receiving a broadcast, Tx is disabled and the node transitions to TDoA mode
            */

            #ifndef TDOA_COMPAT_ENABLE
                /* pure DSR MDOE */
                xSemaphoreTake(rangingTableSet->mutex, portMAX_DELAY);
                processDSRMessage(&rxPacketCache);
                xSemaphoreGive(rangingTableSet->mutex);

            #else
                /* TDOA_COMPAT_MODE - anchor */
                #if defined(ANCHOR_MODE_ENABLE)
                    xSemaphoreTake(anchorTableSet->mutex, portMAX_DELAY);
                    processTDoAMessageForAnchor(&rxPacketCache);
                    xSemaphoreGive(anchorTableSet->mutex);

                /* TDOA_COMPAT_MODE - tag */
                #elif defined(TAG_MODE_ENABLE)
                    dispatchTagClusterMode(&rxPacketCache);

                #else
                    ASSERT(0 && "Warning: Exactly one of ANCHOR_MODE_ENABLE and TAG_MODE_ENABLE must be enabled\n");    
                #endif
            #endif
        }
    }
}

// called after sending a packet
void rangingTxCallback(void *parameters) {
    UWB_Packet_t *packet = (UWB_Packet_t*)parameters;
    Ranging_Message_t *rangingMessage = (Ranging_Message_t*)packet->payload;
    dwTime_t txTime;
    dwt_readtxtimestamp((uint8_t*)&txTime.raw);
    Timestamp_Tuple_t timestamp = {.timestamp = txTime, .seqNumber = rangingMessage->header.msgSequence};

    #if !defined(TDOA_COMPAT_ENABLE) || (defined(TDOA_COMPAT_ENABLE) && defined(TAG_MODE_ENABLE))
        updateSendList(&rangingTableSet->sendList, timestamp);

        #ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
            Midpoint_Adjustment(rangingTableSet->sendList.Txtimestamps[rangingTableSet->sendList.topIndex].timestamp, &rangingTableSet->receiveList);
        #endif

    #elif defined(TDOA_COMPAT_ENABLE) && defined(ANCHOR_MODE_ENABLE)
        updateBroadcastLogForAnchor(anchorTableSet, timestamp);
    #endif
}

// called after receiving a packet
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

    #ifdef OPTIMAL_RANGING_SCHEDULE_ENABLE
        updateReceiveList(&rangingTableSet->receiveList, rxTime);
    #endif

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

    #ifdef TDOA_COMPAT_ENABLE
        #if defined(ANCHOR_MODE_ENABLE)
            anchorTableSetInit(&anchorTableSet);
        #elif defined(TAG_MODE_ENABLE)
            tagTableSetInit(&tagTableSet);
        #endif
    #endif

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

#ifdef  CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE
LOG_GROUP_START(Ranging)
LOG_ADD(LOG_INT16, distTo00, d_curr + 0)
LOG_ADD(LOG_INT16, distTo01, d_curr + 1)
LOG_ADD(LOG_INT16, distTo02, d_curr + 2)
LOG_ADD(LOG_INT16, distTo03, d_curr + 3)
LOG_ADD(LOG_INT16, distTo04, d_curr + 4)
LOG_ADD(LOG_INT16, distTo05, d_curr + 5)
LOG_ADD(LOG_INT16, distTo06, d_curr + 6)
LOG_ADD(LOG_INT16, distTo07, d_curr + 7)
LOG_ADD(LOG_INT16, distTo08, d_curr + 8)
LOG_ADD(LOG_INT16, distTo09, d_curr + 9)
LOG_ADD(LOG_INT16, distTo10, d_curr + 10)
LOG_ADD(LOG_INT16, distTo11, d_curr + 11)
LOG_ADD(LOG_INT16, distTo12, d_curr + 12)
LOG_ADD(LOG_INT16, distTo13, d_curr + 13)
LOG_ADD(LOG_INT16, distTo14, d_curr + 14)
LOG_ADD(LOG_INT16, distTo15, d_curr + 15)
LOG_ADD(LOG_INT16, distTo16, d_curr + 16)
LOG_ADD(LOG_INT16, distTo17, d_curr + 17)
LOG_ADD(LOG_INT16, distTo18, d_curr + 18)
LOG_ADD(LOG_INT16, distTo19, d_curr + 19)
LOG_ADD(LOG_INT16, distTo20, d_curr + 20)
LOG_ADD(LOG_INT16, distTo21, d_curr + 21)
LOG_ADD(LOG_INT16, distTo22, d_curr + 22)
LOG_ADD(LOG_INT16, distTo23, d_curr + 23)
LOG_ADD(LOG_INT16, distTo24, d_curr + 24)
LOG_ADD(LOG_INT16, distTo25, d_curr + 25)
LOG_ADD(LOG_INT16, distTo26, d_curr + 26)
LOG_ADD(LOG_INT16, distTo27, d_curr + 27)
LOG_ADD(LOG_INT16, distTo28, d_curr + 28)
LOG_ADD(LOG_INT16, distTo29, d_curr + 29)
LOG_GROUP_STOP(Ranging)
#endif
#endif
