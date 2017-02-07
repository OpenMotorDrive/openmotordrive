#include <esc/uavcan.h>
#include <esc/can.h>
#include <esc/timing.h>
#include <esc/helpers.h>
#include <esc/param.h>
#include <stdlib.h>
#include <string.h>
#include <canard.h>
#include <libopencm3/stm32/desig.h>

#define APP_VERSION_MAJOR                                           0
#define APP_VERSION_MINOR                                           1
#define APP_NODE_NAME                                               "org.jc.esc"

#define BIT_LEN_TO_SIZE(x) ((x+7)/8)

#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID                      1
#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE               0x0b2a812620a11d40
#define UAVCAN_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_US             600000U
#define UAVCAN_NODE_ID_ALLOCATION_MAX_REQUEST_PERIOD_US             1000000U
#define UAVCAN_NODE_ID_ALLOCATION_MIN_FOLLOWUP_PERIOD_US            0U
#define UAVCAN_NODE_ID_ALLOCATION_MAX_FOLLOWUP_PERIOD_US            400000U
#define UAVCAN_NODE_ID_ALLOCATION_UID_BIT_OFFSET                    8
#define UAVCAN_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UID_IN_REQUEST      6

#define UAVCAN_NODE_STATUS_MESSAGE_SIZE                             7
#define UAVCAN_NODE_STATUS_DATA_TYPE_ID                             341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE                      0x0f0868d0c1a7c6f1

#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE                      BIT_LEN_TO_SIZE(3015)
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE                    0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID                           1

#define UAVCAN_PARAM_GETSET_REQUEST_MAX_SIZE                        BIT_LEN_TO_SIZE(1791)
#define UAVCAN_PARAM_GETSET_RESPONSE_MAX_SIZE                       BIT_LEN_TO_SIZE(2967)
#define UAVCAN_PARAM_GETSET_DATA_TYPE_ID                            11
#define UAVCAN_PARAM_GETSET_DATA_TYPE_SIGNATURE                     0xa7b622f939d1a4d5

#define UAVCAN_PARAM_EXECUTE_OPCODE_REQUEST_MAX_SIZE                BIT_LEN_TO_SIZE(56)
#define UAVCAN_PARAM_EXECUTE_OPCODE_RESPONSE_MAX_SIZE               BIT_LEN_TO_SIZE(49)
#define UAVCAN_PARAM_EXECUTE_OPCODE_DATA_TYPE_ID                    10
#define UAVCAN_PARAM_EXECUTE_OPCODE_DATA_TYPE_SIGNATURE             0x3b131ac5eb69d2cd

#define UAVCAN_RESTARTNODE_REQUEST_MAX_SIZE                         BIT_LEN_TO_SIZE(40)
#define UAVCAN_RESTARTNODE_RESPONSE_MAX_SIZE                        BIT_LEN_TO_SIZE(1)
#define UAVCAN_RESTARTNODE_DATA_TYPE_ID                             5
#define UAVCAN_RESTARTNODE_DATA_TYPE_SIGNATURE                      0x569e05394a3017f0

#define UAVCAN_ESC_STATUS_MESSAGE_SIZE                              BIT_LEN_TO_SIZE(110)
#define UAVCAN_ESC_STATUS_DATA_TYPE_ID                              1034
#define UAVCAN_ESC_STATUS_DATA_TYPE_SIGNATURE                       0xa9af28aea2fbb254

#define UAVCAN_DEBUG_KEYVALUE_MESSAGE_MAX_SIZE                      BIT_LEN_TO_SIZE(502)
#define UAVCAN_DEBUG_KEYVALUE_DATA_TYPE_ID                          16370
#define UAVCAN_DEBUG_KEYVALUE_DATA_TYPE_SIGNATURE                   0xe02f25d6e0c98ae0

#define UAVCAN_PARAM_VALUE_FLOAT_SIZE                               5

#define UAVCAN_NODE_HEALTH_OK                                       0
#define UAVCAN_NODE_HEALTH_WARNING                                  1
#define UAVCAN_NODE_HEALTH_ERROR                                    2
#define UAVCAN_NODE_HEALTH_CRITICAL                                 3

#define UAVCAN_NODE_MODE_OPERATIONAL                                0
#define UAVCAN_NODE_MODE_INITIALIZATION                             1

#define UNIQUE_ID_LENGTH_BYTES                                      16

static restart_handler_ptr restart_cb;

static CanardInstance canard;
static uint8_t canard_memory_pool[1024];

static uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
static uint8_t node_mode   = UAVCAN_NODE_MODE_INITIALIZATION;

static struct {
    uint32_t request_timer_begin_us;
    uint32_t request_delay_us;
    uint32_t unique_id_offset;
} allocation_state;

static uint8_t node_unique_id[UNIQUE_ID_LENGTH_BYTES];

static uint32_t started_at_sec;
static uint32_t last_1hz_ms;

static float getRandomFloat(void);
static void makeNodeStatusMessage(uint8_t* buffer);
static bool shouldAcceptTransfer(const CanardInstance* ins, uint64_t* out_data_type_signature, uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id);
static void process1HzTasks(void);
static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);

static void allocation_init(void);
static void allocation_update(void);
static bool allocation_running(void);
static void allocation_timer_expired(void);
static void allocation_start_request_timer(void);
static void allocation_start_followup_timer(void);


void uavcan_init(void)
{
    desig_get_unique_id((uint32_t*)&node_unique_id[0]);
    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool), onTransferReceived, shouldAcceptTransfer);
    canardSetLocalNodeID(&canard, *param_retrieve_by_name("uavcan.id-uavcan.equipment.esc-esc_index"));
    allocation_init();
}

void uavcan_update(void)
{
    uint32_t tnow_ms = millis();
    if (tnow_ms-last_1hz_ms >= 1000) {
        process1HzTasks();
        last_1hz_ms = tnow_ms;
    }

    allocation_update();

    // receive
    CanardCANFrame rx_frame;
    struct canbus_msg msg;
    const uint64_t timestamp = micros();
    if (canbus_recv_message(&msg)) {
        rx_frame.id = msg.id & CANARD_CAN_EXT_ID_MASK;
        if (msg.ide) rx_frame.id |= CANARD_CAN_FRAME_EFF;
        if (msg.rtr) rx_frame.id |= CANARD_CAN_FRAME_RTR;
        rx_frame.data_len = msg.dlc;
        memcpy(rx_frame.data, msg.data, 8);
        canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }

    // transmit
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;)
    {
        msg.ide = txf->id & CANARD_CAN_FRAME_EFF;
        msg.rtr = txf->id & CANARD_CAN_FRAME_RTR;
        msg.id = txf->id & CANARD_CAN_EXT_ID_MASK;
        msg.dlc = txf->data_len;
        memcpy(msg.data, txf->data, 8);

        bool success = canbus_send_message(&msg);

        if (success) {
            canardPopTxQueue(&canard);
        } else {
            break;
        }
    }
}

void uavcan_set_restart_cb(restart_handler_ptr cb)
{
    restart_cb = cb;
}

void uavcan_send_debug_key_value(const char* name, float val)
{
    size_t name_len = strlen(name);
    uint8_t msg_buf[UAVCAN_DEBUG_KEYVALUE_MESSAGE_MAX_SIZE];
    memcpy(&msg_buf[0], &val, sizeof(float));
    memcpy(&msg_buf[4], name, name_len);
    uint8_t transfer_id;
    canardBroadcast(&canard, UAVCAN_DEBUG_KEYVALUE_DATA_TYPE_SIGNATURE, UAVCAN_DEBUG_KEYVALUE_DATA_TYPE_ID, &transfer_id, CANARD_TRANSFER_PRIORITY_LOWEST, msg_buf, sizeof(float)+name_len);
}

// Node ID allocation - implementation of http://uavcan.org/Specification/figures/dynamic_node_id_allocatee_algorithm.svg
static void allocation_init(void)
{
    if (!allocation_running()) {
        return;
    }

    // Start request timer
    allocation_start_request_timer();
}

static void allocation_update(void)
{
    if (!allocation_running()) {
        return;
    }

    // Check allocation timer
    if (micros() - allocation_state.request_timer_begin_us > allocation_state.request_delay_us) {
        allocation_timer_expired();

    }
}

static void allocation_timer_expired(void)
{
    if (!allocation_running()) {
        return;
    }

    // Start allocation request timer
    allocation_start_request_timer();

    // Send allocation message
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    uint8_t uid_size = MIN(UNIQUE_ID_LENGTH_BYTES-allocation_state.unique_id_offset, UAVCAN_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UID_IN_REQUEST);
    allocation_request[0] = (allocation_state.unique_id_offset == 0) ? 1 : 0;
    memcpy(&allocation_request[1], &node_unique_id[allocation_state.unique_id_offset], uid_size);

    uint8_t node_id_allocation_transfer_id = 0;
    canardBroadcast(&canard, UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE, UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID, &node_id_allocation_transfer_id, CANARD_TRANSFER_PRIORITY_LOW, allocation_request, uid_size+1);

    allocation_state.unique_id_offset = 0;
}

static void handle_allocation_data_broadcast(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if (!allocation_running()) {
        return;
    }

    // Always start the allocation request timer and reset the UID offset
    allocation_start_request_timer();
    allocation_state.unique_id_offset = 0;

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
        // If source node ID is anonymous, return
        return;
    }

    uint8_t received_unique_id[UNIQUE_ID_LENGTH_BYTES];
    uint8_t received_unique_id_len = transfer->payload_len-1;
    uint8_t i;
    for (i=0; i<received_unique_id_len; i++)
    {
        canardDecodeScalar(transfer, i*8+UAVCAN_NODE_ID_ALLOCATION_UID_BIT_OFFSET, 8, false, &received_unique_id[i]);
    }

    if(memcmp(node_unique_id, received_unique_id, received_unique_id_len) != 0)
    {
        // If unique ID does not match, return
        return;
    }

    if (received_unique_id_len < UNIQUE_ID_LENGTH_BYTES) {
        // Unique ID partially matches - set the UID offset and start the followup timer
        allocation_state.unique_id_offset = received_unique_id_len;
        allocation_start_followup_timer();
    } else {
        // Complete match received
        uint8_t allocated_node_id = 0;
        canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
        if (allocated_node_id != 0) {
            canardSetLocalNodeID(ins, allocated_node_id);
        }
    }
}

static void allocation_start_request_timer(void)
{
    if (!allocation_running()) {
        return;
    }

    allocation_state.request_timer_begin_us = micros();
    allocation_state.request_delay_us = UAVCAN_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_US + (uint32_t)(getRandomFloat() * (UAVCAN_NODE_ID_ALLOCATION_MAX_REQUEST_PERIOD_US-UAVCAN_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_US));
}

static void allocation_start_followup_timer(void)
{
    if (!allocation_running()) {
        return;
    }

    allocation_state.request_timer_begin_us = micros();
    allocation_state.request_delay_us = UAVCAN_NODE_ID_ALLOCATION_MIN_FOLLOWUP_PERIOD_US + (uint32_t)(getRandomFloat() * (UAVCAN_NODE_ID_ALLOCATION_MAX_FOLLOWUP_PERIOD_US-UAVCAN_NODE_ID_ALLOCATION_MIN_FOLLOWUP_PERIOD_US));
}

static bool allocation_running(void)
{
    return canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID;
}

static void process1HzTasks(void)
{
    canardCleanupStaleTransfers(&canard, micros());

    {
        uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
        makeNodeStatusMessage(buffer);

        static uint8_t transfer_id;

        canardBroadcast(&canard, UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE, UAVCAN_NODE_STATUS_DATA_TYPE_ID, &transfer_id, CANARD_TRANSFER_PRIORITY_LOWEST, buffer, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
    }

    node_mode = UAVCAN_NODE_MODE_OPERATIONAL;
}

static void handle_get_node_info_request(CanardInstance* ins, CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE];
    memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
    makeNodeStatusMessage(buffer);

    // Version
    buffer[7] = APP_VERSION_MAJOR;
    buffer[8] = APP_VERSION_MINOR;
    buffer[9] = 1;                          // Optional field flags, VCS commit is set

    // Git hash
    uint32_t u32 = GIT_HASH;
    canardEncodeScalar(buffer, 80, 32, &u32);

    // Unique ID
    memcpy(&buffer[24], node_unique_id, sizeof(node_unique_id));

    // Name
    const size_t name_len = strlen(APP_NODE_NAME);
    memcpy(&buffer[41], APP_NODE_NAME, name_len);

    const size_t total_size = 41 + name_len;

    canardRequestOrRespond(ins, transfer->source_node_id, UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE, UAVCAN_GET_NODE_INFO_DATA_TYPE_ID, &transfer->transfer_id, transfer->priority, CanardResponse, buffer, total_size);
}

static void handle_restart_node_request(CanardInstance* ins, CanardRxTransfer* transfer)
{
    static const uint8_t OK = 1;
    uint8_t resp_buf[UAVCAN_RESTARTNODE_RESPONSE_MAX_SIZE];
    uint64_t magic;
    canardDecodeScalar(transfer, 0, 40, false, &magic);
    if (magic == 0xACCE551B1E) {
        if (restart_cb && restart_cb()) {
            canardEncodeScalar(resp_buf, 0, 1, &OK);
        }
    }

    canardRequestOrRespond(ins, transfer->source_node_id, UAVCAN_RESTARTNODE_DATA_TYPE_SIGNATURE, UAVCAN_RESTARTNODE_DATA_TYPE_ID, &transfer->transfer_id, transfer->priority, CanardResponse, resp_buf, UAVCAN_RESTARTNODE_RESPONSE_MAX_SIZE);
}

static void handle_param_opcode_request(CanardInstance* ins, CanardRxTransfer* transfer)
{
    static const uint8_t OK = 1;
    uint8_t resp_buf[UAVCAN_PARAM_EXECUTE_OPCODE_RESPONSE_MAX_SIZE];
    uint8_t opcode;
    canardDecodeScalar(transfer, 0, 8, false, &opcode);
    memset(resp_buf, 0, sizeof(resp_buf));

    switch(opcode) {
        case 0:
            param_write();
            canardEncodeScalar(resp_buf, 48, 1, &OK);
            break;
        case 1:
            param_erase();
            canardEncodeScalar(resp_buf, 48, 1, &OK);
            break;
    }

    canardRequestOrRespond(ins, transfer->source_node_id, UAVCAN_PARAM_EXECUTE_OPCODE_DATA_TYPE_SIGNATURE, UAVCAN_PARAM_EXECUTE_OPCODE_DATA_TYPE_ID, &transfer->transfer_id, transfer->priority, CanardResponse, resp_buf, UAVCAN_PARAM_EXECUTE_OPCODE_RESPONSE_MAX_SIZE);
}

static void handle_param_getset_request(CanardInstance* ins, CanardRxTransfer* transfer)
{
    int16_t param_idx;
    uint8_t value_type;
    uint8_t value_len;
    uint8_t i;
    char param_name[PARAM_MAX_NAME_LEN+1];
    size_t param_name_len;
    uint8_t resp_buf[UAVCAN_PARAM_GETSET_RESPONSE_MAX_SIZE];
    size_t resp_len = 0;
    memset(param_name, 0, sizeof(param_name));

    canardDecodeScalar(transfer, 0, 13, false, &param_idx);
    canardDecodeScalar(transfer, 13, 3, false, &value_type);
    switch(value_type) {
        case 0: // Empty
            value_len = 0;
            break;
        case 1: // int64
            value_len = 8;
            break;
        case 2: // float32
            value_len = 4;
            break;
        case 3: // uint8
            value_len = 1;
            break;
        case 4: // string
            canardDecodeScalar(transfer, 16, 8, false, &value_len);
            value_len++; // including length byte
            break;
    }

    param_name_len = transfer->payload_len - value_len - 2;

    if (param_name_len > PARAM_MAX_NAME_LEN) {
        // Something went wrong
        return;
    }

    if (param_name_len > 0) {
        // Read in the name if it is not empty
        for (i=0; i<param_name_len; i++) {
            canardDecodeScalar(transfer, 16+value_len*8+i*8, 8, false, &param_name[i]);
        }

        // If the name field is not empty, we are to prefer it over the param index field
        param_idx = param_get_index_by_name(param_name);
    }

    if (param_idx >= 0 && param_idx < param_get_num_params()) {
        // Param exists
        if (value_type == 2) {
            // Set request - attempt to set parameter
            float val;
            canardDecodeScalar(transfer, 16, 32, true, &val);
            *param_retrieve_by_index(param_idx) = val;
        } else if (value_type == 1) {
            int64_t val;
            canardDecodeScalar(transfer, 16, 64, true, &val);
            *param_retrieve_by_index(param_idx) = (float)val;
        }

        float* param_val = param_retrieve_by_index(param_idx);
        const struct param_info_s* param_info;
        param_get_info_by_index(param_idx, &param_info);

        size_t resp_name_len = strlen(param_info->name);

        switch(param_info->type) {
            case PARAM_TYPE_FLOAT: {
                // respond with the param name and value
                resp_buf[0] = 2; // param type
                memcpy(&resp_buf[1], param_val, sizeof(float)); // param value
                resp_buf[5] = 2; // param default value
                memcpy(&resp_buf[6], &param_info->default_val, sizeof(float));
                resp_buf[10] = 2; // param max value
                memcpy(&resp_buf[11], &param_info->max_val, sizeof(float));
                resp_buf[15] = 2; // param min value
                memcpy(&resp_buf[16], &param_info->min_val, sizeof(float));
                memcpy(&resp_buf[20], param_info->name, resp_name_len); // param name
                resp_len = resp_name_len+20;
                break;
            }
            case PARAM_TYPE_INT: {
                // respond with the param name and value
                int64_t temp;
                resp_buf[0] = 1; // param type
                temp = (int64_t)*param_val;
                memcpy(&resp_buf[1], &temp, sizeof(int64_t));
                resp_buf[9] = 1; // param default value
                temp = (int64_t)param_info->default_val;
                memcpy(&resp_buf[10], &temp, sizeof(int64_t));
                resp_buf[18] = 1; // param max value
                temp = (int64_t)param_info->max_val;
                memcpy(&resp_buf[19], &temp, sizeof(int64_t));
                resp_buf[27] = 1; // param min value
                temp = (int64_t)param_info->min_val;
                memcpy(&resp_buf[28], &temp, sizeof(int64_t));
                memcpy(&resp_buf[36], param_info->name, resp_name_len); // param name
                resp_len = resp_name_len+36;
                break;
            }
            case PARAM_TYPE_BOOL: {
                bool temp;
                resp_buf[0] = 3; // param type
                temp = (bool)*param_val;
                memcpy(&resp_buf[1], &temp, sizeof(bool));
                resp_buf[2] = 3; // param default value
                temp = (bool)param_info->default_val;
                memcpy(&resp_buf[3], &temp, sizeof(bool));
                resp_buf[4] = 0; // param max value
                resp_buf[5] = 0; // param min value
                memcpy(&resp_buf[6], param_info->name, resp_name_len); // param name
                resp_len = resp_name_len+6;
                break;
            }
        };
    }  else {
        // Param does not exist, send an empty response per the spec
        resp_buf[0] = 0; // param type
        resp_buf[1] = 0; // param default value
        resp_buf[2] = 0; // param max value
        resp_buf[3] = 0; // param min value
        resp_len = 4;
    }

    canardRequestOrRespond(ins, transfer->source_node_id, UAVCAN_PARAM_GETSET_DATA_TYPE_SIGNATURE, UAVCAN_PARAM_GETSET_DATA_TYPE_ID, &transfer->transfer_id, transfer->priority, CanardResponse, resp_buf, resp_len);
}

static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    if (transfer->transfer_type == CanardTransferTypeBroadcast && transfer->data_type_id == UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID) {
        handle_allocation_data_broadcast(ins, transfer);
    } else if (transfer->transfer_type == CanardTransferTypeRequest && transfer->data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID) {
        handle_get_node_info_request(ins, transfer);
    } else if (transfer->transfer_type == CanardTransferTypeRequest && transfer->data_type_id == UAVCAN_PARAM_GETSET_DATA_TYPE_ID) {
        handle_param_getset_request(ins, transfer);
    } else if (transfer->transfer_type == CanardTransferTypeRequest && transfer->data_type_id == UAVCAN_PARAM_EXECUTE_OPCODE_DATA_TYPE_ID) {
        handle_param_opcode_request(ins, transfer);
    } else if (transfer->transfer_type == CanardTransferTypeRequest && transfer->data_type_id == UAVCAN_RESTARTNODE_DATA_TYPE_ID) {
        handle_restart_node_request(ins, transfer);
    }
}

static void makeNodeStatusMessage(uint8_t* buffer)
{
    memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);

    if (started_at_sec == 0) {
        started_at_sec = millis()/1000U;
    }

    const uint32_t uptime_sec = millis()/1000U - started_at_sec;

    canardEncodeScalar(buffer,  0, 32, &uptime_sec);
    canardEncodeScalar(buffer, 32,  2, &node_health);
    canardEncodeScalar(buffer, 34,  3, &node_mode);
}

static bool shouldAcceptTransfer(const CanardInstance* ins, uint64_t* out_data_type_signature, uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id)
{
    UNUSED(ins);
    UNUSED(source_node_id);
    if (allocation_running())
    {
        if ((transfer_type == CanardTransferTypeBroadcast) &&
            (data_type_id == UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID))
        {
            *out_data_type_signature = UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE;
            return true;
        }
        return false;
    }

    if (transfer_type == CanardTransferTypeRequest && data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID)
    {
        *out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
        return true;
    }

    if (transfer_type == CanardTransferTypeRequest && data_type_id == UAVCAN_PARAM_GETSET_DATA_TYPE_ID)
    {
        *out_data_type_signature = UAVCAN_PARAM_GETSET_DATA_TYPE_SIGNATURE;
        return true;
    }

    if (transfer_type == CanardTransferTypeRequest && data_type_id == UAVCAN_PARAM_EXECUTE_OPCODE_DATA_TYPE_ID)
    {
        *out_data_type_signature = UAVCAN_PARAM_EXECUTE_OPCODE_DATA_TYPE_SIGNATURE;
        return true;
    }

    if (transfer_type == CanardTransferTypeRequest && data_type_id == UAVCAN_RESTARTNODE_DATA_TYPE_ID)
    {
        *out_data_type_signature = UAVCAN_RESTARTNODE_DATA_TYPE_SIGNATURE;
        return true;
    }


    return false;
}

static float getRandomFloat(void)
{
    static bool initialized = false;
    if (!initialized)
    {
        initialized = true;
        srand(micros());
    }

    return (float)rand() / (float)RAND_MAX;
}
