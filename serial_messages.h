#ifndef SERIAL_MESSAGES_H
#define SERIAL_MESSAGES_H

#include <stdint.h>
#include <stdbool.h>

enum ser_msg_type_t {
    MSG_TYPE_PARAM_REQUEST_BY_INDEX=0,
    MSG_TYPE_PARAM_VALUE,
    MSG_TYPE_PARAM_INDEX_INVALID
};

struct ser_msg_param_request_by_index {
    uint16_t msg_id;
    uint8_t param_idx;
    uint16_t crc16;
} __attribute__((packed));

struct ser_msg_param_index_invalid {
    uint16_t msg_id;
    uint8_t param_idx;
    uint16_t crc16;
} __attribute__((packed));

struct ser_msg_param_value {
    uint16_t msg_id;
    uint8_t param_idx;
    char param_name[16];
    float param_value;
    uint16_t crc16;
} __attribute__((packed));


#endif // SERIAL_MESSAGES_H
