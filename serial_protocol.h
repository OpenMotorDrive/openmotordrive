#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdint.h>

enum msg_type_t {
    MSG_TYPE_STATUS=0,
    MSG_TYPE_PARAM_VALUE,
    MSG_TYPE_PARAM_SET
};

struct status_msg_t {
    uint8_t len;
    float batt_voltage;
    uint16_t crc16;
};

#endif // SERIAL_PROTOCOL_H
