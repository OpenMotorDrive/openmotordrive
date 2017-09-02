#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <common/can.h>

enum uavcan_loglevel_t {
    UAVCAN_LOGLEVEL_DEBUG = 0,
    UAVCAN_LOGLEVEL_INFO = 1,
    UAVCAN_LOGLEVEL_WARNING = 2,
    UAVCAN_LOGLEVEL_ERROR = 3
};

typedef bool (*restart_handler_ptr)(void);
typedef void (*esc_rawcommand_handler_ptr)(uint8_t len, int16_t* commands);
typedef void (*unhandled_can_frame_handler_ptr)(struct canbus_msg msg);

void uavcan_init(void);
void uavcan_enable(void);
void uavcan_disable(void);
void uavcan_update(void);
void uavcan_set_restart_cb(restart_handler_ptr cb);
void uavcan_set_esc_rawcommand_cb(esc_rawcommand_handler_ptr cb);
void uavcan_set_unhandled_can_frame_cb(unhandled_can_frame_handler_ptr cb);
void uavcan_send_debug_key_value(const char* name, float val);
void uavcan_send_debug_logmessage(enum uavcan_loglevel_t log_level, const char* source, const char* text);
