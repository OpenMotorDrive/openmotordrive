#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef bool (*restart_handler_ptr)(void);
typedef void (*esc_rawcommand_handler_ptr)(uint8_t len, int16_t* commands);

void uavcan_init(void);
void uavcan_update(void);
void uavcan_set_restart_cb(restart_handler_ptr cb);
void uavcan_set_esc_rawcommand_cb(esc_rawcommand_handler_ptr cb);
void uavcan_send_debug_key_value(const char* name, float val);
