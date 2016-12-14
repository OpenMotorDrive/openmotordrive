#pragma once

#include <stdint.h>

void canbus_debug_update(void);
void canbus_debug_write(uint8_t len, char* buf);
