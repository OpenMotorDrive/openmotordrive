#pragma once

#include <stdbool.h>

bool semihost_debug_enabled(void);
void semihost_debug_printf(const char *fmt, ...);
