/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <esc/semihost_debug.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>

#define SWD_CONNECTED (*((uint32_t*)0xE000EDF0) & 1)

extern void initialise_monitor_handles(void);
static bool initialized = false;

bool semihost_debug_enabled(void) {
    return SWD_CONNECTED;
}

void semihost_debug_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    if (SWD_CONNECTED) {
        if (!initialized) {
            initialise_monitor_handles();
            initialized = true;
        }
        vprintf(fmt, args);
    }
    va_end(args);
}
