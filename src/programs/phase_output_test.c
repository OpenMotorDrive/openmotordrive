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


#include <esc/program.h>

#include <esc/helpers.h>
#include <esc/motor.h>
#include <esc/timing.h>
#include <esc/serial.h>
#include <esc/drv.h>

#include <stdio.h>

void program_init(void) {
    motor_set_mode(MOTOR_MODE_PHASE_VOLTAGE_TEST);
}

void program_event_adc_sample(float dt) {
    motor_update_state(dt);

    static uint32_t last_print_ms = 0;
    uint32_t tnow_ms = millis();
    if (tnow_ms-last_print_ms > 500) {
        char buf[200];
        int n = 0;
        uint8_t reg;
        for (reg=1; reg<=4; reg++) {
            uint16_t reg_val = drv_read_register(reg);
            n += sprintf(buf+n, "0x%X 0b", reg);
            int8_t i;
            for (i=10; i>=0; i--) {
                buf[n++] = ((reg_val>>i)&1) ? '1' : '0';
            }
            buf[n++] = '\n';
        }
        buf[n++] = '\n';
        serial_send_dma(n, buf);
        last_print_ms = tnow_ms;
    }

    motor_run_commutation(dt);
}
