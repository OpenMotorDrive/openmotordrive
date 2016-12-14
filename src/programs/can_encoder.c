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
#include <esc/can.h>
#include <esc/encoder.h>
#include <string.h>

void program_init(void) {}

void program_event_adc_sample(float dt) {
    motor_update_state(dt);

    static uint32_t last_send_us = 0;
    uint32_t tnow_us = micros();
    if (tnow_us-last_send_us > 1000) {
        struct canbus_msg m;
        m.id = 500;
        m.ide = false;
        m.rtr = false;
        m.dlc = 4;
        float theta = encoder_get_angle_rad();
        memcpy(m.data, &theta, 4);

        canbus_send_message(&m);
        last_send_us = tnow_us;
    }

    motor_run_commutation(dt);
}
