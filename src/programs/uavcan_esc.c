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


#include <stdbool.h>
#include <esc/program.h>

#include <esc/helpers.h>
#include <esc/motor.h>
#include <esc/timing.h>
#include <esc/semihost_debug.h>
#include <esc/drv.h>
#include <esc/serial.h>
#include <esc/uavcan.h>
#include <esc/param.h>

static uint8_t esc_index;
static uint32_t last_command_us;
static uint32_t last_status_us;

static void handle_uavcan_esc_rawcommand(uint8_t len, int16_t* commands) {
    const float min_duty = 0.08;
    if (esc_index < len) {
        if (commands[esc_index] == 0) {
            motor_set_mode(MOTOR_MODE_DISABLED);
        } else {
            motor_set_mode(MOTOR_MODE_FOC_DUTY);
            if (commands[esc_index] > 0) {
                motor_set_duty_ref(commands[esc_index]/8191.0f*(1.0f-min_duty)+min_duty);
            } else {
                motor_set_duty_ref(commands[esc_index]/8191.0f*(1.0f-min_duty)-min_duty);
            }
        }
        last_command_us = micros();
    }
}

void program_init(void) {
    esc_index = (uint8_t)*param_retrieve_by_name("uavcan.id-uavcan.equipment.esc-esc_index");
    uavcan_set_esc_rawcommand_cb(handle_uavcan_esc_rawcommand);

    motor_set_mode(MOTOR_MODE_DISABLED);
}

void program_event_adc_sample(float dt, struct adc_sample_s* adc_sample) {
    uint32_t tnow_us = micros();

    if (tnow_us-last_command_us > 0.5*1e6) {
        motor_set_mode(MOTOR_MODE_DISABLED);
    }

    if (tnow_us - last_status_us > 0.25*1e6) {
        // TODO: send ESC status message
    }

    motor_update(dt, adc_sample);
}
