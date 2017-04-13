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

static uint32_t tbegin_us;
static bool waiting_to_start = false;
static bool started = false;
static float t_max = 20.0f;

void program_init(void) {
    // Calibrate the encoder
//     motor_set_mode(MOTOR_MODE_ENCODER_CALIBRATION);
    tbegin_us = micros();
    serial_send_dma(1, "\x55");
}

void program_event_adc_sample(float dt, struct adc_sample_s* adc_sample) {
    uint32_t tnow = micros();
    float t = (tnow-tbegin_us)*1.0e-6f;

    if (motor_get_mode() == MOTOR_MODE_DISABLED && !waiting_to_start && !started) {
        waiting_to_start = true;
        tbegin_us = micros();
    } else if (waiting_to_start && !started && t > 0.1f) {
        tbegin_us = micros();
        started = true;
        motor_set_mode(MOTOR_MODE_FOC_DUTY);
    } else if (started && t >= t_max && motor_get_mode() != MOTOR_MODE_DISABLED) {
        motor_set_mode(MOTOR_MODE_DISABLED);
    }

    if (t < 3) {
        motor_set_duty_ref(0.08f);
    } else if (t < 11.5) {
        float thr = ((uint32_t)((t-1)*2))*0.025f;
        motor_set_duty_ref((((uint32_t)(t*4))%2)==0 ? 0.08f : thr);
    } else if (t < 20.0) {
        float thr = ((uint32_t)((t-9.5)*2))*0.025f;
        motor_set_duty_ref((((uint32_t)(t*4))%2)==0 ? thr : -thr);
    }

    motor_update(dt, adc_sample);

    if (started && motor_get_mode() != MOTOR_MODE_DISABLED) {
        motor_print_data(dt);
    }
}
