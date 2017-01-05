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

#include <esc/semihost_debug.h>

#include "servo.h"

void program_init(void) {
    // Calibrate the encoder
    motor_set_mode(MOTOR_MODE_ENCODER_CALIBRATION);
}

static uint32_t tbegin_ms;
static float iq_meas_sum;
static uint32_t iq_meas_count;
void program_event_adc_sample(float dt, struct adc_sample_s* adc_sample) {
    motor_update_state(dt, adc_sample);

    servo_run(dt, M_PI_F/4+0.05f);

    if (motor_get_mode() == MOTOR_MODE_FOC_CURRENT && millis()-tbegin_ms > 1000) {
        iq_meas_sum += motor_get_iq_meas();
        iq_meas_count++;
    }

    if (motor_get_mode() == MOTOR_MODE_DISABLED) {
        tbegin_ms = millis();
        iq_meas_sum = 0;
        iq_meas_count = 0;
        motor_set_mode(MOTOR_MODE_FOC_CURRENT);
    } else if (motor_get_mode() == MOTOR_MODE_FOC_CURRENT && millis()-tbegin_ms > 10000) {
        semihost_debug_printf("%f\n", iq_meas_sum/iq_meas_count);
        motor_set_mode(MOTOR_MODE_DISABLED);
    }

    motor_run_commutation(dt);
    motor_update_ekf(dt);
}
