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

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <esc/adc.h>

enum motor_mode_t {
    MOTOR_MODE_DISABLED = 0,
    MOTOR_MODE_FOC_CURRENT,
    MOTOR_MODE_ENCODER_CALIBRATION,
    MOTOR_MODE_PHASE_VOLTAGE_TEST,
    MOTOR_MODE_DUTY_CONTROL
};

void motor_init(void);
void motor_update_state(float dt, struct adc_sample_s* adc_sample);
void motor_run_commutation(float dt);
void motor_set_mode(enum motor_mode_t mode);
void motor_set_iq_ref(float id_ref);
enum motor_mode_t motor_get_mode(void);
float motor_get_phys_rotor_angle(void);
float motor_get_phys_rotor_ang_vel(void);
float motor_get_elec_rotor_angle(void);
float motor_get_vbatt(void);
float motor_get_iq_est(void);
float motor_get_iq_meas(void);
void motor_print_data(float dt);
void motor_update_ekf(float dt);
void motor_set_omega_ref(float val);
void motor_set_duty_ref(float val);
