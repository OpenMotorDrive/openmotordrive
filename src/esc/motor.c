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

#include <esc/motor.h>

#include <string.h>
#include <esc/helpers.h>
#include <esc/inverter.h>
#include <esc/encoder.h>
#include <esc/timing.h>
#include <esc/curr_pid.h>
#include <esc/serial.h>
#include <esc/semihost_debug.h>
#include <esc/slip.h>
#include <esc/param.h>

#include <esc/uavcan.h>

#include "ekf.h"

enum commutation_method_t {
    COMMUTATION_METHOD_SENSORLESS_EKF=0,
    COMMUTATION_METHOD_ENCODER,
};

static struct {
    float mot_n_pole_pairs;
    float elec_theta_bias;
    float calibration_voltage;
    float foc_bandwidth_hz;
    float start_current;
    enum commutation_method_t commutation_method;
    bool reverse;
    float R_s;
    float L_d;
    float L_q;
    float lambda_m;
    float J;
    float ekf_i_noise;
    float ekf_u_noise;
    float ekf_omega_pnoise;
    float ekf_alpha_load_pnoise;
} params;

static struct {
    float mech_theta; // mechanical rotor angle
    float prev_mech_theta; // previous mechanical rotor angle for differentiation
    float elec_theta; // electrical rotor angle derived from the encoder
    float mech_omega_est; // differentiated encoder measurement
} encoder_state;

static struct {
    uint32_t start_time_us;
    uint8_t step;
    float mech_theta_0;
} encoder_cal_state;

static enum motor_mode_t motor_mode = MOTOR_MODE_DISABLED;
static float duty_ref;
static float duty_tgt;

static float dt;
static struct inverter_sense_data_s inverter_sense_data;
static float prev_u_alpha, prev_u_beta;

static struct {
    float i_a, i_b, i_c;
    float i_alpha, i_beta;
    float i_d, i_q;
    float elec_theta;
    float elec_omega;
    float mech_omega;
} motor_state;

static struct curr_pid_param_s iq_pid_param;
static struct curr_pid_state_s iq_pid_state;

static struct curr_pid_param_s id_pid_param;
static struct curr_pid_state_s id_pid_state;

static struct ekf_obj_s ekf;

static void run_foc(void);
static void run_encoder_calibration(void);
static void run_phase_voltage_test(void);
static void retrieve_encoder_measurement(void);

static void load_config(void)
{
    params.mot_n_pole_pairs = *param_retrieve_by_name("ESC_MOT_POLE_PAIRS");
    params.elec_theta_bias = *param_retrieve_by_name("ESC_ENC_EBIAS");
    params.calibration_voltage = *param_retrieve_by_name("ESC_MOT_CAL_V");
    params.foc_bandwidth_hz = *param_retrieve_by_name("ESC_FOC_BANDWIDTH");
    params.start_current = *param_retrieve_by_name("ESC_FOC_START_CURR");
    params.commutation_method = (enum commutation_method_t)*param_retrieve_by_name("ESC_MOT_COMM_METHOD");
    params.reverse = (bool)*param_retrieve_by_name("ESC_MOT_REVERSE");
    params.R_s = *param_retrieve_by_name("ESC_MOT_R");
    params.L_d = *param_retrieve_by_name("ESC_MOT_L_D");
    params.L_q = *param_retrieve_by_name("ESC_MOT_L_Q");
    params.lambda_m = *param_retrieve_by_name("ESC_MOT_LAMBDA_M");
    params.J = *param_retrieve_by_name("ESC_MOT_J");
    params.ekf_i_noise = *param_retrieve_by_name("ESC_EKF_CURR_M_NSE");
    params.ekf_u_noise = *param_retrieve_by_name("ESC_EKF_VOLT_NSE");
    params.ekf_alpha_load_pnoise = *param_retrieve_by_name("ESC_EKF_ALPHA_LOAD_P_NSE");
    params.ekf_omega_pnoise = *param_retrieve_by_name("ESC_EKF_OMEGA_P_NSE");

    float foc_bandwidth_rads = 2.0f*M_PI_F*params.foc_bandwidth_hz;

    id_pid_param.K_P = params.L_d*foc_bandwidth_rads;
    id_pid_param.K_I = id_pid_param.K_P * params.R_s/params.L_d;

    iq_pid_param.K_P = params.L_q*foc_bandwidth_rads;
    iq_pid_param.K_I = iq_pid_param.K_P * params.R_s/params.L_q;
}

static void retrieve_encoder_measurement(void)
{
    encoder_read_angle();
    encoder_state.mech_theta = wrap_2pi(encoder_get_angle_rad());
    encoder_state.elec_theta = wrap_2pi(encoder_state.mech_theta*params.mot_n_pole_pairs-params.elec_theta_bias);
}

void motor_init(void)
{
    load_config();

    // initialize encoder filter states
    retrieve_encoder_measurement();
    encoder_state.prev_mech_theta = encoder_state.mech_theta;
    encoder_state.mech_omega_est = 0.0f;
}

bool motor_update(void)
{
    // Retreive electrical measurements from inverter, compute dt
    uint8_t prev_seq = inverter_sense_data.seq;
    uint8_t curr_seq = inverter_get_sense_data()->seq;
    uint8_t d_seq = curr_seq-prev_seq;
    if (d_seq < 4) {
        return false;
    }
    inverter_sense_data = *inverter_get_sense_data();
    dt = d_seq*inverter_get_sense_data_sample_period();

    // Retreive angle measurement from encoder
    retrieve_encoder_measurement();

    // Differentiate encoder angle measurement and apply LPF
    const float tc = 0.0002f;
    const float alpha = dt/(dt+tc);
    encoder_state.mech_omega_est += (wrap_pi(encoder_state.mech_theta-encoder_state.prev_mech_theta)/dt - encoder_state.mech_omega_est) * alpha;
    encoder_state.prev_mech_theta = encoder_state.mech_theta;

    // Set motor speed and position states
    switch (params.commutation_method) {
        case COMMUTATION_METHOD_SENSORLESS_EKF: {
            motor_state.mech_omega = ekf_get_state(&ekf)->x[0];
            motor_state.elec_omega = motor_state.mech_omega * params.mot_n_pole_pairs;
            motor_state.elec_theta = ekf_get_state(&ekf)->x[1] + motor_state.elec_omega*dt;
            break;
        }
        case COMMUTATION_METHOD_ENCODER: {
            motor_state.mech_omega = encoder_state.mech_omega_est;
            motor_state.elec_omega = motor_state.mech_omega * params.mot_n_pole_pairs;
            motor_state.elec_theta = encoder_state.elec_theta;
            break;
        }
    }

    // Transform inverter measurements to 2-phase equivalent values in stationary (alpha-beta) and synchronous (d-q) frames
    transform_a_b_c_to_alpha_beta(inverter_sense_data.i_a, inverter_sense_data.i_b, inverter_sense_data.i_c, &motor_state.i_alpha, &motor_state.i_beta);
    transform_alpha_beta_to_d_q(motor_state.elec_theta, motor_state.i_alpha, motor_state.i_beta, &motor_state.i_d, &motor_state.i_q);

    inverter_get_alpha_beta_output_voltages(&prev_u_alpha, &prev_u_beta);

    // Update control loop
    switch (motor_mode) {
        case MOTOR_MODE_DISABLED:
            inverter_disable_output();
            break;

        case MOTOR_MODE_FOC_DUTY: {
            if (duty_ref >= -0.08 && duty_ref <= 0.08) {
                duty_ref = constrain_float(duty_tgt, duty_ref-dt*0.08/3, duty_ref+dt*0.08/3);
            } else {
                duty_ref = duty_tgt;
            }



            motor_set_iq_ref((duty_ref*inverter_sense_data.v_bus/sqrtf(3.0) - motor_state.elec_omega*params.lambda_m)/params.R_s);
            run_foc();
            break;
        }

        case MOTOR_MODE_FOC_CURRENT: {
            run_foc();
            break;
        }
        case MOTOR_MODE_ENCODER_CALIBRATION: {
            run_encoder_calibration();
            break;
        }

        case MOTOR_MODE_PHASE_VOLTAGE_TEST: {
            run_phase_voltage_test();
            break;
        }
    }

    // Update estimator
    ekf_predict(&ekf, dt, prev_u_alpha, prev_u_beta);
    ekf_update(&ekf, motor_state.i_alpha, motor_state.i_beta);
    return true;
}

void motor_set_mode(enum motor_mode_t new_mode)
{
    if (new_mode == motor_mode) {
        return;
    }

    load_config();


    if (new_mode == MOTOR_MODE_DISABLED) {
        inverter_disable_output();
    } else {
        inverter_set_alpha_beta_output_voltages(micros(), 0, 0, 0);
    }

    if (new_mode == MOTOR_MODE_ENCODER_CALIBRATION) {
        params.commutation_method = COMMUTATION_METHOD_ENCODER;
        encoder_cal_state.start_time_us = micros();
        encoder_cal_state.step = 0;
    }

    // reset PID states and inputs
    memset(&id_pid_state,0,sizeof(id_pid_state));
    memset(&iq_pid_state,0,sizeof(iq_pid_state));
    id_pid_param.i_ref = 0.0f;
    iq_pid_param.i_ref = 0.0f;
    duty_tgt = 0;
    duty_ref = 0;

    struct ekf_params_s ekf_params = {
        .R_s = params.R_s,
        .L_d = params.L_d,
        .L_q = params.L_q,
        .lambda_m = params.lambda_m,
        .N_P = params.mot_n_pole_pairs,
        .J = params.J,
        .alpha_load_pnoise = params.ekf_alpha_load_pnoise,
        .omega_pnoise = params.ekf_omega_pnoise,
        .u_noise = params.ekf_u_noise,
        .i_noise = params.ekf_i_noise
    };

    ekf_init(&ekf, &ekf_params, encoder_state.elec_theta);

    motor_mode = new_mode;
}

void motor_set_duty_ref(float val)
{
    duty_tgt = val;
}

void motor_set_iq_ref(float iq_ref)
{
    iq_pid_param.i_ref = constrain_float(iq_ref,-45.0f,45.0f);
}

float motor_get_iq_meas(void)
{
    return motor_state.i_q;
}

enum motor_mode_t motor_get_mode(void)
{
    return motor_mode;
}

float motor_get_phys_rotor_angle(void)
{
    return encoder_state.mech_theta;
}

float motor_get_phys_rotor_ang_vel(void)
{
    return motor_state.mech_omega;
}

float motor_get_elec_rotor_angle(void)
{
    return encoder_state.mech_theta;
}

static void run_foc(void)
{
    id_pid_param.dt = iq_pid_param.dt = dt;
    id_pid_param.dt = iq_pid_param.dt = dt;

    id_pid_param.i_meas = motor_state.i_d;
    iq_pid_param.i_meas = motor_state.i_q;

    float u_alpha, u_beta;
    bool overmodulation = false;

    id_pid_param.i_ref = constrain_float(params.start_current-fabsf(motor_state.elec_omega)/60, 0.0f, params.start_current);

    id_pid_param.ff = 0;//(-params.L_q*motor_state.i_q*motor_state.elec_omega);
    iq_pid_param.ff = 0;//(params.L_d*motor_state.i_d*motor_state.elec_omega + params.lambda_m*motor_state.elec_omega);

    if (overmodulation) {
        id_pid_param.output_limit = inverter_sense_data.v_bus*2.0f/3.0f;
    } else {
        id_pid_param.output_limit = inverter_sense_data.v_bus/sqrtf(3.0f);
    }

    curr_pid_run(&id_pid_param, &id_pid_state);

    // Limit iq such that driving id to zero always takes precedence
    iq_pid_param.output_limit = sqrtf(MAX(SQ(id_pid_param.output_limit)-SQ(id_pid_state.output),0));
    curr_pid_run(&iq_pid_param, &iq_pid_state);

    // Compute u_alpha, u_beta
    transform_d_q_to_alpha_beta(motor_state.elec_theta, id_pid_state.output, iq_pid_state.output, &u_alpha, &u_beta);

    // Output to phases
    inverter_set_alpha_beta_output_voltages(inverter_sense_data.t_us, u_alpha, u_beta, motor_state.elec_omega);
}

static void run_encoder_calibration(void)
{
    float u_alpha, u_beta;
    float t = (micros() - encoder_cal_state.start_time_us)*1.0e-6f;
    float theta = 0.0f;

    switch(encoder_cal_state.step) {
        case 0:
            // the motor is given 1 second to settle
            theta = 0.0f;
            if (t > 1.0f) {
                encoder_cal_state.mech_theta_0 = encoder_state.mech_theta;
                encoder_cal_state.step = 1;
            }

            break;
        case 1:
            // theta rotates to 180deg at the 2.0 second mark and is given 0.5 seconds to settle
            theta = constrain_float(M_PI_F * (t-1.0f)/1.0f, 0.0f, M_PI_F);
            if (t > 2.5f) {
                // params.mot_n_pole_pairs = delta_elec_angle/delta_mech_angle
                float angle_diff = wrap_pi(encoder_state.mech_theta - encoder_cal_state.mech_theta_0);
                params.mot_n_pole_pairs = (uint8_t)roundf((M_PI_F)/fabsf(angle_diff));

                // rotating the field in the positive direction should have rotated the encoder in the positive direction too
                params.reverse = angle_diff < 0;

                encoder_cal_state.step = 2;
            }

            break;
        case 2:
            theta = M_PI_F;

            // correct params.elec_theta_bias to zero atan2f(motor_state.i_q, motor_state.i_d), which represents the electrical angle error
            params.elec_theta_bias = wrap_pi(params.elec_theta_bias - atan2f(motor_state.i_q, motor_state.i_d));

            *param_retrieve_by_name("ESC_ENC_EBIAS") = params.elec_theta_bias;
            *param_retrieve_by_name("ESC_MOT_REVERSE") = params.reverse;
            *param_retrieve_by_name("ESC_MOT_POLES") = params.mot_n_pole_pairs;
            param_write();
//             motor_set_mode(MOTOR_MODE_DISABLED);
            encoder_cal_state.step = 3;
            break;
        case 3:
            theta = constrain_float(M_PI_F + M_PI_F * (t-2.5f)/1.0f, M_PI_F, M_PI_F*2*params.mot_n_pole_pairs);
            if (theta == M_PI_F*2*params.mot_n_pole_pairs) {
                motor_set_mode(MOTOR_MODE_DISABLED);
            }
            break;
    }

    float sin_theta = sinf_fast(theta);
    float cos_theta = cosf_fast(theta);

    float v = constrain_float(params.calibration_voltage, 0.0f, inverter_sense_data.v_bus);
    u_alpha = v * cos_theta;
    u_beta = v * sin_theta;

    inverter_set_alpha_beta_output_voltages(inverter_sense_data.t_us, u_alpha, u_beta, 0);
}

static void run_phase_voltage_test(void)
{
    float u_alpha, u_beta;
    float theta = wrap_2pi(micros()*1e-6f*2.0f*M_PI_F);
    float sin_theta = sinf_fast(theta);
    float cos_theta = cosf_fast(theta);

    float v = constrain_float(1.0f, 0.0f, inverter_sense_data.v_bus);
    u_alpha = v * cos_theta;
    u_beta = v * sin_theta;

    inverter_set_alpha_beta_output_voltages(inverter_sense_data.t_us, u_alpha, u_beta, 0);
}

void motor_print_data(void) {
    uint8_t slip_msg[80];
    uint8_t slip_msg_len = 0;
    uint8_t i;
    uint32_t tnow_us = micros();
    float omega_e = encoder_state.mech_omega_est*params.mot_n_pole_pairs;

    for (i=0; i<sizeof(uint32_t); i++) {
        slip_encode_and_append(((uint8_t*)&tnow_us)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&dt)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&encoder_state.elec_theta)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&omega_e)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&motor_state.i_alpha)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&motor_state.i_beta)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&prev_u_alpha)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&prev_u_beta)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&encoder_state.mech_theta)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&motor_state.i_a)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&motor_state.i_b)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&motor_state.i_c)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    slip_msg[slip_msg_len++] = SLIP_END;

    serial_send_dma(slip_msg_len, (char*)slip_msg);
}
