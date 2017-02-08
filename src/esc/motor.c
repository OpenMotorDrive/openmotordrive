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
#include <esc/pwm.h>
#include <esc/adc.h>
#include <esc/drv.h>
#include <esc/encoder.h>
#include <esc/timing.h>
#include <esc/curr_pid.h>
#include <esc/serial.h>
#include <esc/semihost_debug.h>
#include <esc/slip.h>
#include <esc/param.h>

#include <esc/can.h>

enum commutaton_method_t {
    COMMUTATION_METHOD_SENSORLESS_EKF=0,
    COMMUTATION_METHOD_ENCODER,
};

static struct {
    float mot_n_poles;
    float elec_theta_bias;
    float vsense_div;
    float csa_R;
    float calibration_voltage;
    float foc_bandwidth_hz;
    float start_current;
    enum commutaton_method_t commutation_method;
    bool reverse;
    float R_s;
    float L_d;
    float L_q;
    float K_v;
    float J;
    float ekf_i_noise;
    float ekf_u_noise;
    float ekf_T_l_pnoise;
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

static float csa_cal[3]; // current sense amplifier calibration
static uint32_t csa_meas_t_us;
static enum motor_mode_t motor_mode = MOTOR_MODE_DISABLED;
static float vbatt_m; // battery voltage
static float duty_ref;

static struct {
    float i_a, i_b, i_c;
    float i_alpha, i_beta;
    float i_d, i_q;
    float elec_theta;
    float elec_omega;
    float mech_omega;
} motor_state;

// double-buffered phase output
static volatile uint8_t phase_output_idx = 0;
static volatile struct {
    uint32_t t_us;
    float duty_alpha;
    float duty_beta;
    float omega;
} phase_output[2];

static struct curr_pid_param_s iq_pid_param;
static struct curr_pid_state_s iq_pid_state;

static struct curr_pid_param_s id_pid_param;
static struct curr_pid_state_s id_pid_state;

static void run_foc(float dt);
static void run_encoder_calibration(void);
static void run_phase_voltage_test(void);
static void process_adc_measurements(struct adc_sample_s* adc_sample);
static void retrieve_encoder_measurement(void);
static void update_encoder_state(float dt);
static void update_motor_state(float dt);
static void transform_a_b_c_to_alpha_beta(float a, float b, float c, float* alpha, float* beta);
static void transform_d_q_to_alpha_beta(float theta, float d, float q, float* alpha, float* beta);
static void transform_alpha_beta_to_d_q(float theta, float alpha, float beta, float* d, float* q);
static void set_alpha_beta_output_duty(float duty_alpha, float duty_beta, float omega);
static void calc_phase_duties(float* phaseA, float* phaseB, float* phaseC);
static void init_ekf(float theta);
static void update_ekf(float dt);

struct ekf_state_s {
    float x[5];
    float P[15];
};

static struct ekf_state_s ekf_state[2];
static uint8_t ekf_idx = 0;

static void load_config(void)
{
    params.mot_n_poles = *param_retrieve_by_name("ESC_MOT_POLES");
    params.elec_theta_bias = *param_retrieve_by_name("ESC_ENC_EBIAS");
    params.vsense_div = *param_retrieve_by_name("ESC_HW_VSENSE_DIV");
    params.csa_R = *param_retrieve_by_name("ESC_HW_CSA_R");
    params.calibration_voltage = *param_retrieve_by_name("ESC_MOT_CAL_V");
    params.foc_bandwidth_hz = *param_retrieve_by_name("ESC_FOC_BANDWIDTH");
    params.start_current = *param_retrieve_by_name("ESC_FOC_START_CURR");
    params.commutation_method = (enum commutaton_method_t)*param_retrieve_by_name("ESC_MOT_COMM_METHOD");
    params.reverse = (bool)*param_retrieve_by_name("ESC_MOT_REVERSE");
    params.R_s = *param_retrieve_by_name("ESC_MOT_R");
    params.L_d = *param_retrieve_by_name("ESC_MOT_L_D");
    params.L_q = *param_retrieve_by_name("ESC_MOT_L_Q");
    params.K_v = *param_retrieve_by_name("ESC_MOT_KV");
    params.J = *param_retrieve_by_name("ESC_MOT_J");
    params.ekf_i_noise = *param_retrieve_by_name("ESC_EKF_CURR_M_NSE");
    params.ekf_u_noise = *param_retrieve_by_name("ESC_EKF_VOLT_NSE");
    params.ekf_T_l_pnoise = *param_retrieve_by_name("ESC_EKF_LOAD_T_PNSE");

    float foc_bandwidth_rads = 2.0f*M_PI_F*params.foc_bandwidth_hz;

    id_pid_param.K_P = params.L_d*foc_bandwidth_rads;
    id_pid_param.K_I = id_pid_param.K_P * params.R_s/params.L_d;

    iq_pid_param.K_P = params.L_q*foc_bandwidth_rads;
    iq_pid_param.K_I = iq_pid_param.K_P * params.R_s/params.L_q;
}

void motor_init(void)
{
    uint16_t i;

    load_config();

    // calibrate phase currents
    struct adc_sample_s adc_sample;
    drv_csa_cal_mode_on();
    usleep(50);
    memset(csa_cal, 0, sizeof(csa_cal));
    for(i=0; i<1000; i++) {
        adc_wait_for_sample();
        adc_get_sample(&adc_sample);
        csa_cal[0] += adc_sample.csa_v[0];
        csa_cal[1] += adc_sample.csa_v[1];
        csa_cal[2] += adc_sample.csa_v[2];
    }
    drv_csa_cal_mode_off();
    csa_cal[0] /= 1000;
    csa_cal[1] /= 1000;
    csa_cal[2] /= 1000;

    // initialize encoder filter states
    retrieve_encoder_measurement();
    encoder_state.prev_mech_theta = encoder_state.mech_theta;
    encoder_state.mech_omega_est = 0.0f;

    pwm_set_phase_duty_callback(calc_phase_duties);
}

void motor_update(float dt, struct adc_sample_s* adc_sample)
{
    process_adc_measurements(adc_sample);
    retrieve_encoder_measurement();
    update_encoder_state(dt);
    update_motor_state(dt);

    switch (motor_mode) {
        case MOTOR_MODE_DISABLED:
            set_alpha_beta_output_duty(0, 0, 0);
            break;

        case MOTOR_MODE_FOC_DUTY: {
            motor_set_iq_ref((duty_ref*vbatt_m - 6.36619772367581*motor_state.mech_omega/params.K_v)/params.R_s);
            run_foc(dt);
            break;
        }

        case MOTOR_MODE_FOC_CURRENT: {
            run_foc(dt);
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

    update_ekf(dt);
}

void motor_set_mode(enum motor_mode_t new_mode)
{
    if (new_mode == motor_mode) {
        return;
    }

    set_alpha_beta_output_duty(0, 0, 0);

    if (new_mode == MOTOR_MODE_DISABLED) {
        drv_6_pwm_mode();
    } else {
        drv_3_pwm_mode();
    }

    if (new_mode == MOTOR_MODE_ENCODER_CALIBRATION) {
        encoder_cal_state.start_time_us = micros();
        encoder_cal_state.step = 0;
    }

    // reset PID states and inputs
    memset(&id_pid_state,0,sizeof(id_pid_state));
    memset(&iq_pid_state,0,sizeof(iq_pid_state));
    id_pid_param.i_ref = 0.0f;
    iq_pid_param.i_ref = 0.0f;

    init_ekf(encoder_state.elec_theta);

    motor_mode = new_mode;
}

void motor_set_duty_ref(float val)
{
    duty_ref = val;
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

float motor_get_vbatt(void)
{
    return vbatt_m;
}

static void run_foc(float dt)
{
    id_pid_param.dt = iq_pid_param.dt  = dt;
    id_pid_param.dt = iq_pid_param.dt = dt;

    id_pid_param.i_meas = motor_state.i_d;
    iq_pid_param.i_meas = motor_state.i_q;

    float u_alpha, u_beta;
    bool overmodulation = false;

    id_pid_param.i_ref = 0.0f;
    if (overmodulation) {
        id_pid_param.output_limit = vbatt_m*sqrtf(2.0f/3.0f);
    } else {
        id_pid_param.output_limit = vbatt_m/sqrtf(2.0f);
    }

    curr_pid_run(&id_pid_param, &id_pid_state);

    // limit iq such that driving id to zero always takes precedence
    iq_pid_param.output_limit = sqrtf(MAX(SQ(id_pid_param.output_limit)-SQ(id_pid_state.output),0));
    curr_pid_run(&iq_pid_param, &iq_pid_state);

    transform_d_q_to_alpha_beta(motor_state.elec_theta, id_pid_state.output, iq_pid_state.output, &u_alpha, &u_beta);

    set_alpha_beta_output_duty(u_alpha/vbatt_m, u_beta/vbatt_m, motor_state.elec_omega);
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
                // params.mot_n_poles = delta_elec_angle/delta_mech_angle
                float angle_diff = wrap_pi(encoder_state.mech_theta - encoder_cal_state.mech_theta_0);
                params.mot_n_poles = (uint8_t)roundf((M_PI_F)/fabsf(angle_diff));

                // rotating the field in the positive direction should have rotated the encoder in the positive direction too
                params.reverse = angle_diff < 0;

                encoder_cal_state.step = 2;
            }

            break;
        case 2:
            theta = M_PI_F;

            // correct params.elec_theta_bias to zero atan2f(motor_state.i_q, motor_state.i_d), which represents the electrical angle error
            params.elec_theta_bias = wrap_pi(-params.elec_theta_bias - atan2f(motor_state.i_q, motor_state.i_d));

            *param_retrieve_by_name("ESC_ENC_EBIAS") = params.elec_theta_bias;
            *param_retrieve_by_name("ESC_MOT_REVERSE") = params.reverse;
            *param_retrieve_by_name("ESC_MOT_POLES") = params.mot_n_poles;
            param_write();

            motor_set_mode(MOTOR_MODE_DISABLED);
            break;
    }

    float sin_theta = sinf_fast(theta);
    float cos_theta = cosf_fast(theta);

    float v = constrain_float(params.calibration_voltage, 0.0f, vbatt_m);
    u_alpha = v * cos_theta;
    u_beta = v * sin_theta;

    set_alpha_beta_output_duty(u_alpha/vbatt_m, u_beta/vbatt_m, 0);
}

static void run_phase_voltage_test(void)
{
    float u_alpha, u_beta;
    float theta = wrap_2pi(micros()*1e-6f*1000.0f*2.0f*M_PI_F);
    float sin_theta = sinf_fast(theta);
    float cos_theta = cosf_fast(theta);

    float v = constrain_float(0.5f, 0.0f, vbatt_m);
    u_alpha = v * cos_theta;
    u_beta = v * sin_theta;

    set_alpha_beta_output_duty(u_alpha/vbatt_m, u_beta/vbatt_m, 0);
}

static void process_adc_measurements(struct adc_sample_s* adc_sample)
{
    // Retrieve battery measurement
    vbatt_m = adc_sample->vsense_v * params.vsense_div;

    // Retrieve current sense amplifier measurement
    csa_meas_t_us = adc_sample->t_us;
    motor_state.i_a = (adc_sample->csa_v[0]-csa_cal[0])/(drv_get_csa_gain()*params.csa_R);
    motor_state.i_b = (adc_sample->csa_v[1]-csa_cal[1])/(drv_get_csa_gain()*params.csa_R);
    motor_state.i_c = (adc_sample->csa_v[2]-csa_cal[2])/(drv_get_csa_gain()*params.csa_R);

    // Reconstruct current measurement
    float duty_a, duty_b, duty_c;
    pwm_get_phase_duty(&duty_a, &duty_b, &duty_c);

    if (duty_a > duty_b && duty_a > duty_c) {
        motor_state.i_a = -motor_state.i_b-motor_state.i_c;
    } else if (duty_b > duty_a && duty_b > duty_c) {
        motor_state.i_b = -motor_state.i_a-motor_state.i_c;
    } else {
        motor_state.i_c = -motor_state.i_a-motor_state.i_b;
    }
}

static void retrieve_encoder_measurement(void)
{
    encoder_state.mech_theta = wrap_2pi(encoder_get_angle_rad());
    encoder_state.elec_theta = wrap_2pi(encoder_state.mech_theta*params.mot_n_poles-params.elec_theta_bias);
}

static void update_encoder_state(float dt)
{
    const float tc = 0.0002f;
    const float alpha = dt/(dt+tc);
    encoder_state.mech_omega_est += (wrap_pi(encoder_state.mech_theta-encoder_state.prev_mech_theta)/dt - encoder_state.mech_omega_est) * alpha;
    encoder_state.prev_mech_theta = encoder_state.mech_theta;
}

static void update_motor_state(float dt)
{
    switch (params.commutation_method) {
        case COMMUTATION_METHOD_SENSORLESS_EKF: {
            motor_state.mech_omega = ekf_state[ekf_idx].x[0];
            motor_state.elec_omega = motor_state.mech_omega * params.mot_n_poles;
            motor_state.elec_theta = ekf_state[ekf_idx].x[1] + motor_state.elec_omega*dt;
            break;
        }
        case COMMUTATION_METHOD_ENCODER: {
            motor_state.mech_omega = encoder_state.mech_omega_est;
            motor_state.elec_omega = motor_state.mech_omega * params.mot_n_poles;
            motor_state.elec_theta = encoder_state.elec_theta;
            break;
        }
    }

    transform_a_b_c_to_alpha_beta(motor_state.i_a, motor_state.i_b, motor_state.i_c, &motor_state.i_alpha, &motor_state.i_beta);
    transform_alpha_beta_to_d_q(motor_state.elec_theta, motor_state.i_alpha, motor_state.i_beta, &motor_state.i_d, &motor_state.i_q);
}

static void transform_a_b_c_to_alpha_beta(float a, float b, float c, float* alpha, float* beta)
{
    *alpha = 0.816496580927726f*a - 0.408248290463863f*b - 0.408248290463863f*c;
    *beta = 0.707106781186547f*b - 0.707106781186547f*c;
}

static void transform_d_q_to_alpha_beta(float theta, float d, float q, float* alpha, float* beta)
{
    float sin_theta = sinf_fast(theta);
    float cos_theta = cosf_fast(theta);

    *alpha = d*cos_theta - q*sin_theta;
    *beta = d*sin_theta + q*cos_theta;
}

static void transform_alpha_beta_to_d_q(float theta, float alpha, float beta, float* d, float* q)
{
    float sin_theta = sinf_fast(theta);
    float cos_theta = cosf_fast(theta);

    *d = alpha*cos_theta + beta*sin_theta;
    *q = -alpha*sin_theta + beta*cos_theta;
}

static void set_alpha_beta_output_duty(float duty_alpha, float duty_beta, float omega)
{
    uint8_t next_phase_output_idx = (phase_output_idx+1)%2;
    phase_output[next_phase_output_idx].t_us = csa_meas_t_us;
    phase_output[next_phase_output_idx].omega = omega;
    phase_output[next_phase_output_idx].duty_alpha = duty_alpha;
    phase_output[next_phase_output_idx].duty_beta = duty_beta;
    phase_output_idx = next_phase_output_idx;
}

static void calc_phase_duties(float* phaseA, float* phaseB, float* phaseC)
{
    if (motor_mode == MOTOR_MODE_DISABLED) {
        (*phaseA) = 0;
        (*phaseB) = 0;
        (*phaseC) = 0;
        return;
    }

    uint32_t tnow_us = micros();
    uint32_t dt_us = tnow_us-phase_output[phase_output_idx].t_us;
    float dt = dt_us*1e-6f;
    float delta_theta = dt * phase_output[phase_output_idx].omega;
    if (dt > 0.01f) {
        (*phaseA) = 0;
        (*phaseB) = 0;
        (*phaseC) = 0;
        return;
    }

    float alpha, beta;
    transform_d_q_to_alpha_beta(delta_theta, phase_output[phase_output_idx].duty_alpha, phase_output[phase_output_idx].duty_beta, &alpha, &beta);

    // Space-vector generator
    // Per http://www.embedded.com/design/real-world-applications/4441150/2/Painless-MCU-implementation-of-space-vector-modulation-for-electric-motor-systems
    // Does not overmodulate, provided the input magnitude is <= 1.0/sqrt(2)
    float Vneutral;

    (*phaseA) = alpha * sqrtf(2.0f/3.0f);
    (*phaseB) = (-(alpha/2.0f)+(beta*sqrtf(3.0f)/2.0f)) * sqrtf(2.0f/3.0f);
    (*phaseC) = (-(alpha/2.0f)-(beta*sqrtf(3.0f)/2.0f)) * sqrtf(2.0f/3.0f);

    Vneutral = 0.5f * (MAX(MAX((*phaseA),(*phaseB)),(*phaseC)) + MIN(MIN((*phaseA),(*phaseB)),(*phaseC)));

    (*phaseA) += 0.5f-Vneutral;
    (*phaseB) += 0.5f-Vneutral;
    (*phaseC) += 0.5f-Vneutral;

    (*phaseA) = constrain_float(*phaseA, 0.0f, 1.0f);
    (*phaseB) = constrain_float(*phaseB, 0.0f, 1.0f);
    (*phaseC) = constrain_float(*phaseC, 0.0f, 1.0f);
}

static void init_ekf(float theta)
{
    float* state = ekf_state[ekf_idx].x;
    float* cov = ekf_state[ekf_idx].P;

    state[0] = 0.0f; // omega
    state[1] = theta;
    state[2] = motor_state.i_d;
    state[3] = motor_state.i_q;
    state[4] = 0;
    // 0 1 2 3
    //   4 5 6
    //     7 8
    //       9
    cov[0] = 100.000000000000;
    cov[1] = 0;
    cov[2] = 0;
    cov[3] = 0;
    cov[4] = 0;
    cov[5] = 9.86960440109;
    cov[6] = 0;
    cov[7] = 0;
    cov[8] = 0;
    cov[9] = ((params.ekf_i_noise)*(params.ekf_i_noise));
    cov[10] = 0;
    cov[11] = 0;
    cov[12] = ((params.ekf_i_noise)*(params.ekf_i_noise));
    cov[13] = 0;
    cov[14] = 0.0100000000000000;
}

static void update_ekf(float dt)
{
    float u_alpha = phase_output[(phase_output_idx+1)%2].duty_alpha * vbatt_m;
    float u_beta = phase_output[(phase_output_idx+1)%2].duty_beta * vbatt_m;

    uint8_t next_ekf_idx = (ekf_idx+1)%2;
    float* state = ekf_state[ekf_idx].x;
    float* cov = ekf_state[ekf_idx].P;
    float* state_n = ekf_state[next_ekf_idx].x;
    float* cov_n = ekf_state[next_ekf_idx].P;

    static float subx[77];
    subx[0] = cosf_fast(state[1]);
    subx[1] = sinf_fast(state[1]);
    subx[2] = state[2] + dt*(params.L_q*params.mot_n_poles*state[0]*state[3] - params.R_s*state[2] + subx[0]*u_alpha + subx[1]*u_beta)/params.L_d;
    subx[3] = params.mot_n_poles*dt;
    subx[4] = cosf_fast(state[0]*subx[3] + state[1]);
    subx[5] = 1.0/params.L_q;
    subx[6] = dt*subx[5]*(-params.L_d*params.mot_n_poles*state[0]*state[2] - params.R_s*state[3] + subx[0]*u_beta - subx[1]*u_alpha - 20.0*state[0]/(M_PI*params.K_v)) + state[3];
    subx[7] = sinf_fast(state[0]*subx[3] + state[1]);
    subx[8] = subx[2]*subx[4] - subx[6]*subx[7];
    subx[9] = dt*(state[2]*(params.L_d - params.L_q) + 30.0/(M_PI*params.J*params.K_v));
    subx[10] = cov[4]*subx[3] + cov[8];
    subx[11] = dt/params.J;
    subx[12] = dt*state[3]*(params.L_d - params.L_q);
    subx[13] = cov[0]*subx[3] + cov[1] - subx[10]*subx[11] + subx[12]*(cov[2]*subx[3] + cov[6]) + subx[9]*(cov[3]*subx[3] + cov[7]);
    subx[14] = 1 - params.R_s*dt/params.L_d;
    subx[15] = dt*(subx[0]*u_beta - subx[1]*u_alpha)/params.L_d;
    subx[16] = params.L_q*params.mot_n_poles*dt*state[0]/params.L_d;
    subx[17] = params.L_q*params.mot_n_poles*dt*state[3]/params.L_d;
    subx[18] = cov[10]*subx[14] + cov[12]*subx[16] + cov[3]*subx[17] + cov[7]*subx[15];
    subx[19] = cov[11]*subx[14] + cov[13]*subx[16] + cov[4]*subx[17] + cov[8]*subx[15];
    subx[20] = cov[10]*subx[16] + cov[2]*subx[17] + cov[6]*subx[15] + cov[9]*subx[14];
    subx[21] = cov[0]*subx[17] + cov[1]*subx[15] + cov[2]*subx[14] + cov[3]*subx[16];
    subx[22] = -subx[11]*subx[19] + subx[12]*subx[20] + subx[18]*subx[9] + subx[21];
    subx[23] = -params.R_s*dt*subx[5] + 1;
    subx[24] = dt*subx[5]*(-params.L_d*params.mot_n_poles*state[2] - 20.0/(M_PI*params.K_v));
    subx[25] = dt*subx[5]*(-subx[0]*u_alpha - subx[1]*u_beta);
    subx[26] = params.L_d*params.mot_n_poles*dt*state[0]*subx[5];
    subx[27] = -cov[10]*subx[26] + cov[12]*subx[23] + cov[3]*subx[24] + cov[7]*subx[25];
    subx[28] = -cov[11]*subx[26] + cov[13]*subx[23] + cov[4]*subx[24] + cov[8]*subx[25];
    subx[29] = cov[10]*subx[23] + cov[2]*subx[24] + cov[6]*subx[25] - cov[9]*subx[26];
    subx[30] = -params.L_d*cov[2]*state[0]*subx[3]*subx[5] + cov[0]*subx[24] + cov[1]*subx[25] + cov[3]*subx[23];
    subx[31] = -subx[11]*subx[28] + subx[12]*subx[29] + subx[27]*subx[9] + subx[30];
    subx[32] = subx[13]*subx[8] + subx[22]*subx[7] + subx[31]*subx[4];
    subx[33] = -subx[2]*subx[7] - subx[4]*subx[6];
    subx[34] = cov[1]*subx[3] + cov[5] + subx[3]*(cov[0]*subx[3] + cov[1]);
    subx[35] = cov[1]*subx[17] + cov[5]*subx[15] + cov[6]*subx[14] + cov[7]*subx[16];
    subx[36] = subx[21]*subx[3] + subx[35];
    subx[37] = cov[1]*subx[24] + cov[5]*subx[25] - cov[6]*subx[26] + cov[7]*subx[23];
    subx[38] = subx[30]*subx[3] + subx[37];
    subx[39] = subx[34]*subx[8] + subx[36]*subx[7] + subx[38]*subx[4];
    subx[40] = subx[14]*subx[29] + subx[15]*subx[37] + subx[16]*subx[27] + subx[17]*subx[30];
    subx[41] = subx[14]*subx[20] + subx[15]*subx[35] + subx[16]*subx[18] + subx[17]*subx[21] + ((dt)*(dt))*((subx[0])*(subx[0]))*((params.ekf_u_noise)*(params.ekf_u_noise))/((params.L_d)*(params.L_d)) + ((dt)*(dt))*((subx[1])*(subx[1]))*((params.ekf_u_noise)*(params.ekf_u_noise))/((params.L_d)*(params.L_d));
    subx[42] = subx[36]*subx[8] + subx[40]*subx[4] + subx[41]*subx[7];
    subx[43] = subx[23]*subx[27] + subx[24]*subx[30] + subx[25]*subx[37] - subx[26]*subx[29] + ((dt)*(dt))*((subx[0])*(subx[0]))*((params.ekf_u_noise)*(params.ekf_u_noise))/((params.L_q)*(params.L_q)) + ((dt)*(dt))*((subx[1])*(subx[1]))*((params.ekf_u_noise)*(params.ekf_u_noise))/((params.L_q)*(params.L_q));
    subx[44] = subx[38]*subx[8] + subx[40]*subx[7] + subx[43]*subx[4];
    subx[45] = subx[33]*subx[39] + subx[42]*subx[4] - subx[44]*subx[7];
    subx[46] = ((params.ekf_i_noise)*(params.ekf_i_noise)) + subx[39]*subx[8] + subx[42]*subx[7] + subx[44]*subx[4];
    subx[47] = subx[33]*subx[34] + subx[36]*subx[4] - subx[38]*subx[7];
    subx[48] = subx[33]*subx[36] - subx[40]*subx[7] + subx[41]*subx[4];
    subx[49] = subx[33]*subx[38] + subx[40]*subx[4] - subx[43]*subx[7];
    subx[50] = ((params.ekf_i_noise)*(params.ekf_i_noise)) + subx[33]*subx[47] + subx[48]*subx[4] - subx[49]*subx[7];
    subx[51] = 1.0/(-((subx[45])*(subx[45])) + subx[46]*subx[50]);
    subx[52] = subx[13]*subx[33] + subx[22]*subx[4] - subx[31]*subx[7];
    subx[53] = subx[45]*subx[51];
    subx[54] = subx[32]*subx[50]*subx[51] - subx[52]*subx[53];
    subx[55] = -subx[32]*subx[53] + subx[46]*subx[51]*subx[52];
    subx[56] = motor_state.i_alpha - subx[2]*subx[4] + subx[6]*subx[7];
    subx[57] = subx[39]*subx[50]*subx[51] - subx[47]*subx[53];
    subx[58] = -subx[39]*subx[53] + subx[46]*subx[47]*subx[51];
    subx[59] = subx[42]*subx[50]*subx[51] - subx[48]*subx[53];
    subx[60] = -subx[42]*subx[53] + subx[46]*subx[48]*subx[51];
    subx[61] = subx[44]*subx[50]*subx[51] - subx[49]*subx[53];
    subx[62] = -subx[44]*subx[53] + subx[46]*subx[49]*subx[51];
    subx[63] = subx[10]*subx[8] + subx[19]*subx[7] + subx[28]*subx[4];
    subx[64] = subx[10]*subx[33] + subx[19]*subx[4] - subx[28]*subx[7];
    subx[65] = subx[50]*subx[51]*subx[63] - subx[53]*subx[64];
    subx[66] = subx[46]*subx[51]*subx[64] - subx[53]*subx[63];
    subx[67] = -subx[4]*subx[54] + subx[55]*subx[7];
    subx[68] = -subx[33]*subx[55] - subx[54]*subx[8];
    subx[69] = -subx[4]*subx[55] - subx[54]*subx[7];
    subx[70] = cov[11]*subx[12] + cov[13]*subx[9] - cov[14]*subx[11] + cov[4];
    subx[71] = -subx[4]*subx[57] + subx[58]*subx[7];
    subx[72] = -subx[4]*subx[58] - subx[57]*subx[7];
    subx[73] = -subx[33]*subx[58] - subx[57]*subx[8] + 1;
    subx[74] = -subx[4]*subx[59] + subx[60]*subx[7];
    subx[75] = -subx[33]*subx[60] - subx[59]*subx[8];
    subx[76] = -subx[4]*subx[60] - subx[59]*subx[7] + 1;
    state_n[0] = dt*(state[2]*state[3]*(params.L_d - params.L_q) - state[4]/params.J + 30.0*state[3]/(M_PI*params.J*params.K_v)) + state[0] + subx[54]*(motor_state.i_beta + subx[33]) + subx[55]*subx[56];
    state_n[1] = state[0]*subx[3] + state[1] + subx[56]*subx[58] + subx[57]*(motor_state.i_beta + subx[33]);
    state_n[2] = subx[2] + subx[56]*subx[60] + subx[59]*(motor_state.i_beta + subx[33]);
    state_n[3] = subx[56]*subx[62] + subx[61]*(motor_state.i_beta + subx[33]) + subx[6];
    state_n[4] = state[4] + subx[56]*subx[66] + subx[65]*(motor_state.i_beta + subx[33]);
    cov_n[0] = cov[0] + cov[2]*subx[12] + cov[3]*subx[9] - cov[4]*subx[11] - subx[11]*subx[70] + subx[12]*(cov[10]*subx[9] - cov[11]*subx[11] + cov[2] + cov[9]*subx[12]) + subx[13]*subx[68] + subx[22]*subx[69] + subx[31]*subx[67] + subx[9]*(cov[10]*subx[12] + cov[12]*subx[9] - cov[13]*subx[11] + cov[3]);
    cov_n[1] = subx[13] + subx[34]*subx[68] + subx[36]*subx[69] + subx[38]*subx[67];
    cov_n[2] = subx[22] + subx[36]*subx[68] + subx[40]*subx[67] + subx[41]*subx[69];
    cov_n[3] = subx[31] + subx[38]*subx[68] + subx[40]*subx[69] + subx[43]*subx[67];
    cov_n[4] = subx[10]*subx[68] + subx[19]*subx[69] + subx[28]*subx[67] + subx[70];
    cov_n[5] = subx[34]*subx[73] + subx[36]*subx[72] + subx[38]*subx[71];
    cov_n[6] = subx[36]*subx[73] + subx[40]*subx[71] + subx[41]*subx[72];
    cov_n[7] = subx[38]*subx[73] + subx[40]*subx[72] + subx[43]*subx[71];
    cov_n[8] = subx[10]*subx[73] + subx[19]*subx[72] + subx[28]*subx[71];
    cov_n[9] = subx[36]*subx[75] + subx[40]*subx[74] + subx[41]*subx[76];
    cov_n[10] = subx[38]*subx[75] + subx[40]*subx[76] + subx[43]*subx[74];
    cov_n[11] = subx[10]*subx[75] + subx[19]*subx[76] + subx[28]*subx[74];
    cov_n[12] = subx[38]*(-subx[33]*subx[62] - subx[61]*subx[8]) + subx[40]*(-subx[4]*subx[62] - subx[61]*subx[7]) + subx[43]*(-subx[4]*subx[61] + subx[62]*subx[7] + 1);
    cov_n[13] = subx[10]*(-subx[33]*subx[62] - subx[61]*subx[8]) + subx[19]*(-subx[4]*subx[62] - subx[61]*subx[7]) + subx[28]*(-subx[4]*subx[61] + subx[62]*subx[7] + 1);
    cov_n[14] = ((params.ekf_T_l_pnoise)*(params.ekf_T_l_pnoise)) + cov[14] + subx[10]*(-subx[33]*subx[66] - subx[65]*subx[8]) + subx[19]*(-subx[4]*subx[66] - subx[65]*subx[7]) + subx[28]*(-subx[4]*subx[65] + subx[66]*subx[7]);

    state_n[1] = wrap_2pi(state_n[1]);

    ekf_idx = next_ekf_idx;
}

void motor_print_data(float dt) {
    uint8_t slip_msg[64];
    uint8_t slip_msg_len = 0;
    uint8_t i;
    uint32_t tnow_us = micros();
    float omega_e = encoder_state.mech_omega_est*params.mot_n_poles;
    uint32_t adc_errcnt = adc_get_errcnt();

    float prev_u_alpha = phase_output[(phase_output_idx+1)%2].duty_alpha * vbatt_m;
    float prev_u_beta = phase_output[(phase_output_idx+1)%2].duty_beta * vbatt_m;

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

    for (i=0; i<sizeof(uint32_t); i++) {
        slip_encode_and_append(((uint8_t*)&adc_errcnt)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    slip_msg[slip_msg_len++] = SLIP_END;

    serial_send_dma(slip_msg_len, (char*)slip_msg);
}
