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

#include <esc/can.h>

// config things - to be made params later
static uint8_t mot_n_poles = 7;
static float elec_theta_bias = 0.0f;
// TODO support reverse
static bool reverse = false;
static const float curr_KR = 0.0f;
static const float curr_KP = .1f;
static const float curr_KI = 350.0f;
static const float vsense_div = 20.0f;
static const float csa_G = 10.0f;
static const float csa_R = 0.001f;
static const float calibration_voltage = 2.0f;

static uint32_t csa_meas_t_us = 0;
static float csa_cal[3] = {0.0f, 0.0f, 0.0f}; // current sense amplifier calibration
static float vbatt_m = 0.0f; // battery voltage
static float ia_m = 0.0f, ib_m = 0.0f, ic_m = 0.0f; // phase currents
static float i_alpha_m = 0.0f, i_beta_m = 0.0f; // alpha-beta-gamma (clarke) transform of phase currents
static float id_meas = 0.0f, iq_meas = 0.0f; // dqo transform of phase currents
static float mech_theta_m = 0.0f; // mechanical rotor angle
static float prev_mech_theta_m = 0.0f; // previous mechanical rotor angle for differentiation
static float elec_theta_m = 0.0f; // electrical rotor angle
static float elec_theta_est = 0.0f;
static float elec_omega_est = 0.0f;
static float mech_omega_est = 0.0f; // mechanical rotor angular velocity
static enum motor_mode_t motor_mode = MOTOR_MODE_DISABLED;
static float omega_ref;
static float omega_integrator;
static float omega_err_filtered;
static float duty_ref;

// double-buffered phase output
static volatile uint8_t phase_output_idx = 0;
static volatile struct {
    uint32_t t_us;
    float duty_alpha;
    float duty_beta;
    float omega;
} phase_output[2];

static struct {
    uint32_t start_time_us;
    uint8_t step;
    float mech_theta_0;
} encoder_calibration_state;

static struct curr_pid_param_s iq_pid_param;
static struct curr_pid_state_s iq_pid_state;

static struct curr_pid_param_s id_pid_param;
static struct curr_pid_state_s id_pid_state;

static void process_adc_measurements(struct adc_sample_s* adc_sample);
static void retrieve_encoder_measurement(void);
static void update_estimates(float dt);
static void load_pid_configs(void);
static void transform_a_b_c_to_alpha_beta(float a, float b, float c, float* alpha, float* beta);
static void transform_d_q_to_alpha_beta(float theta, float d, float q, float* alpha, float* beta);
static void transform_alpha_beta_to_d_q(float theta, float alpha, float beta, float* d, float* q);
static void set_alpha_beta_output_duty(float duty_alpha, float duty_beta, float omega);
static void calc_phase_duties(float* phaseA, float* phaseB, float* phaseC);

static bool ekf_running = false;
static uint8_t ekf_idx = 0;
static float ekf_state[2][5];
static float ekf_cov[2][15];

// EKF parameters
static const float R_s = 0.102f;
static const float L_d = 28*1e-6f;
static const float L_q = 43*1e-6f;
static const float K_v = 360.0f;
static const float J = 0.00003f;
static const float N_P = 7.0f;
static const float i_noise = 0.01f;
static const float u_noise = 0.6f;
static const float T_l_pnoise = 0.01f;

static void ekf_init(float theta)
{
    float* state = ekf_state[ekf_idx];
    float* cov = ekf_cov[ekf_idx];

    ekf_running = true;
    state[0] = 0.0f; // omega
    state[1] = theta;
    state[2] = id_meas;
    state[3] = iq_meas;
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
    cov[9] = ((i_noise)*(i_noise));
    cov[10] = 0;
    cov[11] = 0;
    cov[12] = ((i_noise)*(i_noise));
    cov[13] = 0;
    cov[14] = 0.0100000000000000;
}

void motor_update_ekf(float dt)
{
    float u_alpha = phase_output[(phase_output_idx+1)%2].duty_alpha * vbatt_m;
    float u_beta = phase_output[(phase_output_idx+1)%2].duty_beta * vbatt_m;
    uint8_t next_ekf_idx = (ekf_idx+1)%2;
    float* state = ekf_state[ekf_idx];
    float* cov = ekf_cov[ekf_idx];
    float* state_n = ekf_state[next_ekf_idx];
    float* cov_n = ekf_cov[next_ekf_idx];

    static float subx[77];
    subx[0] = cosf_fast(state[1]);
    subx[1] = sinf_fast(state[1]);
    subx[2] = state[2] + dt*(L_q*N_P*state[0]*state[3] - R_s*state[2] + subx[0]*u_alpha + subx[1]*u_beta)/L_d;
    subx[3] = N_P*dt;
    subx[4] = cosf_fast(state[0]*subx[3] + state[1]);
    subx[5] = 1.0/L_q;
    subx[6] = dt*subx[5]*(-L_d*N_P*state[0]*state[2] - R_s*state[3] + subx[0]*u_beta - subx[1]*u_alpha - 20.0*state[0]/(M_PI*K_v)) + state[3];
    subx[7] = sinf_fast(state[0]*subx[3] + state[1]);
    subx[8] = subx[2]*subx[4] - subx[6]*subx[7];
    subx[9] = dt*(state[2]*(L_d - L_q) + 30.0/(M_PI*J*K_v));
    subx[10] = cov[4]*subx[3] + cov[8];
    subx[11] = dt/J;
    subx[12] = dt*state[3]*(L_d - L_q);
    subx[13] = cov[0]*subx[3] + cov[1] - subx[10]*subx[11] + subx[12]*(cov[2]*subx[3] + cov[6]) + subx[9]*(cov[3]*subx[3] + cov[7]);
    subx[14] = 1 - R_s*dt/L_d;
    subx[15] = dt*(subx[0]*u_beta - subx[1]*u_alpha)/L_d;
    subx[16] = L_q*N_P*dt*state[0]/L_d;
    subx[17] = L_q*N_P*dt*state[3]/L_d;
    subx[18] = cov[10]*subx[14] + cov[12]*subx[16] + cov[3]*subx[17] + cov[7]*subx[15];
    subx[19] = cov[11]*subx[14] + cov[13]*subx[16] + cov[4]*subx[17] + cov[8]*subx[15];
    subx[20] = cov[10]*subx[16] + cov[2]*subx[17] + cov[6]*subx[15] + cov[9]*subx[14];
    subx[21] = cov[0]*subx[17] + cov[1]*subx[15] + cov[2]*subx[14] + cov[3]*subx[16];
    subx[22] = -subx[11]*subx[19] + subx[12]*subx[20] + subx[18]*subx[9] + subx[21];
    subx[23] = -R_s*dt*subx[5] + 1;
    subx[24] = dt*subx[5]*(-L_d*N_P*state[2] - 20.0/(M_PI*K_v));
    subx[25] = dt*subx[5]*(-subx[0]*u_alpha - subx[1]*u_beta);
    subx[26] = L_d*N_P*dt*state[0]*subx[5];
    subx[27] = -cov[10]*subx[26] + cov[12]*subx[23] + cov[3]*subx[24] + cov[7]*subx[25];
    subx[28] = -cov[11]*subx[26] + cov[13]*subx[23] + cov[4]*subx[24] + cov[8]*subx[25];
    subx[29] = cov[10]*subx[23] + cov[2]*subx[24] + cov[6]*subx[25] - cov[9]*subx[26];
    subx[30] = -L_d*cov[2]*state[0]*subx[3]*subx[5] + cov[0]*subx[24] + cov[1]*subx[25] + cov[3]*subx[23];
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
    subx[41] = subx[14]*subx[20] + subx[15]*subx[35] + subx[16]*subx[18] + subx[17]*subx[21] + ((dt)*(dt))*((subx[0])*(subx[0]))*((u_noise)*(u_noise))/((L_d)*(L_d)) + ((dt)*(dt))*((subx[1])*(subx[1]))*((u_noise)*(u_noise))/((L_d)*(L_d));
    subx[42] = subx[36]*subx[8] + subx[40]*subx[4] + subx[41]*subx[7];
    subx[43] = subx[23]*subx[27] + subx[24]*subx[30] + subx[25]*subx[37] - subx[26]*subx[29] + ((dt)*(dt))*((subx[0])*(subx[0]))*((u_noise)*(u_noise))/((L_q)*(L_q)) + ((dt)*(dt))*((subx[1])*(subx[1]))*((u_noise)*(u_noise))/((L_q)*(L_q));
    subx[44] = subx[38]*subx[8] + subx[40]*subx[7] + subx[43]*subx[4];
    subx[45] = subx[33]*subx[39] + subx[42]*subx[4] - subx[44]*subx[7];
    subx[46] = ((i_noise)*(i_noise)) + subx[39]*subx[8] + subx[42]*subx[7] + subx[44]*subx[4];
    subx[47] = subx[33]*subx[34] + subx[36]*subx[4] - subx[38]*subx[7];
    subx[48] = subx[33]*subx[36] - subx[40]*subx[7] + subx[41]*subx[4];
    subx[49] = subx[33]*subx[38] + subx[40]*subx[4] - subx[43]*subx[7];
    subx[50] = ((i_noise)*(i_noise)) + subx[33]*subx[47] + subx[48]*subx[4] - subx[49]*subx[7];
    subx[51] = 1.0/(-((subx[45])*(subx[45])) + subx[46]*subx[50]);
    subx[52] = subx[13]*subx[33] + subx[22]*subx[4] - subx[31]*subx[7];
    subx[53] = subx[45]*subx[51];
    subx[54] = subx[32]*subx[50]*subx[51] - subx[52]*subx[53];
    subx[55] = -subx[32]*subx[53] + subx[46]*subx[51]*subx[52];
    subx[56] = i_alpha_m - subx[2]*subx[4] + subx[6]*subx[7];
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
    state_n[0] = dt*(state[2]*state[3]*(L_d - L_q) - state[4]/J + 30.0*state[3]/(M_PI*J*K_v)) + state[0] + subx[54]*(i_beta_m + subx[33]) + subx[55]*subx[56];
    state_n[1] = state[0]*subx[3] + state[1] + subx[56]*subx[58] + subx[57]*(i_beta_m + subx[33]);
    state_n[2] = subx[2] + subx[56]*subx[60] + subx[59]*(i_beta_m + subx[33]);
    state_n[3] = subx[56]*subx[62] + subx[61]*(i_beta_m + subx[33]) + subx[6];
    state_n[4] = state[4] + subx[56]*subx[66] + subx[65]*(i_beta_m + subx[33]);
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
    cov_n[14] = ((T_l_pnoise)*(T_l_pnoise)) + cov[14] + subx[10]*(-subx[33]*subx[66] - subx[65]*subx[8]) + subx[19]*(-subx[4]*subx[66] - subx[65]*subx[7]) + subx[28]*(-subx[4]*subx[65] + subx[66]*subx[7]);

    state_n[1] = wrap_2pi(state_n[1]);

    ekf_idx = next_ekf_idx;
}

void motor_print_data(float dt) {
    uint8_t slip_msg[64];
    uint8_t slip_msg_len = 0;
    uint8_t i;
    uint32_t tnow_us = micros();
    float omega_e = mech_omega_est*mot_n_poles;
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
        slip_encode_and_append(((uint8_t*)&elec_theta_m)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&omega_e)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&i_alpha_m)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&i_beta_m)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
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

void motor_init(void)
{
    // calibrate phase currents
    uint16_t i;
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
    prev_mech_theta_m = mech_theta_m;
    mech_omega_est = 0.0f;

    pwm_set_phase_duty_callback(calc_phase_duties);
}

void motor_update_state(float dt, struct adc_sample_s* adc_sample)
{
    retrieve_encoder_measurement();
    process_adc_measurements(adc_sample);
    update_estimates(dt);
}

static void run_foc_duty_input(float dt)
{
    load_pid_configs();

    id_pid_param.dt = iq_pid_param.dt = dt;

    id_pid_param.i_meas = id_meas;
    iq_pid_param.i_meas = iq_meas;

    id_pid_param.i_ref = 0.0f;
    iq_pid_param.i_ref = 0.0f;

    id_pid_param.output_limit = vbatt_m/sqrtf(2.0f);

    curr_pid_run(&id_pid_param, &id_pid_state);

    float u_ref = duty_ref*vbatt_m;
    float u_d = id_pid_state.output;
    iq_pid_param.output_limit = sqrtf(MAX(SQ(id_pid_param.output_limit)-SQ(id_pid_state.output),0));
    curr_pid_run(&iq_pid_param, &iq_pid_state);

    float u_q;
    if (u_ref >= 0) {
        u_q = sqrtf(MAX(SQ(u_ref)-SQ(u_d),0));
    } else {
        u_q = -sqrtf(MAX(SQ(u_ref)-SQ(u_d),0));
    }

    float u_alpha, u_beta;
    transform_d_q_to_alpha_beta(elec_theta_est, u_d, u_q, &u_alpha, &u_beta);

    set_alpha_beta_output_duty(u_alpha/vbatt_m, u_beta/vbatt_m, elec_omega_est);
}

static void run_foc(float dt)
{
    load_pid_configs();

    id_pid_param.dt = iq_pid_param.dt  = dt;
    id_pid_param.dt = iq_pid_param.dt = dt;

    id_pid_param.i_meas = id_meas;
    iq_pid_param.i_meas = iq_meas;

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

    transform_d_q_to_alpha_beta(elec_theta_est, id_pid_state.output, iq_pid_state.output, &u_alpha, &u_beta);

    set_alpha_beta_output_duty(u_alpha/vbatt_m, u_beta/vbatt_m, elec_omega_est);
}

void motor_run_commutation(float dt)
{
    float u_alpha, u_beta;
    switch (motor_mode) {
        case MOTOR_MODE_DISABLED:
            set_alpha_beta_output_duty(0, 0, 0);
            break;

        case MOTOR_MODE_DUTY_CONTROL: {
            run_foc_duty_input(dt);
            break;
        }

        case MOTOR_MODE_SPEED_CONTROL: {
            const float tc = 0.005f;
            const float alpha = dt/(dt+tc);
            omega_err_filtered += (omega_ref-elec_omega_est/mot_n_poles - omega_err_filtered) * alpha;
            omega_integrator += omega_err_filtered * 20.0f * dt;
            omega_integrator = constrain_float(omega_integrator,-5.0f,5.0f);
            iq_pid_param.i_ref = constrain_float(omega_err_filtered*0.2f, -5, 5) + omega_integrator;
            run_foc(dt);
            break;
        }

        case MOTOR_MODE_FOC_CURRENT: {
            run_foc(dt);
            break;
        }
        case MOTOR_MODE_ENCODER_CALIBRATION: {
            float t = (micros() - encoder_calibration_state.start_time_us)*1.0e-6f;
            float theta = 0.0f;

            switch(encoder_calibration_state.step) {
                case 0:
                    // the motor is given 1 second to settle
                    theta = 0.0f;
                    if (t > 1.0f) {
                        encoder_calibration_state.mech_theta_0 = mech_theta_m;
                        encoder_calibration_state.step = 1;
                    }

                    break;
                case 1:
                    // theta rotates to 180deg at the 2.0 second mark and is given 0.5 seconds to settle
                    theta = constrain_float(M_PI_F * (t-1.0f)/1.0f, 0.0f, M_PI_F);
                    if (t > 2.5f) {
                        // mot_n_poles = delta_elec_angle/delta_mech_angle
                        float angle_diff = wrap_pi(mech_theta_m - encoder_calibration_state.mech_theta_0);
                        mot_n_poles = (uint8_t)roundf((M_PI_F)/fabsf(angle_diff));

                        // rotating the field in the positive direction should have rotated the encoder in the positive direction too
                        reverse = angle_diff < 0;

                        encoder_calibration_state.step = 2;
                    }

                    break;
                case 2:
                    theta = M_PI_F;

                    // correct elec_theta_bias to zero atan2f(iq_meas, id_meas), which represents the electrical angle error
                    elec_theta_bias = wrap_pi(elec_theta_bias - atan2f(iq_meas, id_meas));

                    motor_set_mode(MOTOR_MODE_DISABLED);
//                     semihost_debug_printf("mot_n_poles %u rev %u\n", mot_n_poles, reverse);
                    break;
            }

            float sin_theta = sinf_fast(theta);
            float cos_theta = cosf_fast(theta);

            float v = constrain_float(calibration_voltage, 0.0f, vbatt_m);
            u_alpha = v * cos_theta;
            u_beta = v * sin_theta;

            set_alpha_beta_output_duty(u_alpha/vbatt_m, u_beta/vbatt_m, 0);

            break;
        }

        case MOTOR_MODE_PHASE_VOLTAGE_TEST: {
            float theta = wrap_2pi(micros()*1e-6f*1000.0f*2.0f*M_PI_F);
            float sin_theta = sinf_fast(theta);
            float cos_theta = cosf_fast(theta);

            float v = constrain_float(0.5f, 0.0f, vbatt_m);
            u_alpha = v * cos_theta;
            u_beta = v * sin_theta;

            set_alpha_beta_output_duty(u_alpha/vbatt_m, u_beta/vbatt_m, 0);
            break;
        }
    }
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
        encoder_calibration_state.start_time_us = micros();
        encoder_calibration_state.step = 0;
    }

    // reset PID states and inputs
    memset(&id_pid_state,0,sizeof(id_pid_state));
    memset(&iq_pid_state,0,sizeof(iq_pid_state));
    id_pid_param.i_ref = 0.0f;
    iq_pid_param.i_ref = 0.0f;

    if (new_mode == MOTOR_MODE_FOC_CURRENT || new_mode == MOTOR_MODE_SPEED_CONTROL || new_mode == MOTOR_MODE_DUTY_CONTROL) {
        ekf_init(M_PI_F);
    }

    motor_mode = new_mode;
}

void motor_set_duty_ref(float val)
{
    duty_ref = val;
}

void motor_set_iq_ref(float iq_ref)
{
    iq_pid_param.i_ref = iq_ref;
}

float motor_get_iq_meas(void)
{
    return iq_meas;
}

enum motor_mode_t motor_get_mode(void)
{
    return motor_mode;
}

float motor_get_phys_rotor_angle(void)
{
    return mech_theta_m;
}

float motor_get_phys_rotor_ang_vel(void)
{
    return mech_omega_est;
}

float motor_get_elec_rotor_angle(void)
{
    return mech_theta_m;
}

float motor_get_vbatt(void)
{
    return vbatt_m;
}

void motor_set_omega_ref(float val)
{
    omega_ref = val;
}

static void process_adc_measurements(struct adc_sample_s* adc_sample)
{
    // Retrieve battery measurement
    vbatt_m = adc_sample->vsense_v * vsense_div;

    // Retrieve current sense amplifier measurement
    csa_meas_t_us = adc_sample->t_us;
    ia_m = (adc_sample->csa_v[0]-csa_cal[0])/(csa_G*csa_R);
    ib_m = (adc_sample->csa_v[1]-csa_cal[1])/(csa_G*csa_R);
    ic_m = (adc_sample->csa_v[2]-csa_cal[2])/(csa_G*csa_R);

    // Reconstruct current measurement
    float duty_a, duty_b, duty_c;
    pwm_get_phase_duty(&duty_a, &duty_b, &duty_c);

    if (duty_a > duty_b && duty_a > duty_c) {
        ia_m = -ib_m-ic_m;
    } else if (duty_b > duty_a && duty_b > duty_c) {
        ib_m = -ia_m-ic_m;
    } else {
        ic_m = -ia_m-ib_m;
    }
}

static void retrieve_encoder_measurement(void)
{
    mech_theta_m = wrap_2pi(encoder_get_angle_rad());
    elec_theta_m = wrap_2pi(mech_theta_m*mot_n_poles-elec_theta_bias);
}

static void update_estimates(float dt)
{
    const float tc = 0.0f;
    const float alpha = dt/(dt+tc);
    mech_omega_est += (wrap_pi(mech_theta_m-prev_mech_theta_m)/dt - mech_omega_est) * alpha;
    prev_mech_theta_m = mech_theta_m;

    transform_a_b_c_to_alpha_beta(ia_m, ib_m, ic_m, &i_alpha_m, &i_beta_m);

    if (ekf_running) {
        float* state = ekf_state[ekf_idx];
        elec_omega_est = state[0] * mot_n_poles;
        elec_theta_est = state[1] + elec_omega_est*dt;
    } else {
        elec_theta_est = elec_theta_m;
        elec_omega_est = mech_omega_est * mot_n_poles;
    }

    transform_alpha_beta_to_d_q(elec_theta_est, i_alpha_m, i_beta_m, &id_meas, &iq_meas);
}

static void load_pid_configs(void)
{
    id_pid_param.K_R = iq_pid_param.K_R = curr_KR;
    id_pid_param.K_P = iq_pid_param.K_P = curr_KP;
    id_pid_param.K_I = iq_pid_param.K_I = curr_KI;
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
