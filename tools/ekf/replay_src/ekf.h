#pragma once
#include <math.h>
#include "math_helpers.h"

struct ekf_state_s {
    float x[5];
    float P[15];
    float innov[2];
    float NIS;
};

static struct ekf_state_s ekf_state[2];
static uint8_t ekf_idx = 0;
static float x_at_curr_meas[5];
static void ekf_init(float init_theta) {
    float* state = ekf_state[ekf_idx].x;
    float* cov = ekf_state[ekf_idx].P;
    cov[0] = 0;
    cov[1] = 0;
    cov[2] = 0;
    cov[3] = 0;
    cov[4] = 0;
    cov[5] = 9.86960440108936;
    cov[6] = 0;
    cov[7] = 0;
    cov[8] = 0;
    cov[9] = 0;
    cov[10] = 0;
    cov[11] = 0;
    cov[12] = 0;
    cov[13] = 0;
    cov[14] = 0;
    memset(&ekf_state[ekf_idx], 0, sizeof(ekf_state[ekf_idx]));
    state[1] = init_theta;
}

static float subx[43];
static void ekf_predict(float dt, float u_alpha, float u_beta) {
    uint8_t next_ekf_idx = (ekf_idx+1)%2;
    float* state = ekf_state[ekf_idx].x;
    float* cov = ekf_state[ekf_idx].P;
    float* state_n = ekf_state[next_ekf_idx].x;
    float* cov_n = ekf_state[next_ekf_idx].P;
    // 333 operations
    subx[0] = 1.0/J;
    subx[1] = -3.0/2.0*L_d*state[2] + (3.0/2.0)*L_q*state[2] + 1.22474487139159*lambda_r;
    subx[2] = N_P*dt;
    subx[3] = cosf_fast(state[1]);
    subx[4] = sinf_fast(state[1]);
    subx[5] = (L_q*N_P*state[0]*state[3] - R_s*state[2] + subx[3]*u_alpha + subx[4]*u_beta)/L_d;
    subx[6] = (-L_d*N_P*state[0]*state[2] - 1.22474487139159*N_P*lambda_r*state[0] - R_s*state[3] + subx[3]*u_beta - subx[4]*u_alpha)/L_q;
    subx[7] = dt*subx[0];
    subx[8] = N_P*dt*subx[0]*subx[1];
    subx[9] = (1.0/2.0)*N_P*dt*state[3]*subx[0]*(-3*L_d + 3*L_q);
    subx[10] = cov[11]*subx[9] + cov[13]*subx[8] - cov[14]*subx[7] + cov[4];
    subx[11] = cov[10]*subx[9] + cov[12]*subx[8] - cov[13]*subx[7] + cov[3];
    subx[12] = cov[10]*subx[8] - cov[11]*subx[7] + cov[2] + cov[9]*subx[9];
    subx[13] = cov[0] + (1.0/2.0)*cov[2]*state[3]*subx[0]*subx[2]*(-3*L_d + 3*L_q) + cov[3]*subx[0]*subx[1]*subx[2] - cov[4]*subx[7];
    subx[14] = cov[1] + cov[6]*subx[9] + cov[7]*subx[8] - cov[8]*subx[7];
    subx[15] = 1 - R_s*dt/L_d;
    subx[16] = dt*(subx[3]*u_beta - subx[4]*u_alpha)/L_d;
    subx[17] = L_q*N_P*dt*state[0]/L_d;
    subx[18] = L_q*state[3]/L_d;
    subx[19] = 1 - R_s*dt/L_q;
    subx[20] = dt*(-subx[3]*u_alpha - subx[4]*u_beta)/L_q;
    subx[21] = dt*(-L_d*N_P*state[2] - 1.22474487139159*N_P*lambda_r)/L_q;
    subx[22] = L_d*N_P*dt*state[0]/L_q;
    subx[23] = cov[10]*subx[17] + cov[2]*subx[18]*subx[2] + cov[6]*subx[16] + cov[9]*subx[15];
    subx[24] = cov[1]*subx[18]*subx[2] + cov[5]*subx[16] + cov[6]*subx[15] + cov[7]*subx[17];
    subx[25] = cov[10]*subx[15] + cov[12]*subx[17] + cov[3]*subx[18]*subx[2] + cov[7]*subx[16];
    subx[26] = cov[0]*subx[18]*subx[2] + cov[1]*subx[16] + cov[2]*subx[15] + cov[3]*subx[17];
    state_n[0] = dt*subx[0]*(N_P*state[3]*subx[1] - state[4]) + state[0];
    state_n[1] = state[0]*subx[2] + state[1];
    state_n[2] = dt*subx[5] + state[2];
    state_n[3] = dt*subx[6] + state[3];
    state_n[4] = state[4];
    x_at_curr_meas[0] = state[0] + subx[0]*(dt - i_delay)*(N_P*state[3]*subx[1] - state[4]);
    x_at_curr_meas[1] = N_P*state[0]*(dt - i_delay) + state[1];
    x_at_curr_meas[2] = state[2] + subx[5]*(dt - i_delay);
    x_at_curr_meas[3] = state[3] + subx[6]*(dt - i_delay);
    x_at_curr_meas[4] = state[4];
    cov_n[0] = ((omega_pnoise)*(omega_pnoise)) - subx[10]*subx[7] + subx[11]*subx[8] + subx[12]*subx[9] + subx[13];
    cov_n[1] = subx[13]*subx[2] + subx[14];
    cov_n[2] = subx[11]*subx[17] + subx[12]*subx[15] + subx[13]*subx[18]*subx[2] + subx[14]*subx[16];
    cov_n[3] = subx[11]*subx[19] - subx[12]*subx[22] + subx[13]*subx[21] + subx[14]*subx[20];
    cov_n[4] = subx[10];
    cov_n[5] = cov[1]*subx[2] + cov[5] + subx[2]*(cov[0]*subx[2] + cov[1]) + ((theta_pnoise)*(theta_pnoise));
    cov_n[6] = subx[15]*(cov[2]*subx[2] + cov[6]) + subx[16]*(cov[1]*subx[2] + cov[5]) + subx[17]*(cov[3]*subx[2] + cov[7]) + subx[18]*subx[2]*(cov[0]*subx[2] + cov[1]);
    cov_n[7] = subx[19]*(cov[3]*subx[2] + cov[7]) + subx[20]*(cov[1]*subx[2] + cov[5]) + subx[21]*(cov[0]*subx[2] + cov[1]) - subx[22]*(cov[2]*subx[2] + cov[6]);
    cov_n[8] = cov[4]*subx[2] + cov[8];
    cov_n[9] = N_P*dt*subx[18]*subx[26] + subx[15]*subx[23] + subx[16]*subx[24] + subx[17]*subx[25] + ((dt)*(dt))*((subx[3])*(subx[3]))*((u_noise)*(u_noise))/((L_d)*(L_d)) + ((dt)*(dt))*((subx[4])*(subx[4]))*((u_noise)*(u_noise))/((L_d)*(L_d));
    cov_n[10] = subx[19]*subx[25] + subx[20]*subx[24] + subx[21]*subx[26] - subx[22]*subx[23];
    cov_n[11] = cov[11]*subx[15] + cov[13]*subx[17] + cov[4]*subx[18]*subx[2] + cov[8]*subx[16];
    cov_n[12] = subx[19]*(-cov[10]*subx[22] + cov[12]*subx[19] + cov[3]*subx[21] + cov[7]*subx[20]) + subx[20]*(cov[1]*subx[21] + cov[5]*subx[20] - cov[6]*subx[22] + cov[7]*subx[19]) + subx[21]*(cov[0]*subx[21] + cov[1]*subx[20] - cov[2]*subx[22] + cov[3]*subx[19]) - subx[22]*(cov[10]*subx[19] + cov[2]*subx[21] + cov[6]*subx[20] - cov[9]*subx[22]) + ((dt)*(dt))*((subx[3])*(subx[3]))*((u_noise)*(u_noise))/((L_q)*(L_q)) + ((dt)*(dt))*((subx[4])*(subx[4]))*((u_noise)*(u_noise))/((L_q)*(L_q));
    cov_n[13] = -cov[11]*subx[22] + cov[13]*subx[19] + cov[4]*subx[21] + cov[8]*subx[20];
    cov_n[14] = ((T_l_pnoise)*(T_l_pnoise)) + cov[14];

    state_n[1] = wrap_2pi(state_n[1]);
    ekf_idx = next_ekf_idx;
}

static void ekf_update(float i_alpha_m, float i_beta_m) {
    uint8_t next_ekf_idx = (ekf_idx+1)%2;
    float* state = x_at_curr_meas;
    float* cov = ekf_state[ekf_idx].P;
    float* state_n = ekf_state[next_ekf_idx].x;
    float* cov_n = ekf_state[next_ekf_idx].P;
    float* innov = ekf_state[next_ekf_idx].innov;
    float* NIS = &ekf_state[next_ekf_idx].NIS;

    // 294 operations
    subx[0] = cosf_fast(state[1]);
    subx[1] = sinf_fast(state[1]);
    subx[2] = state[2]*subx[0] - state[3]*subx[1];
    subx[3] = cov[1]*subx[2] + cov[2]*subx[1] + cov[3]*subx[0];
    subx[4] = -state[2]*subx[1] - state[3]*subx[0];
    subx[5] = cov[5]*subx[2] + cov[6]*subx[1] + cov[7]*subx[0];
    subx[6] = cov[10]*subx[0] + cov[6]*subx[2] + cov[9]*subx[1];
    subx[7] = cov[10]*subx[1] + cov[12]*subx[0] + cov[7]*subx[2];
    subx[8] = subx[0]*subx[6] - subx[1]*subx[7] + subx[4]*subx[5];
    subx[9] = ((i_noise + 0.05*sqrtf(((i_alpha_m)*(i_alpha_m)) + ((i_beta_m)*(i_beta_m))))*(i_noise + 0.05*sqrtf(((i_alpha_m)*(i_alpha_m)) + ((i_beta_m)*(i_beta_m)))));
    subx[10] = subx[0]*subx[7] + subx[1]*subx[6] + subx[2]*subx[5] + subx[9];
    subx[11] = cov[5]*subx[4] + cov[6]*subx[0] - cov[7]*subx[1];
    subx[12] = -cov[10]*subx[1] + cov[6]*subx[4] + cov[9]*subx[0];
    subx[13] = cov[10]*subx[0] - cov[12]*subx[1] + cov[7]*subx[4];
    subx[14] = subx[0]*subx[12] + subx[11]*subx[4] - subx[13]*subx[1] + subx[9];
    subx[15] = 1.0/(subx[10]*subx[14] - ((subx[8])*(subx[8])));
    subx[16] = subx[14]*subx[15];
    subx[17] = cov[1]*subx[4] + cov[2]*subx[0] - cov[3]*subx[1];
    subx[18] = subx[15]*subx[8];
    subx[19] = subx[16]*subx[3] - subx[17]*subx[18];
    subx[20] = i_beta_m + subx[4];
    subx[21] = subx[10]*subx[15];
    subx[22] = subx[17]*subx[21] - subx[18]*subx[3];
    subx[23] = i_alpha_m - state[2]*subx[0] + state[3]*subx[1];
    subx[24] = -subx[11]*subx[18] + subx[16]*subx[5];
    subx[25] = subx[11]*subx[21] - subx[18]*subx[5];
    subx[26] = -subx[12]*subx[18] + subx[16]*subx[6];
    subx[27] = subx[12]*subx[21] - subx[18]*subx[6];
    subx[28] = subx[13]*subx[21] - subx[18]*subx[7];
    subx[29] = -subx[13]*subx[18] + subx[16]*subx[7];
    subx[30] = cov[11]*subx[0] - cov[13]*subx[1] + cov[8]*subx[4];
    subx[31] = cov[11]*subx[1] + cov[13]*subx[0] + cov[8]*subx[2];
    subx[32] = -subx[18]*subx[31] + subx[21]*subx[30];
    subx[33] = subx[16]*subx[31] - subx[18]*subx[30];
    subx[34] = -subx[19]*subx[2] - subx[22]*subx[4];
    subx[35] = -subx[0]*subx[22] - subx[19]*subx[1];
    subx[36] = -subx[0]*subx[19] + subx[1]*subx[22];
    subx[37] = -subx[24]*subx[2] - subx[25]*subx[4] + 1;
    subx[38] = -subx[0]*subx[25] - subx[1]*subx[24];
    subx[39] = -subx[0]*subx[24] + subx[1]*subx[25];
    subx[40] = -subx[0]*subx[26] + subx[1]*subx[27];
    subx[41] = -subx[26]*subx[2] - subx[27]*subx[4];
    subx[42] = -subx[0]*subx[27] - subx[1]*subx[26] + 1;
    state_n[0] = state[0] + subx[19]*subx[20] + subx[22]*subx[23];
    state_n[1] = state[1] + subx[20]*subx[24] + subx[23]*subx[25];
    state_n[2] = state[2] + subx[20]*subx[26] + subx[23]*subx[27];
    state_n[3] = state[3] + subx[20]*subx[29] + subx[23]*subx[28];
    state_n[4] = state[4] + subx[20]*subx[33] + subx[23]*subx[32];
    cov_n[0] = cov[0] + cov[1]*subx[34] + cov[2]*subx[35] + cov[3]*subx[36];
    cov_n[1] = cov[1] + cov[5]*subx[34] + cov[6]*subx[35] + cov[7]*subx[36];
    cov_n[2] = cov[10]*subx[36] + cov[2] + cov[6]*subx[34] + cov[9]*subx[35];
    cov_n[3] = cov[10]*subx[35] + cov[12]*subx[36] + cov[3] + cov[7]*subx[34];
    cov_n[4] = cov[11]*subx[35] + cov[13]*subx[36] + cov[4] + cov[8]*subx[34];
    cov_n[5] = cov[5]*subx[37] + cov[6]*subx[38] + cov[7]*subx[39];
    cov_n[6] = cov[10]*subx[39] + cov[6]*subx[37] + cov[9]*subx[38];
    cov_n[7] = cov[10]*subx[38] + cov[12]*subx[39] + cov[7]*subx[37];
    cov_n[8] = cov[11]*subx[38] + cov[13]*subx[39] + cov[8]*subx[37];
    cov_n[9] = cov[10]*subx[40] + cov[6]*subx[41] + cov[9]*subx[42];
    cov_n[10] = cov[10]*subx[42] + cov[12]*subx[40] + cov[7]*subx[41];
    cov_n[11] = cov[11]*subx[42] + cov[13]*subx[40] + cov[8]*subx[41];
    cov_n[12] = cov[10]*(-subx[0]*subx[28] - subx[1]*subx[29]) + cov[12]*(-subx[0]*subx[29] + subx[1]*subx[28] + 1) + cov[7]*(-subx[28]*subx[4] - subx[29]*subx[2]);
    cov_n[13] = cov[11]*(-subx[0]*subx[28] - subx[1]*subx[29]) + cov[13]*(-subx[0]*subx[29] + subx[1]*subx[28] + 1) + cov[8]*(-subx[28]*subx[4] - subx[29]*subx[2]);
    cov_n[14] = cov[11]*(-subx[0]*subx[32] - subx[1]*subx[33]) + cov[13]*(-subx[0]*subx[33] + subx[1]*subx[32]) + cov[14] + cov[8]*(-subx[2]*subx[33] - subx[32]*subx[4]);
    innov[0] = i_alpha_m - state[2]*cosf_fast(state[1]) + state[3]*sinf_fast(state[1]);
    innov[1] = i_beta_m - state[2]*sinf_fast(state[1]) - state[3]*cosf_fast(state[1]);
    *NIS = subx[20]*(subx[16]*subx[20] - subx[18]*subx[23]) + subx[23]*(-subx[18]*subx[20] + subx[21]*subx[23]);

    state_n[1] = wrap_2pi(state_n[1]);
    ekf_idx = next_ekf_idx;
}
