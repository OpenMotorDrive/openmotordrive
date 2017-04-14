#pragma once
#include <math.h>
#include "math_helpers.h"

#ifndef FTYPE
  #define FTYPE float
#endif

struct ekf_state_s {
    FTYPE x[5];
    FTYPE P[15];
    FTYPE innov[2];
    FTYPE NIS;
};

static struct ekf_state_s ekf_state[2];
static uint8_t ekf_idx = 0;
static FTYPE x_at_curr_meas[5];
static void ekf_init(FTYPE init_theta) {
    FTYPE* state = ekf_state[ekf_idx].x;
    FTYPE* cov = ekf_state[ekf_idx].P;
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

static FTYPE subx[42];
static void ekf_predict(FTYPE dt, FTYPE u_alpha, FTYPE u_beta) {
    uint8_t next_ekf_idx = (ekf_idx+1)%2;
    FTYPE* state = ekf_state[ekf_idx].x;
    FTYPE* cov = ekf_state[ekf_idx].P;
    FTYPE* state_n = ekf_state[next_ekf_idx].x;
    FTYPE* cov_n = ekf_state[next_ekf_idx].P;
    // 339 operations
    subx[0] = (3.0L/2.0L)*N_P*(lambda_r*state[3] + state[2]*state[3]*(L_d - L_q)) - state[4];
    subx[1] = dt/J;
    subx[2] = N_P*dt;
    subx[3] = cos(state[1]);
    subx[4] = sin(state[1]);
    subx[5] = L_q*N_P*state[0]*state[3] - R_s*state[2] + subx[3]*u_alpha + subx[4]*u_beta;
    subx[6] = 1.0/L_d;
    subx[7] = -L_d*N_P*state[0]*state[2] - N_P*lambda_r*state[0] - R_s*state[3] + subx[3]*u_beta - subx[4]*u_alpha;
    subx[8] = 1.0/L_q;
    subx[9] = (3.0L/2.0L)*N_P*cov[11]*state[3]*subx[1]*(L_d - L_q) + (3.0L/2.0L)*N_P*cov[13]*subx[1]*(lambda_r + state[2]*(L_d - L_q)) - cov[14]*subx[1] + cov[4];
    subx[10] = (3.0L/2.0L)*N_P*dt*(lambda_r + state[2]*(L_d - L_q))/J;
    subx[11] = (3.0L/2.0L)*N_P*dt*state[3]*(L_d - L_q)/J;
    subx[12] = cov[10]*subx[11] + cov[12]*subx[10] - cov[13]*subx[1] + cov[3];
    subx[13] = cov[10]*subx[10] - cov[11]*subx[1] + cov[2] + cov[9]*subx[11];
    subx[14] = cov[0] + cov[2]*subx[11] + cov[3]*subx[10] - cov[4]*subx[1];
    subx[15] = cov[1] + cov[6]*subx[11] + cov[7]*subx[10] - cov[8]*subx[1];
    subx[16] = -R_s*dt*subx[6] + 1;
    subx[17] = dt*subx[6]*(subx[3]*u_beta - subx[4]*u_alpha);
    subx[18] = L_q*N_P*dt*state[0]*subx[6];
    subx[19] = L_q*state[3]*subx[6];
    subx[20] = -R_s*dt*subx[8] + 1;
    subx[21] = dt*subx[8]*(-L_d*N_P*state[2] - N_P*lambda_r);
    subx[22] = dt*subx[8]*(-subx[3]*u_alpha - subx[4]*u_beta);
    subx[23] = L_d*N_P*dt*state[0]*subx[8];
    subx[24] = cov[10]*subx[18] + cov[2]*subx[19]*subx[2] + cov[6]*subx[17] + cov[9]*subx[16];
    subx[25] = cov[1]*subx[19]*subx[2] + cov[5]*subx[17] + cov[6]*subx[16] + cov[7]*subx[18];
    subx[26] = cov[10]*subx[16] + cov[12]*subx[18] + cov[3]*subx[19]*subx[2] + cov[7]*subx[17];
    subx[27] = L_q*cov[3]*state[0]*subx[2]*subx[6] + cov[0]*subx[19]*subx[2] + cov[1]*subx[17] + cov[2]*subx[16];
    state_n[0] = state[0] + subx[0]*subx[1];
    state_n[1] = state[0]*subx[2] + state[1];
    state_n[2] = dt*subx[5]*subx[6] + state[2];
    state_n[3] = dt*subx[7]*subx[8] + state[3];
    state_n[4] = state[4];
    x_at_curr_meas[0] = state[0] + subx[0]*(dt - i_delay)/J;
    x_at_curr_meas[1] = N_P*state[0]*(dt - i_delay) + state[1];
    x_at_curr_meas[2] = state[2] + subx[5]*subx[6]*(dt - i_delay);
    x_at_curr_meas[3] = state[3] + subx[7]*subx[8]*(dt - i_delay);
    x_at_curr_meas[4] = state[4];
    cov_n[0] = pow(omega_pnoise, 2) + subx[10]*subx[12] + subx[11]*subx[13] + subx[14] - subx[1]*subx[9];
    cov_n[1] = subx[14]*subx[2] + subx[15];
    cov_n[2] = subx[12]*subx[18] + subx[13]*subx[16] + subx[14]*subx[19]*subx[2] + subx[15]*subx[17];
    cov_n[3] = subx[12]*subx[20] - subx[13]*subx[23] + subx[14]*subx[21] + subx[15]*subx[22];
    cov_n[4] = subx[9];
    cov_n[5] = cov[1]*subx[2] + cov[5] + subx[2]*(cov[0]*subx[2] + cov[1]) + pow(theta_pnoise, 2);
    cov_n[6] = subx[16]*(cov[2]*subx[2] + cov[6]) + subx[17]*(cov[1]*subx[2] + cov[5]) + subx[18]*(cov[3]*subx[2] + cov[7]) + subx[19]*subx[2]*(cov[0]*subx[2] + cov[1]);
    cov_n[7] = subx[20]*(cov[3]*subx[2] + cov[7]) + subx[21]*(cov[0]*subx[2] + cov[1]) + subx[22]*(cov[1]*subx[2] + cov[5]) - subx[23]*(cov[2]*subx[2] + cov[6]);
    cov_n[8] = cov[4]*subx[2] + cov[8];
    cov_n[9] = N_P*dt*subx[19]*subx[27] + subx[16]*subx[24] + subx[17]*subx[25] + subx[18]*subx[26] + pow(dt, 2)*pow(subx[3], 2)*pow(u_noise, 2)/pow(L_d, 2) + pow(dt, 2)*pow(subx[4], 2)*pow(u_noise, 2)/pow(L_d, 2);
    cov_n[10] = subx[20]*subx[26] + subx[21]*subx[27] + subx[22]*subx[25] - subx[23]*subx[24];
    cov_n[11] = cov[11]*subx[16] + cov[13]*subx[18] + cov[4]*subx[19]*subx[2] + cov[8]*subx[17];
    cov_n[12] = subx[20]*(-cov[10]*subx[23] + cov[12]*subx[20] + cov[3]*subx[21] + cov[7]*subx[22]) + subx[21]*(cov[0]*subx[21] + cov[1]*subx[22] - cov[2]*subx[23] + cov[3]*subx[20]) + subx[22]*(cov[1]*subx[21] + cov[5]*subx[22] - cov[6]*subx[23] + cov[7]*subx[20]) - subx[23]*(cov[10]*subx[20] + cov[2]*subx[21] + cov[6]*subx[22] - cov[9]*subx[23]) + pow(dt, 2)*pow(subx[3], 2)*pow(u_noise, 2)/pow(L_q, 2) + pow(dt, 2)*pow(subx[4], 2)*pow(u_noise, 2)/pow(L_q, 2);
    cov_n[13] = -cov[11]*subx[23] + cov[13]*subx[20] + cov[4]*subx[21] + cov[8]*subx[22];
    cov_n[14] = pow(T_l_pnoise, 2) + cov[14];

    state_n[1] = wrap_2pi(state_n[1]);
    ekf_idx = next_ekf_idx;
}

static void ekf_update(FTYPE i_alpha_m, FTYPE i_beta_m) {
    uint8_t next_ekf_idx = (ekf_idx+1)%2;
    FTYPE* state = x_at_curr_meas;
    FTYPE* cov = ekf_state[ekf_idx].P;
    FTYPE* state_n = ekf_state[next_ekf_idx].x;
    FTYPE* cov_n = ekf_state[next_ekf_idx].P;
    FTYPE* innov = ekf_state[next_ekf_idx].innov;
    FTYPE* NIS = &ekf_state[next_ekf_idx].NIS;

    // 289 operations
    subx[0] = cos(state[1]);
    subx[1] = sin(state[1]);
    subx[2] = state[2]*subx[0] - state[3]*subx[1];
    subx[3] = cov[1]*subx[2] + cov[2]*subx[1] + cov[3]*subx[0];
    subx[4] = -state[2]*subx[1] - state[3]*subx[0];
    subx[5] = cov[5]*subx[2] + cov[6]*subx[1] + cov[7]*subx[0];
    subx[6] = cov[10]*subx[0] + cov[6]*subx[2] + cov[9]*subx[1];
    subx[7] = cov[10]*subx[1] + cov[12]*subx[0] + cov[7]*subx[2];
    subx[8] = subx[0]*subx[6] - subx[1]*subx[7] + subx[4]*subx[5];
    subx[9] = pow(i_noise, 2) + subx[0]*subx[7] + subx[1]*subx[6] + subx[2]*subx[5];
    subx[10] = cov[5]*subx[4] + cov[6]*subx[0] - cov[7]*subx[1];
    subx[11] = -cov[10]*subx[1] + cov[6]*subx[4] + cov[9]*subx[0];
    subx[12] = cov[10]*subx[0] - cov[12]*subx[1] + cov[7]*subx[4];
    subx[13] = pow(i_noise, 2) + subx[0]*subx[11] + subx[10]*subx[4] - subx[12]*subx[1];
    subx[14] = 1.0/(subx[13]*subx[9] - pow(subx[8], 2));
    subx[15] = subx[13]*subx[14];
    subx[16] = cov[1]*subx[4] + cov[2]*subx[0] - cov[3]*subx[1];
    subx[17] = subx[14]*subx[8];
    subx[18] = subx[15]*subx[3] - subx[16]*subx[17];
    subx[19] = i_beta_m + subx[4];
    subx[20] = subx[14]*subx[9];
    subx[21] = subx[16]*subx[20] - subx[17]*subx[3];
    subx[22] = i_alpha_m - state[2]*subx[0] + state[3]*subx[1];
    subx[23] = -subx[10]*subx[17] + subx[15]*subx[5];
    subx[24] = subx[10]*subx[20] - subx[17]*subx[5];
    subx[25] = -subx[11]*subx[17] + subx[15]*subx[6];
    subx[26] = subx[11]*subx[20] - subx[17]*subx[6];
    subx[27] = subx[12]*subx[20] - subx[17]*subx[7];
    subx[28] = -subx[12]*subx[17] + subx[15]*subx[7];
    subx[29] = cov[11]*subx[0] - cov[13]*subx[1] + cov[8]*subx[4];
    subx[30] = cov[11]*subx[1] + cov[13]*subx[0] + cov[8]*subx[2];
    subx[31] = -subx[17]*subx[30] + subx[20]*subx[29];
    subx[32] = subx[15]*subx[30] - subx[17]*subx[29];
    subx[33] = -subx[18]*subx[2] - subx[21]*subx[4];
    subx[34] = -subx[0]*subx[21] - subx[18]*subx[1];
    subx[35] = -subx[0]*subx[18] + subx[1]*subx[21];
    subx[36] = -subx[23]*subx[2] - subx[24]*subx[4] + 1;
    subx[37] = -subx[0]*subx[24] - subx[1]*subx[23];
    subx[38] = -subx[0]*subx[23] + subx[1]*subx[24];
    subx[39] = -subx[0]*subx[25] + subx[1]*subx[26];
    subx[40] = -subx[25]*subx[2] - subx[26]*subx[4];
    subx[41] = -subx[0]*subx[26] - subx[1]*subx[25] + 1;
    state_n[0] = state[0] + subx[18]*subx[19] + subx[21]*subx[22];
    state_n[1] = state[1] + subx[19]*subx[23] + subx[22]*subx[24];
    state_n[2] = state[2] + subx[19]*subx[25] + subx[22]*subx[26];
    state_n[3] = state[3] + subx[19]*subx[28] + subx[22]*subx[27];
    state_n[4] = state[4] + subx[19]*subx[32] + subx[22]*subx[31];
    cov_n[0] = cov[0] + cov[1]*subx[33] + cov[2]*subx[34] + cov[3]*subx[35];
    cov_n[1] = cov[1] + cov[5]*subx[33] + cov[6]*subx[34] + cov[7]*subx[35];
    cov_n[2] = cov[10]*subx[35] + cov[2] + cov[6]*subx[33] + cov[9]*subx[34];
    cov_n[3] = cov[10]*subx[34] + cov[12]*subx[35] + cov[3] + cov[7]*subx[33];
    cov_n[4] = cov[11]*subx[34] + cov[13]*subx[35] + cov[4] + cov[8]*subx[33];
    cov_n[5] = cov[5]*subx[36] + cov[6]*subx[37] + cov[7]*subx[38];
    cov_n[6] = cov[10]*subx[38] + cov[6]*subx[36] + cov[9]*subx[37];
    cov_n[7] = cov[10]*subx[37] + cov[12]*subx[38] + cov[7]*subx[36];
    cov_n[8] = cov[11]*subx[37] + cov[13]*subx[38] + cov[8]*subx[36];
    cov_n[9] = cov[10]*subx[39] + cov[6]*subx[40] + cov[9]*subx[41];
    cov_n[10] = cov[10]*subx[41] + cov[12]*subx[39] + cov[7]*subx[40];
    cov_n[11] = cov[11]*subx[41] + cov[13]*subx[39] + cov[8]*subx[40];
    cov_n[12] = cov[10]*(-subx[0]*subx[27] - subx[1]*subx[28]) + cov[12]*(-subx[0]*subx[28] + subx[1]*subx[27] + 1) + cov[7]*(-subx[27]*subx[4] - subx[28]*subx[2]);
    cov_n[13] = cov[11]*(-subx[0]*subx[27] - subx[1]*subx[28]) + cov[13]*(-subx[0]*subx[28] + subx[1]*subx[27] + 1) + cov[8]*(-subx[27]*subx[4] - subx[28]*subx[2]);
    cov_n[14] = cov[11]*(-subx[0]*subx[31] - subx[1]*subx[32]) + cov[13]*(-subx[0]*subx[32] + subx[1]*subx[31]) + cov[14] + cov[8]*(-subx[2]*subx[32] - subx[31]*subx[4]);
    innov[0] = i_alpha_m - state[2]*cos(state[1]) + state[3]*sin(state[1]);
    innov[1] = i_beta_m - state[2]*sin(state[1]) - state[3]*cos(state[1]);
    *NIS = subx[19]*(subx[15]*subx[19] - subx[17]*subx[22]) + subx[22]*(-subx[17]*subx[19] + subx[20]*subx[22]);

    state_n[1] = wrap_2pi(state_n[1]);
    ekf_idx = next_ekf_idx;
}
