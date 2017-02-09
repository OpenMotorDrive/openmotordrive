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

static void ekf_update(float dt, float u_alpha, float u_beta, float i_alpha_m, float i_beta_m) {
    uint8_t next_ekf_idx = (ekf_idx+1)%2;
    float* state = ekf_state[ekf_idx].x;
    float* cov = ekf_state[ekf_idx].P;
    float* state_n = ekf_state[next_ekf_idx].x;
    float* cov_n = ekf_state[next_ekf_idx].P;
    float* innov = ekf_state[next_ekf_idx].innov;
    float* NIS = &ekf_state[next_ekf_idx].NIS;

    // 590 operations
    static float subx[80];
    subx[0] = cosf_fast(state[1]);
    subx[1] = sinf_fast(state[1]);
    subx[2] = 1.0/L_d;
    subx[3] = dt*subx[2]*(L_q*N_P*state[0]*state[3] - R_s*state[2] + subx[0]*u_alpha + subx[1]*u_beta) + state[2];
    subx[4] = N_P*dt;
    subx[5] = cosf_fast(state[0]*subx[4] + state[1]);
    subx[6] = state[3] + dt*(-L_d*N_P*state[0]*state[2] - R_s*state[3] + subx[0]*u_beta - subx[1]*u_alpha - 20.0*state[0]/(M_PI*K_v))/L_q;
    subx[7] = sinf_fast(state[0]*subx[4] + state[1]);
    subx[8] = subx[3]*subx[5] - subx[6]*subx[7];
    subx[9] = dt*(state[2]*(L_d - L_q) + 30.0/(M_PI*J*K_v));
    subx[10] = cov[4]*subx[4] + cov[8];
    subx[11] = dt/J;
    subx[12] = dt*state[3]*(L_d - L_q);
    subx[13] = cov[0]*subx[4] + cov[1] - subx[10]*subx[11] + subx[12]*(cov[2]*subx[4] + cov[6]) + subx[9]*(cov[3]*subx[4] + cov[7]);
    subx[14] = -R_s*dt*subx[2] + 1;
    subx[15] = dt*subx[2]*(subx[0]*u_beta - subx[1]*u_alpha);
    subx[16] = L_q*N_P*dt*state[0]*subx[2];
    subx[17] = L_q*state[3]*subx[2];
    subx[18] = cov[10]*subx[14] + cov[12]*subx[16] + cov[3]*subx[17]*subx[4] + cov[7]*subx[15];
    subx[19] = cov[11]*subx[14] + cov[13]*subx[16] + cov[4]*subx[17]*subx[4] + cov[8]*subx[15];
    subx[20] = cov[10]*subx[16] + cov[2]*subx[17]*subx[4] + cov[6]*subx[15] + cov[9]*subx[14];
    subx[21] = L_q*cov[3]*state[0]*subx[2]*subx[4] + cov[0]*subx[17]*subx[4] + cov[1]*subx[15] + cov[2]*subx[14];
    subx[22] = -subx[11]*subx[19] + subx[12]*subx[20] + subx[18]*subx[9] + subx[21];
    subx[23] = 1 - R_s*dt/L_q;
    subx[24] = dt*(-L_d*N_P*state[2] - 20.0/(M_PI*K_v))/L_q;
    subx[25] = dt*(-subx[0]*u_alpha - subx[1]*u_beta)/L_q;
    subx[26] = L_d*N_P*dt*state[0]/L_q;
    subx[27] = -cov[10]*subx[26] + cov[12]*subx[23] + cov[3]*subx[24] + cov[7]*subx[25];
    subx[28] = -cov[11]*subx[26] + cov[13]*subx[23] + cov[4]*subx[24] + cov[8]*subx[25];
    subx[29] = cov[10]*subx[23] + cov[2]*subx[24] + cov[6]*subx[25] - cov[9]*subx[26];
    subx[30] = cov[0]*subx[24] + cov[1]*subx[25] - cov[2]*subx[26] + cov[3]*subx[23];
    subx[31] = -subx[11]*subx[28] + subx[12]*subx[29] + subx[27]*subx[9] + subx[30];
    subx[32] = subx[13]*subx[8] + subx[22]*subx[7] + subx[31]*subx[5];
    subx[33] = -subx[3]*subx[7] - subx[5]*subx[6];
    subx[34] = cov[1]*subx[4] + cov[5] + subx[4]*(cov[0]*subx[4] + cov[1]) + ((theta_pnoise)*(theta_pnoise));
    subx[35] = cov[1]*subx[17]*subx[4] + cov[5]*subx[15] + cov[6]*subx[14] + cov[7]*subx[16];
    subx[36] = subx[21]*subx[4] + subx[35];
    subx[37] = cov[1]*subx[24] + cov[5]*subx[25] - cov[6]*subx[26] + cov[7]*subx[23];
    subx[38] = subx[30]*subx[4] + subx[37];
    subx[39] = subx[34]*subx[8] + subx[36]*subx[7] + subx[38]*subx[5];
    subx[40] = subx[14]*subx[29] + subx[15]*subx[37] + subx[16]*subx[27] + subx[17]*subx[30]*subx[4];
    subx[41] = subx[14]*subx[20] + subx[15]*subx[35] + subx[16]*subx[18] + subx[17]*subx[21]*subx[4] + ((dt)*(dt))*((subx[0])*(subx[0]))*((u_noise)*(u_noise))/((L_d)*(L_d)) + ((dt)*(dt))*((subx[1])*(subx[1]))*((u_noise)*(u_noise))/((L_d)*(L_d));
    subx[42] = subx[36]*subx[8] + subx[40]*subx[5] + subx[41]*subx[7];
    subx[43] = subx[23]*subx[27] + subx[24]*subx[30] + subx[25]*subx[37] - subx[26]*subx[29] + ((dt)*(dt))*((subx[0])*(subx[0]))*((u_noise)*(u_noise))/((L_q)*(L_q)) + ((dt)*(dt))*((subx[1])*(subx[1]))*((u_noise)*(u_noise))/((L_q)*(L_q));
    subx[44] = subx[38]*subx[8] + subx[40]*subx[7] + subx[43]*subx[5];
    subx[45] = subx[33]*subx[39] + subx[42]*subx[5] - subx[44]*subx[7];
    subx[46] = ((i_noise)*(i_noise)) + subx[39]*subx[8] + subx[42]*subx[7] + subx[44]*subx[5];
    subx[47] = subx[33]*subx[34] + subx[36]*subx[5] - subx[38]*subx[7];
    subx[48] = subx[33]*subx[36] - subx[40]*subx[7] + subx[41]*subx[5];
    subx[49] = subx[33]*subx[38] + subx[40]*subx[5] - subx[43]*subx[7];
    subx[50] = ((i_noise)*(i_noise)) + subx[33]*subx[47] + subx[48]*subx[5] - subx[49]*subx[7];
    subx[51] = 1.0/(-((subx[45])*(subx[45])) + subx[46]*subx[50]);
    subx[52] = subx[50]*subx[51];
    subx[53] = subx[13]*subx[33] + subx[22]*subx[5] - subx[31]*subx[7];
    subx[54] = subx[45]*subx[51];
    subx[55] = subx[32]*subx[52] - subx[53]*subx[54];
    subx[56] = i_beta_m + subx[33];
    subx[57] = subx[46]*subx[51];
    subx[58] = -subx[32]*subx[54] + subx[53]*subx[57];
    subx[59] = i_alpha_m - subx[3]*subx[5] + subx[6]*subx[7];
    subx[60] = subx[39]*subx[52] - subx[47]*subx[54];
    subx[61] = -subx[39]*subx[54] + subx[47]*subx[57];
    subx[62] = subx[42]*subx[52] - subx[48]*subx[54];
    subx[63] = -subx[42]*subx[54] + subx[48]*subx[57];
    subx[64] = subx[44]*subx[52] - subx[49]*subx[54];
    subx[65] = -subx[44]*subx[54] + subx[49]*subx[57];
    subx[66] = subx[10]*subx[8] + subx[19]*subx[7] + subx[28]*subx[5];
    subx[67] = subx[10]*subx[33] + subx[19]*subx[5] - subx[28]*subx[7];
    subx[68] = subx[52]*subx[66] - subx[54]*subx[67];
    subx[69] = -subx[54]*subx[66] + subx[57]*subx[67];
    subx[70] = -subx[55]*subx[5] + subx[58]*subx[7];
    subx[71] = -subx[33]*subx[58] - subx[55]*subx[8];
    subx[72] = -subx[55]*subx[7] - subx[58]*subx[5];
    subx[73] = cov[11]*subx[12] + cov[13]*subx[9] - cov[14]*subx[11] + cov[4];
    subx[74] = -subx[5]*subx[60] + subx[61]*subx[7];
    subx[75] = -subx[5]*subx[61] - subx[60]*subx[7];
    subx[76] = -subx[33]*subx[61] - subx[60]*subx[8] + 1;
    subx[77] = -subx[5]*subx[62] + subx[63]*subx[7];
    subx[78] = -subx[33]*subx[63] - subx[62]*subx[8];
    subx[79] = -subx[5]*subx[63] - subx[62]*subx[7] + 1;
    state_n[0] = dt*(state[2]*state[3]*(L_d - L_q) - state[4]/J + 30.0*state[3]/(M_PI*J*K_v)) + state[0] + subx[55]*subx[56] + subx[58]*subx[59];
    state_n[1] = state[0]*subx[4] + state[1] + subx[56]*subx[60] + subx[59]*subx[61];
    state_n[2] = subx[3] + subx[56]*subx[62] + subx[59]*subx[63];
    state_n[3] = subx[56]*subx[64] + subx[59]*subx[65] + subx[6];
    state_n[4] = state[4] + subx[56]*subx[68] + subx[59]*subx[69];
    cov_n[0] = cov[0] + cov[2]*subx[12] + cov[3]*subx[9] - cov[4]*subx[11] + ((omega_pnoise)*(omega_pnoise)) - subx[11]*subx[73] + subx[12]*(cov[10]*subx[9] - cov[11]*subx[11] + cov[2] + cov[9]*subx[12]) + subx[13]*subx[71] + subx[22]*subx[72] + subx[31]*subx[70] + subx[9]*(cov[10]*subx[12] + cov[12]*subx[9] - cov[13]*subx[11] + cov[3]);
    cov_n[1] = subx[13] + subx[34]*subx[71] + subx[36]*subx[72] + subx[38]*subx[70];
    cov_n[2] = subx[22] + subx[36]*subx[71] + subx[40]*subx[70] + subx[41]*subx[72];
    cov_n[3] = subx[31] + subx[38]*subx[71] + subx[40]*subx[72] + subx[43]*subx[70];
    cov_n[4] = subx[10]*subx[71] + subx[19]*subx[72] + subx[28]*subx[70] + subx[73];
    cov_n[5] = subx[34]*subx[76] + subx[36]*subx[75] + subx[38]*subx[74];
    cov_n[6] = subx[36]*subx[76] + subx[40]*subx[74] + subx[41]*subx[75];
    cov_n[7] = subx[38]*subx[76] + subx[40]*subx[75] + subx[43]*subx[74];
    cov_n[8] = subx[10]*subx[76] + subx[19]*subx[75] + subx[28]*subx[74];
    cov_n[9] = subx[36]*subx[78] + subx[40]*subx[77] + subx[41]*subx[79];
    cov_n[10] = subx[38]*subx[78] + subx[40]*subx[79] + subx[43]*subx[77];
    cov_n[11] = subx[10]*subx[78] + subx[19]*subx[79] + subx[28]*subx[77];
    cov_n[12] = subx[38]*(-subx[33]*subx[65] - subx[64]*subx[8]) + subx[40]*(-subx[5]*subx[65] - subx[64]*subx[7]) + subx[43]*(-subx[5]*subx[64] + subx[65]*subx[7] + 1);
    cov_n[13] = subx[10]*(-subx[33]*subx[65] - subx[64]*subx[8]) + subx[19]*(-subx[5]*subx[65] - subx[64]*subx[7]) + subx[28]*(-subx[5]*subx[64] + subx[65]*subx[7] + 1);
    cov_n[14] = ((T_l_pnoise)*(T_l_pnoise)) + cov[14] + subx[10]*(-subx[33]*subx[69] - subx[68]*subx[8]) + subx[19]*(-subx[5]*subx[69] - subx[68]*subx[7]) + subx[28]*(-subx[5]*subx[68] + subx[69]*subx[7]);
    innov[0] = i_alpha_m - state[2]*subx[0] + state[3]*subx[1];
    innov[1] = i_beta_m - state[2]*subx[1] - state[3]*subx[0];
    *NIS = subx[56]*(subx[52]*subx[56] - subx[54]*subx[59]) + subx[59]*(-subx[54]*subx[56] + subx[57]*subx[59]);

    state_n[1] = wrap_2pi(state_n[1]);
    ekf_idx = next_ekf_idx;
}
