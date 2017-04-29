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

static FTYPE subx[57];
static void ekf_predict(FTYPE dt, FTYPE i_alpha_m, FTYPE i_beta_m, FTYPE u_alpha, FTYPE u_beta) {
    uint8_t next_ekf_idx = (ekf_idx+1)%2;
    FTYPE* state = ekf_state[ekf_idx].x;
    FTYPE* cov = ekf_state[ekf_idx].P;
    FTYPE* state_n = ekf_state[next_ekf_idx].x;
    FTYPE* cov_n = ekf_state[next_ekf_idx].P;
    // 425 operations
    subx[0] = 1.0/L_d;
    subx[1] = cosf_fast(state[1]);
    subx[2] = sinf_fast(state[1]);
    subx[3] = dt*subx[0]*(L_q*N_P*state[0]*state[3] - R_s*state[2] + subx[1]*u_alpha + subx[2]*u_beta);
    subx[4] = 1.0/L_q;
    subx[5] = dt*subx[4]*(-L_d*N_P*state[0]*state[2] - N_P*lambda_r*state[0] - R_s*state[3] + subx[1]*u_beta - subx[2]*u_alpha);
    subx[6] = 1.0/J;
    subx[7] = dt*subx[6];
    subx[8] = (3.0/2.0)*lambda_r + (3.0/2.0)*state[2]*(L_d - L_q);
    subx[9] = dt*subx[6]*(N_P*state[3]*subx[8] - state[4]);
    subx[10] = N_P*dt;
    subx[11] = 2*N_P*dt*state[0] + state[1];
    subx[12] = ((dt)*(dt));
    subx[13] = N_P*dt*subx[6]*subx[8];
    subx[14] = (3.0/2.0)*N_P*dt*state[3]*subx[6]*(L_d - L_q);
    subx[15] = cov[11]*subx[14] + cov[13]*subx[13] - cov[14]*subx[7] + cov[4];
    subx[16] = cov[10]*subx[14] + cov[12]*subx[13] - cov[13]*subx[7] + cov[3];
    subx[17] = cov[10]*subx[13] - cov[11]*subx[7] + cov[2] + cov[9]*subx[14];
    subx[18] = N_P*cov[3]*dt*subx[6]*subx[8] + cov[0] + (3.0/2.0)*cov[2]*state[3]*subx[10]*subx[6]*(L_d - L_q) - cov[4]*subx[7];
    subx[19] = cov[1] + cov[6]*subx[14] + cov[7]*subx[13] - cov[8]*subx[7];
    subx[20] = -R_s*dt*subx[0] + 1;
    subx[21] = dt*subx[0]*(subx[1]*u_beta - subx[2]*u_alpha);
    subx[22] = L_q*N_P*dt*state[0]*subx[0];
    subx[23] = L_q*state[3]*subx[0];
    subx[24] = -R_s*dt*subx[4] + 1;
    subx[25] = dt*subx[4]*(-L_d*N_P*state[2] - N_P*lambda_r);
    subx[26] = dt*subx[4]*(-subx[1]*u_alpha - subx[2]*u_beta);
    subx[27] = L_d*N_P*dt*state[0]*subx[4];
    subx[28] = cov[10]*subx[22] + cov[2]*subx[10]*subx[23] + cov[6]*subx[21] + cov[9]*subx[20];
    subx[29] = cov[1]*subx[10]*subx[23] + cov[5]*subx[21] + cov[6]*subx[20] + cov[7]*subx[22];
    subx[30] = cov[10]*subx[20] + cov[12]*subx[22] + cov[3]*subx[10]*subx[23] + cov[7]*subx[21];
    subx[31] = L_q*N_P*cov[3]*dt*state[0]*subx[0] + cov[0]*subx[10]*subx[23] + cov[1]*subx[21] + cov[2]*subx[20];
    state_n[0] = state[0] + subx[7]*((1.0/4.0)*N_P*(lambda_r + (L_d - L_q)*(state[2] + 2*subx[3]))*(state[3] + 2*subx[5]) - 1.0/6.0*state[4]) + (5.0/6.0)*subx[9];
    state_n[1] = (5.0/6.0)*N_P*dt*state[0] + state[1] + subx[10]*((1.0/3.0)*dt*subx[6]*(N_P*state[3]*subx[8] - state[4]) + (1.0/6.0)*state[0]);
    state_n[2] = dt*subx[0]*((1.0/6.0)*L_q*N_P*(state[0] + 2*subx[9])*(state[3] + 2*subx[5]) - 1.0/6.0*R_s*(state[2] + 2*subx[3]) + (1.0/6.0)*u_alpha*cosf_fast(subx[11]) + (1.0/6.0)*u_beta*sinf_fast(subx[11])) + state[2] + (5.0/6.0)*subx[3];
    state_n[3] = dt*subx[4]*(-1.0/6.0*L_d*N_P*(state[0] + 2*subx[9])*(state[2] + 2*subx[3]) - 1.0/6.0*N_P*lambda_r*(state[0] + 2*subx[9]) - 1.0/6.0*R_s*(state[3] + 2*subx[5]) - 1.0/6.0*u_alpha*sinf_fast(subx[11]) + (1.0/6.0)*u_beta*cosf_fast(subx[11])) + state[3] + (5.0/6.0)*subx[5];
    state_n[4] = state[4];
    cov_n[0] = ((omega_pnoise)*(omega_pnoise))*subx[12] + subx[13]*subx[16] + subx[14]*subx[17] - subx[15]*subx[7] + subx[18];
    cov_n[1] = subx[10]*subx[18] + subx[19];
    cov_n[2] = subx[10]*subx[18]*subx[23] + subx[16]*subx[22] + subx[17]*subx[20] + subx[19]*subx[21];
    cov_n[3] = subx[16]*subx[24] - subx[17]*subx[27] + subx[18]*subx[25] + subx[19]*subx[26];
    cov_n[4] = subx[15];
    cov_n[5] = cov[1]*subx[10] + cov[5] + subx[10]*(cov[0]*subx[10] + cov[1]);
    cov_n[6] = subx[10]*subx[23]*(cov[0]*subx[10] + cov[1]) + subx[20]*(cov[2]*subx[10] + cov[6]) + subx[21]*(cov[1]*subx[10] + cov[5]) + subx[22]*(cov[3]*subx[10] + cov[7]);
    cov_n[7] = subx[24]*(cov[3]*subx[10] + cov[7]) + subx[25]*(cov[0]*subx[10] + cov[1]) + subx[26]*(cov[1]*subx[10] + cov[5]) - subx[27]*(cov[2]*subx[10] + cov[6]);
    cov_n[8] = cov[4]*subx[10] + cov[8];
    cov_n[9] = N_P*dt*subx[23]*subx[31] + subx[20]*subx[28] + subx[21]*subx[29] + subx[22]*subx[30] + subx[12]*((subx[1])*(subx[1]))*((u_noise)*(u_noise))/((L_d)*(L_d)) + subx[12]*((subx[2])*(subx[2]))*((u_noise)*(u_noise))/((L_d)*(L_d));
    cov_n[10] = subx[24]*subx[30] + subx[25]*subx[31] + subx[26]*subx[29] - subx[27]*subx[28];
    cov_n[11] = cov[11]*subx[20] + cov[13]*subx[22] + cov[4]*subx[10]*subx[23] + cov[8]*subx[21];
    cov_n[12] = subx[24]*(-cov[10]*subx[27] + cov[12]*subx[24] + cov[3]*subx[25] + cov[7]*subx[26]) + subx[25]*(-L_d*cov[2]*state[0]*subx[10]*subx[4] + cov[0]*subx[25] + cov[1]*subx[26] + cov[3]*subx[24]) + subx[26]*(cov[1]*subx[25] + cov[5]*subx[26] - cov[6]*subx[27] + cov[7]*subx[24]) - subx[27]*(cov[10]*subx[24] + cov[2]*subx[25] + cov[6]*subx[26] - cov[9]*subx[27]) + subx[12]*((subx[1])*(subx[1]))*((u_noise)*(u_noise))/((L_q)*(L_q)) + subx[12]*((subx[2])*(subx[2]))*((u_noise)*(u_noise))/((L_q)*(L_q));
    cov_n[13] = -cov[11]*subx[27] + cov[13]*subx[24] + cov[4]*subx[25] + cov[8]*subx[26];
    cov_n[14] = ((T_l_pnoise)*(T_l_pnoise))*subx[12] + cov[14];

    ekf_state[next_ekf_idx].NIS = ekf_state[ekf_idx].NIS;
    state_n[1] = wrap_2pi(state_n[1]);
    ekf_idx = next_ekf_idx;
}

static void ekf_update(FTYPE dt, FTYPE i_alpha_m, FTYPE i_beta_m, FTYPE u_alpha, FTYPE u_beta) {
    uint8_t next_ekf_idx = (ekf_idx+1)%2;
    FTYPE* state = ekf_state[ekf_idx].x;
    FTYPE* cov = ekf_state[ekf_idx].P;
    FTYPE* state_n = ekf_state[next_ekf_idx].x;
    FTYPE* cov_n = ekf_state[next_ekf_idx].P;
    FTYPE* innov = ekf_state[next_ekf_idx].innov;
    FTYPE* NIS = &ekf_state[next_ekf_idx].NIS;

    // 496 operations
    subx[0] = cosf_fast(state[1]);
    subx[1] = 1.0/L_q;
    subx[2] = 1.0/L_d;
    subx[3] = sinf_fast(state[1]);
    subx[4] = L_q*N_P*param2*state[3]*subx[2]*subx[3] + param2*subx[0]*subx[1]*(-L_d*N_P*state[2] - N_P*lambda_r);
    subx[5] = param2*subx[2]*(L_q*N_P*state[0]*state[3] - R_s*state[2] + subx[0]*u_alpha + subx[3]*u_beta) + state[2];
    subx[6] = subx[0]*u_beta - subx[3]*u_alpha;
    subx[7] = param2*subx[1]*(-L_d*N_P*state[0]*state[2] - N_P*lambda_r*state[0] - R_s*state[3] + subx[6]) + state[3];
    subx[8] = param2*subx[0]*subx[1]*(-subx[0]*u_alpha - subx[3]*u_beta) + param2*subx[2]*subx[3]*subx[6] + subx[0]*subx[5] - subx[3]*subx[7];
    subx[9] = -L_d*N_P*param2*state[0]*subx[0]*subx[1] + subx[3]*(-R_s*param2*subx[2] + 1);
    subx[10] = L_q*N_P*param2*state[0]*subx[2]*subx[3] + subx[0]*(-R_s*param2*subx[1] + 1);
    subx[11] = cov[0]*subx[4] + cov[1]*subx[8] + cov[2]*subx[9] + cov[3]*subx[10];
    subx[12] = L_d*N_P*param2*state[0]*subx[1]*subx[3] + subx[0]*(-R_s*param2*subx[2] + 1);
    subx[13] = cov[10]*subx[10] + cov[2]*subx[4] + cov[6]*subx[8] + cov[9]*subx[9];
    subx[14] = L_q*N_P*param2*state[0]*subx[0]*subx[2] - subx[3]*(-R_s*param2*subx[1] + 1);
    subx[15] = cov[10]*subx[9] + cov[12]*subx[10] + cov[3]*subx[4] + cov[7]*subx[8];
    subx[16] = L_q*N_P*param2*state[3]*subx[0]*subx[2] - param2*subx[1]*subx[3]*(-L_d*N_P*state[2] - N_P*lambda_r);
    subx[17] = cov[1]*subx[4] + cov[5]*subx[8] + cov[6]*subx[9] + cov[7]*subx[10];
    subx[18] = param2*subx[0]*subx[2]*subx[6] - param2*subx[1]*subx[3]*(-subx[0]*u_alpha - subx[3]*u_beta) - subx[0]*subx[7] - subx[3]*subx[5];
    subx[19] = subx[11]*subx[16] + subx[12]*subx[13] + subx[14]*subx[15] + subx[17]*subx[18];
    subx[20] = ((i_noise + param1*sqrtf(((i_alpha_m)*(i_alpha_m)) + ((i_beta_m)*(i_beta_m))))*(i_noise + param1*sqrtf(((i_alpha_m)*(i_alpha_m)) + ((i_beta_m)*(i_beta_m)))));
    subx[21] = cov[10]*subx[14] + cov[2]*subx[16] + cov[6]*subx[18] + cov[9]*subx[12];
    subx[22] = cov[10]*subx[12] + cov[12]*subx[14] + cov[3]*subx[16] + cov[7]*subx[18];
    subx[23] = cov[0]*subx[16] + cov[1]*subx[18] + cov[2]*subx[12] + cov[3]*subx[14];
    subx[24] = cov[1]*subx[16] + cov[5]*subx[18] + cov[6]*subx[12] + cov[7]*subx[14];
    subx[25] = subx[12]*subx[21] + subx[14]*subx[22] + subx[16]*subx[23] + subx[18]*subx[24] + subx[20];
    subx[26] = subx[10]*subx[15] + subx[11]*subx[4] + subx[13]*subx[9] + subx[17]*subx[8] + subx[20];
    subx[27] = 1.0/(-((subx[19])*(subx[19])) + subx[25]*subx[26]);
    subx[28] = subx[25]*subx[27];
    subx[29] = subx[19]*subx[27];
    subx[30] = subx[11]*subx[28] - subx[23]*subx[29];
    subx[31] = i_beta_m - subx[0]*subx[7] - subx[3]*subx[5];
    subx[32] = subx[26]*subx[27];
    subx[33] = -subx[11]*subx[29] + subx[23]*subx[32];
    subx[34] = i_alpha_m - subx[0]*subx[5] + subx[3]*subx[7];
    subx[35] = subx[17]*subx[28] - subx[24]*subx[29];
    subx[36] = -subx[17]*subx[29] + subx[24]*subx[32];
    subx[37] = subx[13]*subx[28] - subx[21]*subx[29];
    subx[38] = -subx[13]*subx[29] + subx[21]*subx[32];
    subx[39] = -subx[15]*subx[29] + subx[22]*subx[32];
    subx[40] = subx[15]*subx[28] - subx[22]*subx[29];
    subx[41] = cov[11]*subx[12] + cov[13]*subx[14] + cov[4]*subx[16] + cov[8]*subx[18];
    subx[42] = cov[11]*subx[9] + cov[13]*subx[10] + cov[4]*subx[4] + cov[8]*subx[8];
    subx[43] = -subx[29]*subx[42] + subx[32]*subx[41];
    subx[44] = subx[28]*subx[42] - subx[29]*subx[41];
    subx[45] = -subx[16]*subx[33] - subx[30]*subx[4] + 1;
    subx[46] = -subx[18]*subx[33] - subx[30]*subx[8];
    subx[47] = -subx[12]*subx[33] - subx[30]*subx[9];
    subx[48] = -subx[10]*subx[30] - subx[14]*subx[33];
    subx[49] = -subx[16]*subx[36] - subx[35]*subx[4];
    subx[50] = -subx[18]*subx[36] - subx[35]*subx[8] + 1;
    subx[51] = -subx[12]*subx[36] - subx[35]*subx[9];
    subx[52] = -subx[10]*subx[35] - subx[14]*subx[36];
    subx[53] = -subx[10]*subx[37] - subx[14]*subx[38];
    subx[54] = -subx[16]*subx[38] - subx[37]*subx[4];
    subx[55] = -subx[18]*subx[38] - subx[37]*subx[8];
    subx[56] = -subx[12]*subx[38] - subx[37]*subx[9] + 1;
    state_n[0] = state[0] + subx[30]*subx[31] + subx[33]*subx[34];
    state_n[1] = state[1] + subx[31]*subx[35] + subx[34]*subx[36];
    state_n[2] = state[2] + subx[31]*subx[37] + subx[34]*subx[38];
    state_n[3] = state[3] + subx[31]*subx[40] + subx[34]*subx[39];
    state_n[4] = state[4] + subx[31]*subx[44] + subx[34]*subx[43];
    cov_n[0] = cov[0]*subx[45] + cov[1]*subx[46] + cov[2]*subx[47] + cov[3]*subx[48];
    cov_n[1] = cov[1]*subx[45] + cov[5]*subx[46] + cov[6]*subx[47] + cov[7]*subx[48];
    cov_n[2] = cov[10]*subx[48] + cov[2]*subx[45] + cov[6]*subx[46] + cov[9]*subx[47];
    cov_n[3] = cov[10]*subx[47] + cov[12]*subx[48] + cov[3]*subx[45] + cov[7]*subx[46];
    cov_n[4] = cov[11]*subx[47] + cov[13]*subx[48] + cov[4]*subx[45] + cov[8]*subx[46];
    cov_n[5] = cov[1]*subx[49] + cov[5]*subx[50] + cov[6]*subx[51] + cov[7]*subx[52];
    cov_n[6] = cov[10]*subx[52] + cov[2]*subx[49] + cov[6]*subx[50] + cov[9]*subx[51];
    cov_n[7] = cov[10]*subx[51] + cov[12]*subx[52] + cov[3]*subx[49] + cov[7]*subx[50];
    cov_n[8] = cov[11]*subx[51] + cov[13]*subx[52] + cov[4]*subx[49] + cov[8]*subx[50];
    cov_n[9] = cov[10]*subx[53] + cov[2]*subx[54] + cov[6]*subx[55] + cov[9]*subx[56];
    cov_n[10] = cov[10]*subx[56] + cov[12]*subx[53] + cov[3]*subx[54] + cov[7]*subx[55];
    cov_n[11] = cov[11]*subx[56] + cov[13]*subx[53] + cov[4]*subx[54] + cov[8]*subx[55];
    cov_n[12] = cov[10]*(-subx[12]*subx[39] - subx[40]*subx[9]) + cov[12]*(-subx[10]*subx[40] - subx[14]*subx[39] + 1) + cov[3]*(-subx[16]*subx[39] - subx[40]*subx[4]) + cov[7]*(-subx[18]*subx[39] - subx[40]*subx[8]);
    cov_n[13] = cov[11]*(-subx[12]*subx[39] - subx[40]*subx[9]) + cov[13]*(-subx[10]*subx[40] - subx[14]*subx[39] + 1) + cov[4]*(-subx[16]*subx[39] - subx[40]*subx[4]) + cov[8]*(-subx[18]*subx[39] - subx[40]*subx[8]);
    cov_n[14] = cov[11]*(-subx[12]*subx[43] - subx[44]*subx[9]) + cov[13]*(-subx[10]*subx[44] - subx[14]*subx[43]) + cov[14] + cov[4]*(-subx[16]*subx[43] - subx[44]*subx[4]) + cov[8]*(-subx[18]*subx[43] - subx[44]*subx[8]);
    innov[0] = i_alpha_m - (state[2] + param2*(L_q*N_P*state[0]*state[3] - R_s*state[2] + u_alpha*cosf_fast(state[1]) + u_beta*sinf_fast(state[1]))/L_d)*cosf_fast(state[1]) + (state[3] + param2*(-L_d*N_P*state[0]*state[2] - N_P*lambda_r*state[0] - R_s*state[3] - u_alpha*sinf_fast(state[1]) + u_beta*cosf_fast(state[1]))/L_q)*sinf_fast(state[1]);
    innov[1] = i_beta_m - (state[2] + param2*(L_q*N_P*state[0]*state[3] - R_s*state[2] + u_alpha*cosf_fast(state[1]) + u_beta*sinf_fast(state[1]))/L_d)*sinf_fast(state[1]) - (state[3] + param2*(-L_d*N_P*state[0]*state[2] - N_P*lambda_r*state[0] - R_s*state[3] - u_alpha*sinf_fast(state[1]) + u_beta*cosf_fast(state[1]))/L_q)*cosf_fast(state[1]);
    *NIS = subx[31]*(subx[28]*subx[31] - subx[29]*subx[34]) + subx[34]*(-subx[29]*subx[31] + subx[32]*subx[34]);

    state_n[1] = wrap_2pi(state_n[1]);
    ekf_idx = next_ekf_idx;
}
