#include "ekf.h"
#include <math.h>
#include <string.h>
#include <common/helpers.h>

struct ekf_state_s* ekf_get_state(struct ekf_obj_s* this) {
    return &this->state[this->state_idx];
}
void ekf_init(struct ekf_obj_s* this, struct ekf_params_s* params, float init_theta) {
    float* state = this->state[this->state_idx].x;
    float* cov = this->state[this->state_idx].P;
    memset(this, 0, sizeof(*this));
    memcpy(&this->params, params, sizeof(this->params));
    state[0] = 0.0;
    state[1] = init_theta;
    state[2] = 0.0;
    state[3] = 0.0;
    state[4] = 0.0;
    state[5] = 1.0/this->params.J;
    cov[0] = 0;
    cov[1] = 0;
    cov[2] = 0;
    cov[3] = 0;
    cov[4] = 0;
    cov[5] = 0;
    cov[6] = 9.86960440108936;
    cov[7] = 0;
    cov[8] = 0;
    cov[9] = 0;
    cov[10] = 0;
    cov[11] = 0;
    cov[12] = 0;
    cov[13] = 0;
    cov[14] = 0;
    cov[15] = 0;
    cov[16] = 0;
    cov[17] = 0;
    cov[18] = 0;
    cov[19] = 0;
    cov[20] = 0.25/((this->params.J)*(this->params.J));
}

static float subx[49];
void ekf_predict(struct ekf_obj_s* this, float dt, float u_alpha, float u_beta) {
    uint8_t next_state_idx = (this->state_idx+1)%2;
    float* state = this->state[this->state_idx].x;
    float* cov = this->state[this->state_idx].P;
    float* state_n = this->state[next_state_idx].x;
    float* cov_n = this->state[next_state_idx].P;
    // 474 operations
    subx[0] = sinf_fast(state[1]);
    subx[1] = cosf_fast(state[1]);
    subx[2] = 1.0/this->params.L_q;
    subx[3] = dt*subx[2]*(-2*state[0]*state[2]*this->params.L_d*this->params.N_P - 2*state[0]*this->params.N_P*this->params.lambda_m - 2*state[3]*this->params.R_s - 2*subx[0]*u_alpha + 2*subx[1]*u_beta) + state[3];
    subx[4] = 1.0/this->params.L_d;
    subx[5] = subx[4]*(state[0]*state[3]*this->params.L_q*this->params.N_P - state[2]*this->params.R_s + subx[0]*u_beta + subx[1]*u_alpha);
    subx[6] = 2*dt*subx[5] + state[2];
    subx[7] = this->params.L_d - this->params.L_q;
    subx[8] = dt*state[5];
    subx[9] = (3.0/2.0)*state[3]*this->params.N_P*(state[2]*subx[7] + this->params.lambda_m) - state[4];
    subx[10] = dt*state[5]*subx[9];
    subx[11] = dt*this->params.N_P;
    subx[12] = ((dt)*(dt));
    subx[13] = dt*subx[9];
    subx[14] = (3.0/2.0)*dt*state[5]*this->params.N_P*(state[2]*subx[7] + this->params.lambda_m);
    subx[15] = (3.0/2.0)*dt*state[3]*state[5]*subx[7]*this->params.N_P;
    subx[16] = cov[14]*subx[15] + cov[17]*subx[14] - cov[19]*subx[8] + cov[20]*subx[13] + cov[5];
    subx[17] = (3.0/2.0)*cov[13]*dt*state[3]*state[5]*subx[7]*this->params.N_P + (3.0/2.0)*cov[16]*subx[8]*this->params.N_P*(state[2]*subx[7] + this->params.lambda_m) - cov[18]*subx[8] + cov[19]*subx[13] + cov[4];
    subx[18] = cov[12]*subx[15] + cov[15]*subx[14] - cov[16]*subx[8] + cov[17]*subx[13] + cov[3];
    subx[19] = cov[11]*subx[15] + cov[12]*subx[14] - cov[13]*subx[8] + cov[14]*subx[13] + cov[2];
    subx[20] = cov[0] + cov[2]*subx[15] + cov[3]*subx[14] - cov[4]*subx[8] + cov[5]*dt*subx[9];
    subx[21] = cov[10]*subx[13] + cov[1] + cov[7]*subx[15] + cov[8]*subx[14] - cov[9]*subx[8];
    subx[22] = -dt*subx[4]*this->params.R_s + 1;
    subx[23] = dt*subx[4]*(-subx[0]*u_alpha + subx[1]*u_beta);
    subx[24] = dt*state[0]*subx[4]*this->params.L_q*this->params.N_P;
    subx[25] = state[3]*subx[4]*this->params.L_q;
    subx[26] = -dt*subx[2]*this->params.R_s + 1;
    subx[27] = dt*subx[2]*(-state[2]*this->params.L_d*this->params.N_P - this->params.N_P*this->params.lambda_m);
    subx[28] = dt*subx[2]*(-subx[0]*u_beta - subx[1]*u_alpha);
    subx[29] = dt*state[0]*subx[2]*this->params.L_d*this->params.N_P;
    subx[30] = cov[11]*subx[22] + cov[12]*subx[24] + cov[2]*subx[11]*subx[25] + cov[7]*subx[23];
    subx[31] = cov[1]*subx[11]*subx[25] + cov[6]*subx[23] + cov[7]*subx[22] + cov[8]*subx[24];
    subx[32] = cov[12]*subx[22] + cov[15]*subx[24] + cov[3]*subx[11]*subx[25] + cov[8]*subx[23];
    subx[33] = cov[0]*subx[11]*subx[25] + cov[1]*subx[23] + cov[2]*subx[22] + cov[3]*state[0]*subx[11]*subx[4]*this->params.L_q;
    state_n[0] = state[0] + (5.0/6.0)*subx[10] + subx[8]*(-1.0/6.0*state[4] + (1.0/4.0)*subx[3]*this->params.N_P*(subx[6]*subx[7] + this->params.lambda_m));
    state_n[1] = (5.0/6.0)*dt*state[0]*this->params.N_P + state[1] + subx[11]*((1.0/3.0)*dt*state[5]*subx[9] + (1.0/6.0)*state[0]);
    state_n[2] = (1.0/6.0)*dt*subx[4]*(subx[0]*u_beta + subx[1]*u_alpha + subx[3]*this->params.L_q*this->params.N_P*(state[0] + 2*subx[10]) - subx[6]*this->params.R_s) + (5.0/6.0)*dt*subx[5] + state[2];
    state_n[3] = dt*subx[2]*(-1.0/6.0*subx[0]*u_alpha + (1.0/6.0)*subx[1]*u_beta - 1.0/6.0*subx[3]*this->params.R_s - 1.0/6.0*subx[6]*this->params.L_d*this->params.N_P*(state[0] + 2*subx[10]) - 1.0/6.0*this->params.N_P*this->params.lambda_m*(state[0] + 2*subx[10])) + (5.0/6.0)*dt*subx[2]*(-state[0]*state[2]*this->params.L_d*this->params.N_P - state[0]*this->params.N_P*this->params.lambda_m - state[3]*this->params.R_s - subx[0]*u_alpha + subx[1]*u_beta) + state[3];
    state_n[4] = state[4];
    state_n[5] = state[5];
    cov_n[0] = subx[12]*((this->params.omega_pnoise)*(this->params.omega_pnoise)) + subx[13]*subx[16] + subx[14]*subx[18] + subx[15]*subx[19] - subx[17]*subx[8] + subx[20];
    cov_n[1] = subx[11]*subx[20] + subx[21];
    cov_n[2] = subx[11]*subx[20]*subx[25] + subx[18]*subx[24] + subx[19]*subx[22] + subx[21]*subx[23];
    cov_n[3] = subx[18]*subx[26] - subx[19]*subx[29] + subx[20]*subx[27] + subx[21]*subx[28];
    cov_n[4] = subx[17];
    cov_n[5] = subx[16];
    cov_n[6] = cov[1]*subx[11] + cov[6] + subx[11]*(cov[0]*subx[11] + cov[1]);
    cov_n[7] = subx[11]*subx[25]*(cov[0]*subx[11] + cov[1]) + subx[22]*(cov[2]*subx[11] + cov[7]) + subx[23]*(cov[1]*subx[11] + cov[6]) + subx[24]*(cov[3]*subx[11] + cov[8]);
    cov_n[8] = subx[26]*(cov[3]*subx[11] + cov[8]) + subx[27]*(cov[0]*subx[11] + cov[1]) + subx[28]*(cov[1]*subx[11] + cov[6]) - subx[29]*(cov[2]*subx[11] + cov[7]);
    cov_n[9] = cov[4]*subx[11] + cov[9];
    cov_n[10] = cov[10] + cov[5]*dt*this->params.N_P;
    cov_n[11] = dt*subx[25]*subx[33]*this->params.N_P + ((subx[0])*(subx[0]))*subx[12]*((this->params.u_noise)*(this->params.u_noise))/((this->params.L_d)*(this->params.L_d)) + subx[12]*((subx[1])*(subx[1]))*((this->params.u_noise)*(this->params.u_noise))/((this->params.L_d)*(this->params.L_d)) + subx[22]*subx[30] + subx[23]*subx[31] + subx[24]*subx[32];
    cov_n[12] = subx[26]*subx[32] + subx[27]*subx[33] + subx[28]*subx[31] - subx[29]*subx[30];
    cov_n[13] = cov[13]*subx[22] + cov[16]*subx[24] + cov[4]*subx[11]*subx[25] + cov[9]*subx[23];
    cov_n[14] = cov[10]*subx[23] + cov[14]*subx[22] + cov[17]*subx[24] + cov[5]*dt*subx[25]*this->params.N_P;
    cov_n[15] = ((subx[0])*(subx[0]))*subx[12]*((this->params.u_noise)*(this->params.u_noise))/((this->params.L_q)*(this->params.L_q)) + subx[12]*((subx[1])*(subx[1]))*((this->params.u_noise)*(this->params.u_noise))/((this->params.L_q)*(this->params.L_q)) + subx[26]*(-cov[12]*subx[29] + cov[15]*subx[26] + cov[3]*subx[27] + cov[8]*subx[28]) + subx[27]*(cov[0]*subx[27] + cov[1]*subx[28] - cov[2]*subx[29] + cov[3]*subx[26]) + subx[28]*(cov[1]*subx[27] + cov[6]*subx[28] - cov[7]*subx[29] + cov[8]*subx[26]) - subx[29]*(-cov[11]*subx[29] + cov[12]*subx[26] + cov[2]*subx[27] + cov[7]*subx[28]);
    cov_n[16] = -cov[13]*subx[29] + cov[16]*subx[26] + cov[4]*subx[27] + cov[9]*subx[28];
    cov_n[17] = cov[10]*subx[28] - cov[14]*subx[29] + cov[17]*subx[26] + cov[5]*dt*subx[2]*(-state[2]*this->params.L_d*this->params.N_P - this->params.N_P*this->params.lambda_m);
    cov_n[18] = cov[18] + subx[12]*((this->params.alpha_load_pnoise)*(this->params.alpha_load_pnoise))/((state[5])*(state[5]));
    cov_n[19] = cov[19];
    cov_n[20] = cov[20] + 0.0001*((state[5])*(state[5]))*subx[12];

    state_n[1] = wrap_2pi(state_n[1]);
    this->state_idx = next_state_idx;
}

void ekf_update(struct ekf_obj_s* this, float i_alpha_m, float i_beta_m) {
    uint8_t next_state_idx = (this->state_idx+1)%2;
    float* state = this->state[this->state_idx].x;
    float* cov = this->state[this->state_idx].P;
    float* state_n = this->state[next_state_idx].x;
    float* cov_n = this->state[next_state_idx].P;
    float* innov = this->state[next_state_idx].innov;
    float* NIS = &this->state[next_state_idx].NIS;

    // 352 operations
    subx[0] = cosf_fast(state[1]);
    subx[1] = sinf_fast(state[1]);
    subx[2] = state[2]*subx[0] - state[3]*subx[1];
    subx[3] = cov[1]*subx[2] + cov[2]*subx[1] + cov[3]*subx[0];
    subx[4] = -state[2]*subx[1] - state[3]*subx[0];
    subx[5] = cov[6]*subx[2] + cov[7]*subx[1] + cov[8]*subx[0];
    subx[6] = cov[11]*subx[1] + cov[12]*subx[0] + cov[7]*subx[2];
    subx[7] = cov[12]*subx[1] + cov[15]*subx[0] + cov[8]*subx[2];
    subx[8] = subx[0]*subx[6] - subx[1]*subx[7] + subx[4]*subx[5];
    subx[9] = subx[0]*subx[7] + subx[1]*subx[6] + subx[2]*subx[5] + ((this->params.i_noise)*(this->params.i_noise));
    subx[10] = cov[6]*subx[4] + cov[7]*subx[0] - cov[8]*subx[1];
    subx[11] = cov[11]*subx[0] - cov[12]*subx[1] + cov[7]*subx[4];
    subx[12] = cov[12]*subx[0] - cov[15]*subx[1] + cov[8]*subx[4];
    subx[13] = subx[0]*subx[11] + subx[10]*subx[4] - subx[12]*subx[1] + ((this->params.i_noise)*(this->params.i_noise));
    subx[14] = 1.0/(subx[13]*subx[9] - ((subx[8])*(subx[8])));
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
    subx[25] = subx[11]*subx[20] - subx[17]*subx[6];
    subx[26] = -subx[11]*subx[17] + subx[15]*subx[6];
    subx[27] = subx[12]*subx[20] - subx[17]*subx[7];
    subx[28] = -subx[12]*subx[17] + subx[15]*subx[7];
    subx[29] = cov[13]*subx[0] - cov[16]*subx[1] + cov[9]*subx[4];
    subx[30] = cov[13]*subx[1] + cov[16]*subx[0] + cov[9]*subx[2];
    subx[31] = -subx[17]*subx[30] + subx[20]*subx[29];
    subx[32] = subx[15]*subx[30] - subx[17]*subx[29];
    subx[33] = cov[10]*subx[2] + cov[14]*subx[1] + cov[17]*subx[0];
    subx[34] = cov[10]*subx[4] + cov[14]*subx[0] - cov[17]*subx[1];
    subx[35] = subx[15]*subx[33] - subx[17]*subx[34];
    subx[36] = -subx[17]*subx[33] + subx[20]*subx[34];
    subx[37] = -subx[18]*subx[2] - subx[21]*subx[4];
    subx[38] = -subx[0]*subx[21] - subx[18]*subx[1];
    subx[39] = -subx[0]*subx[18] + subx[1]*subx[21];
    subx[40] = -subx[23]*subx[2] - subx[24]*subx[4] + 1;
    subx[41] = -subx[0]*subx[24] - subx[1]*subx[23];
    subx[42] = -subx[0]*subx[23] + subx[1]*subx[24];
    subx[43] = -subx[0]*subx[25] - subx[1]*subx[26] + 1;
    subx[44] = -subx[0]*subx[26] + subx[1]*subx[25];
    subx[45] = -subx[25]*subx[4] - subx[26]*subx[2];
    subx[46] = -subx[0]*subx[27] - subx[1]*subx[28];
    subx[47] = -subx[0]*subx[28] + subx[1]*subx[27] + 1;
    subx[48] = -subx[27]*subx[4] - subx[28]*subx[2];
    state_n[0] = state[0] + subx[18]*subx[19] + subx[21]*subx[22];
    state_n[1] = state[1] + subx[19]*subx[23] + subx[22]*subx[24];
    state_n[2] = state[2] + subx[19]*subx[26] + subx[22]*subx[25];
    state_n[3] = state[3] + subx[19]*subx[28] + subx[22]*subx[27];
    state_n[4] = state[4] + subx[19]*subx[32] + subx[22]*subx[31];
    state_n[5] = state[5] + subx[19]*subx[35] + subx[22]*subx[36];
    cov_n[0] = cov[0] + cov[1]*subx[37] + cov[2]*subx[38] + cov[3]*subx[39];
    cov_n[1] = cov[1] + cov[6]*subx[37] + cov[7]*subx[38] + cov[8]*subx[39];
    cov_n[2] = cov[11]*subx[38] + cov[12]*subx[39] + cov[2] + cov[7]*subx[37];
    cov_n[3] = cov[12]*subx[38] + cov[15]*subx[39] + cov[3] + cov[8]*subx[37];
    cov_n[4] = cov[13]*subx[38] + cov[16]*subx[39] + cov[4] + cov[9]*subx[37];
    cov_n[5] = cov[10]*subx[37] + cov[14]*subx[38] + cov[17]*subx[39] + cov[5];
    cov_n[6] = cov[6]*subx[40] + cov[7]*subx[41] + cov[8]*subx[42];
    cov_n[7] = cov[11]*subx[41] + cov[12]*subx[42] + cov[7]*subx[40];
    cov_n[8] = cov[12]*subx[41] + cov[15]*subx[42] + cov[8]*subx[40];
    cov_n[9] = cov[13]*subx[41] + cov[16]*subx[42] + cov[9]*subx[40];
    cov_n[10] = cov[10]*subx[40] + cov[14]*subx[41] + cov[17]*subx[42];
    cov_n[11] = cov[11]*subx[43] + cov[12]*subx[44] + cov[7]*subx[45];
    cov_n[12] = cov[12]*subx[43] + cov[15]*subx[44] + cov[8]*subx[45];
    cov_n[13] = cov[13]*subx[43] + cov[16]*subx[44] + cov[9]*subx[45];
    cov_n[14] = cov[10]*subx[45] + cov[14]*subx[43] + cov[17]*subx[44];
    cov_n[15] = cov[12]*subx[46] + cov[15]*subx[47] + cov[8]*subx[48];
    cov_n[16] = cov[13]*subx[46] + cov[16]*subx[47] + cov[9]*subx[48];
    cov_n[17] = cov[10]*subx[48] + cov[14]*subx[46] + cov[17]*subx[47];
    cov_n[18] = cov[13]*(-subx[0]*subx[31] - subx[1]*subx[32]) + cov[16]*(-subx[0]*subx[32] + subx[1]*subx[31]) + cov[18] + cov[9]*(-subx[2]*subx[32] - subx[31]*subx[4]);
    cov_n[19] = cov[10]*(-subx[2]*subx[32] - subx[31]*subx[4]) + cov[14]*(-subx[0]*subx[31] - subx[1]*subx[32]) + cov[17]*(-subx[0]*subx[32] + subx[1]*subx[31]) + cov[19];
    cov_n[20] = cov[10]*(-subx[2]*subx[35] - subx[36]*subx[4]) + cov[14]*(-subx[0]*subx[36] - subx[1]*subx[35]) + cov[17]*(-subx[0]*subx[35] + subx[1]*subx[36]) + cov[20];
    innov[0] = i_alpha_m - state[2]*cosf_fast(state[1]) + state[3]*sinf_fast(state[1]);
    innov[1] = i_beta_m - state[2]*sinf_fast(state[1]) - state[3]*cosf_fast(state[1]);
    *NIS = subx[19]*(subx[15]*subx[19] - subx[17]*subx[22]) + subx[22]*(-subx[17]*subx[19] + subx[20]*subx[22]);

    state_n[1] = wrap_2pi(state_n[1]);
    this->state_idx = next_state_idx;
}
