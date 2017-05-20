#pragma once
#include <stdint.h>

struct ekf_state_s {
    float x[6];
    float P[21];
    float innov[2];
    float NIS;
};

struct ekf_params_s {
    float R_s;
    float L_d;
    float L_q;
    float lambda_m;
    float N_P;
    float J;
    float alpha_load_pnoise;
    float omega_pnoise;
    float u_noise;
    float i_noise;
};

struct ekf_obj_s {
    struct ekf_params_s params;
    struct ekf_state_s state[2];
    uint8_t state_idx;
};

void ekf_init(struct ekf_obj_s* this, struct ekf_params_s* params, float init_theta);
void ekf_predict(struct ekf_obj_s* this, float dt, float u_alpha, float u_beta);
void ekf_update(struct ekf_obj_s* this, float i_alpha_m, float i_beta_m);
struct ekf_state_s* ekf_get_state(struct ekf_obj_s* this);
