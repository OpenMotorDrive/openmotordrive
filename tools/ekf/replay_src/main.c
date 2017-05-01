// This program directly reads in raw data from the ESC

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include "helpers.h"
#include "slip.h"

#include "ekf.h"

static void transform_alpha_beta_to_d_q(float theta, float alpha, float beta, float* d, float* q);

struct ekf_obj_s ekf;
struct ekf_params_s ekf_params;
static float encoder_theta_e_bias;
static float encoder_delay;

static const struct {
    const char* name;
    float* ptr;
} param_info[] = {
    {"R_s", &ekf_params.R_s},
    {"L_d", &ekf_params.L_d},
    {"L_q", &ekf_params.L_q},
    {"lambda_m", &ekf_params.lambda_m},
    {"J", &ekf_params.J},
    {"N_P", &ekf_params.N_P},
    {"i_noise", &ekf_params.i_noise},
    {"u_noise", &ekf_params.u_noise},
    {"alpha_load_pnoise", &ekf_params.alpha_load_pnoise},
    {"omega_pnoise", &ekf_params.omega_pnoise},
    {"encoder_theta_e_bias", &encoder_theta_e_bias},
    {"encoder_delay", &encoder_delay},
};

#define N_PARAMS (sizeof(param_info)/sizeof(param_info[0]))

static bool read_config_file(FILE* config_file) {
    char line[255];
    uint8_t count = 0;
    bool set[N_PARAMS] = {};
    while (fgets(line, 255, config_file)) {
        char name[255];
        float val;
        if (sscanf(line, "%s %f\n", name, &val) != 2) {
            return false;
        }
        uint32_t i;
        bool found = false;
        for (i=0; i<N_PARAMS; i++) {
            if (strcmp(param_info[i].name, name) == 0) {
                if (set[i]) {
                    printf("duplicate param %s\n", name);
                    return false;
                }
                *param_info[i].ptr = val;
                set[i] = true;
                found = true;
                break;
            }
        }
        if (!found) {
            printf("no param %s\n", name);
            return false;
        }
        count++;
    }
    if (count == N_PARAMS) {
        return true;
    } else {
        uint8_t i;
        for (i=0; i<N_PARAMS; i++) {
            if (!set[i]) {
                printf("missing param %s\n", param_info[i].name);
            }
        }
        return false;
    }
}

struct packet_s {
    uint32_t tnow_us;
    float dt;
    float encoder_theta_e;
    float encoder_omega_e;
    float i_alpha_m;
    float i_beta_m;
    float u_alpha;
    float u_beta;
    float encoder_theta_m;
    float i_a;
    float i_b;
    float i_c;
};

static long double theta_e_err_abs_sum = 0;
static long double theta_e_err_sq_sum = 0;
static long double curr_innov_sq_sum = 0;
static long double NIS_sum = 0;
static long double variance_sum = 0;
static long double dt_sum = 0;
static long double load_torque_sq_sum = 0;
static long double curr_err_sq_sum = 0;

static float prev_u_alpha = 0, prev_u_beta = 0;

static void handle_decoded_pkt(uint8_t len, uint8_t* buf, FILE* out_file) {
    static bool ekf_initialized = false;
    if (len != sizeof(struct packet_s)) {
        return;
    }
    struct packet_s* pkt = (struct packet_s*)buf;

    pkt->encoder_theta_e = wrap_2pi(pkt->encoder_theta_e-encoder_theta_e_bias+pkt->encoder_omega_e*encoder_delay);

    if (!ekf_initialized) {
        ekf_init(&ekf, &ekf_params, pkt->encoder_theta_e);
        ekf_initialized = true;
    } else {
        float* x = ekf_get_state(&ekf)->x;
        float* P = ekf_get_state(&ekf)->P;
//         ekf_state[ekf_idx].x[1] = pkt->encoder_theta_e;
//         memset(ekf_state[ekf_idx].P, 0, sizeof(ekf_state[ekf_idx].P));

        float u_alpha, u_beta;
        ekf_predict(&ekf, pkt->dt, pkt->i_alpha_m, pkt->i_beta_m, pkt->u_alpha, pkt->u_beta);
        ekf_update(&ekf, pkt->dt, pkt->i_alpha_m, pkt->i_beta_m, pkt->u_alpha, pkt->u_beta);

        if (P[9] < 0) {
            P[9] = 0;
        }

//         ekf_state[ekf_idx].x[0] = pkt->encoder_omega_e/N_P;
//         ekf_state[ekf_idx].x[1] = pkt->encoder_theta_e;
//         memset(ekf_state[ekf_idx].P, 0, sizeof(ekf_state[ekf_idx].P));

    }

    float* x = ekf_get_state(&ekf)->x;
    float* P = ekf_get_state(&ekf)->P;
    float* innov = ekf_get_state(&ekf)->innov;
    float NIS = ekf_get_state(&ekf)->NIS;

    float i_d_m, i_q_m;
    transform_alpha_beta_to_d_q(pkt->encoder_theta_e, pkt->i_alpha_m, pkt->i_beta_m, &i_d_m, &i_q_m);

    float i_alpha, i_beta;
    transform_alpha_beta_to_d_q(-x[1], x[2], x[3], &i_alpha, &i_beta);


    float u_d, u_q;
    transform_alpha_beta_to_d_q(pkt->encoder_theta_e, pkt->u_alpha, pkt->u_beta, &u_d, &u_q);

    float theta_e_err = wrap_pi(pkt->encoder_theta_e-x[1]);
    float omega_e_est = x[0]*ekf.params.N_P;
    float omega_e_err = pkt->encoder_omega_e-omega_e_est;

    load_torque_sq_sum += SQ(x[4]);
    theta_e_err_abs_sum += fabsf(theta_e_err);
    theta_e_err_sq_sum += SQ(theta_e_err);
    curr_innov_sq_sum += SQ(innov[0])+SQ(innov[1]);
    curr_err_sq_sum += SQ(pkt->i_alpha_m-i_alpha)+SQ(pkt->i_beta_m-i_beta);
    NIS_sum += NIS;
//         variance_sum += P[0];
//         variance_sum += P[5];
    variance_sum += P[9]+P[12];
//         variance_sum += P[14];
    dt_sum += pkt->dt;

#ifndef NO_BULK_DATA
    fprintf(out_file, "{\"t_us\":%u, \"dt\":%9g, \"encoder_theta_e\":%9g, \"encoder_omega_e\":%9g, \"i_alpha_m\":%9g, \"i_beta_m\":%9g, \"u_alpha\":%9g, \"u_beta\":%9g, \"x\":[%9g, %9g, %9g, %9g, %9g], \"P\": [%9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g], \"theta_e_err\":%9g, \"omega_e_est\":%9g, \"omega_e_err\":%9g, \"i_d_m\": %9g, \"i_q_m\": %9g, \"NIS\": %9g, \"u_d\": %9g, \"u_q\": %9g}", pkt->tnow_us, pkt->dt, pkt->encoder_theta_e, pkt->encoder_omega_e, pkt->i_alpha_m, pkt->i_beta_m, pkt->u_alpha, pkt->u_beta, x[0], x[1], x[2], x[3], x[4], P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7], P[8], P[9], P[10], P[11], P[12], P[13], P[14], theta_e_err, omega_e_est, omega_e_err, i_d_m, i_q_m, NIS, u_d, u_q);
#endif
}

int main(int argc, char **argv) {
    if (argc != 4) {
        printf("%s <CONFIG_FILE> <IN_FILE> <OUT_FILE>\n", argv[0]);
        return 1;
    }

    FILE* config_file = fopen(argv[1], "r");
    FILE* in_file = fopen(argv[2], "r");
    FILE* out_file = fopen(argv[3], "w+");

    if (config_file == NULL) {
        printf("could not open file %s\n", argv[1]);
        return 1;
    }

    if (in_file == NULL) {
        printf("could not open file %s\n", argv[2]);
        return 1;
    }

    if (out_file == NULL) {
        printf("could not open file %s\n", argv[3]);
        return 1;
    }

    if (!read_config_file(config_file)) {
        printf("could not read config file\n");
        return 1;
    }

    fprintf(out_file, "{\"data\":[\n");

    uint8_t pkt_buf[255];
    uint8_t pkt_len = 0;
    uint32_t frame_num = 0;
    bool first_frame = true;

    while (1) {
        uint8_t byte = fgetc(in_file);
        if (feof(in_file) || dt_sum > 11.5) {
            break;
        }
        pkt_buf[pkt_len++] = (uint8_t)byte;
        if (byte == SLIP_END) {
            uint8_t decoded_pkt[255];
            uint8_t decoded_pkt_len;
            decoded_pkt_len = slip_decode(pkt_len, pkt_buf, decoded_pkt);

            if (decoded_pkt_len == sizeof(struct packet_s)) {
                if (!first_frame) {
#ifndef NO_BULK_DATA
                    fprintf(out_file, ",\n");
#endif
                }
                handle_decoded_pkt(decoded_pkt_len, decoded_pkt, out_file);
                first_frame = false;
            } else {
//                 printf("frame %u length incorrect, %u\n", frame_num, decoded_pkt_len);
            }

            frame_num++;
            pkt_len = 0;
        }
    }

    double ISE = theta_e_err_sq_sum/dt_sum;
    double int_NIS = NIS_sum/dt_sum;
    double var_int = variance_sum/dt_sum;
    double load_sq_int = load_torque_sq_sum/dt_sum;
    double curr_err_sq_int = curr_err_sq_sum/dt_sum;
    if (isnan(ISE)) {
        ISE = DBL_MAX;
    }
    if (isnan(int_NIS)) {
        int_NIS = DBL_MAX;
    }
    if (isnan(var_int)) {
        var_int = DBL_MAX;
    }
    if (isnan(load_sq_int)) {
        load_sq_int = DBL_MAX;
    }
    if (isnan(curr_err_sq_int)) {
        curr_err_sq_int = DBL_MAX;
    }

    fprintf(out_file, "],\n");
//     fprintf(out_file, "\"theta_IAE\": %9g,\n", (double)(theta_e_err_abs_sum/dt_sum));
    fprintf(out_file, "\"N_P\": %9g,\n", ekf.params.N_P);
    fprintf(out_file, "\"int_NIS\": %9g,\n", int_NIS);
    fprintf(out_file, "\"theta_ISE\": %9g,\n", ISE);
    fprintf(out_file, "\"var_int\": %9g,\n", var_int);
    fprintf(out_file, "\"load_sq_int\": %9g,\n", load_sq_int);
    fprintf(out_file, "\"curr_err_sq_int\": %9g\n", curr_err_sq_int);
    fprintf(out_file, "}\n");

    printf("dt_sum %9g\n", (double)dt_sum);
    printf("ISE %9g\n", ISE);
    printf("var_int %9g\n", var_int);
//     printf("IAE %9g\n", theta_e_err_abs_sum/dt_sum);
    printf("NIS_sum/dt_sum %9g\n", int_NIS);
    printf("load_sq_int %9g\n", load_sq_int);
    printf("curr_err_sq_int %9g\n", curr_err_sq_int);
//     printf("curr_innov_sq_sum/dt_sum %9g\n", curr_innov_sq_sum/dt_sum);

    fclose(config_file);
    fclose(in_file);
    fclose(out_file);

    return 0;
}

static void transform_alpha_beta_to_d_q(float theta, float alpha, float beta, float* d, float* q)
{
    float sin_theta = sinf_fast(theta);
    float cos_theta = cosf_fast(theta);

    *d = alpha*cos_theta + beta*sin_theta;
    *q = -alpha*sin_theta + beta*cos_theta;
}
