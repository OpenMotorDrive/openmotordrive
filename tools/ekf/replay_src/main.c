// This program directly reads in raw data from the ESC

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "math_helpers.h"
#include "slip.h"

static void ekf_init(float init_theta);
static void ekf_update(float dt, float u_alpha, float u_beta, float i_alpha_m, float i_beta_m);
static void transform_alpha_beta_to_d_q(float theta, float alpha, float beta, float* d, float* q);

static float R_s;
static float L_d;
static float L_q;
static float K_v;
static float J;
static float N_P;
static float i_noise;
static float u_noise;
static float T_l_pnoise;
static float omega_pnoise;
static float theta_pnoise;
static float encoder_theta_e_bias;

static const struct {
    const char* name;
    float* ptr;
} param_info[] = {
    {"R_s", &R_s},
    {"L_d", &L_d},
    {"L_q", &L_q},
    {"K_v", &K_v},
    {"J", &J},
    {"N_P", &N_P},
    {"i_noise", &i_noise},
    {"u_noise", &u_noise},
    {"T_l_pnoise", &T_l_pnoise},
    {"omega_pnoise", &omega_pnoise},
    {"theta_pnoise", &theta_pnoise},
    {"encoder_theta_e_bias", &encoder_theta_e_bias},
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

#include "ekf.h"

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
};

static long double theta_e_err_abs_sum = 0;
static long double theta_e_err_sq_sum = 0;
static long double curr_innov_sq_sum = 0;
static long double NIS_sum = 0;
static long double dt_sum = 0;

static void handle_decoded_pkt(uint8_t len, uint8_t* buf, FILE* out_file) {
    static bool ekf_initialized = false;
    if (len != sizeof(struct packet_s)) {
        return;
    }
    struct packet_s* pkt = (struct packet_s*)buf;
    pkt->encoder_theta_e = wrap_2pi(pkt->encoder_theta_e-encoder_theta_e_bias);
    if (!ekf_initialized) {
        ekf_init(pkt->encoder_theta_e);
        ekf_initialized = true;
    } else {
        ekf_update(pkt->dt, pkt->u_alpha, pkt->u_beta, pkt->i_alpha_m, pkt->i_beta_m);
    }
    float* x = ekf_state[ekf_idx].x;
    float* P = ekf_state[ekf_idx].P;
    float* innov = ekf_state[ekf_idx].innov;
    float NIS = ekf_state[ekf_idx].NIS;

    float i_d_m, i_q_m;
    transform_alpha_beta_to_d_q(pkt->encoder_theta_e, pkt->i_alpha_m, pkt->i_beta_m, &i_d_m, &i_q_m);

    float theta_e_err = wrap_pi(pkt->encoder_theta_e-x[1]);
    float omega_e_est = x[0]*N_P;
    float omega_e_err = pkt->encoder_omega_e-omega_e_est;

    theta_e_err_abs_sum += fabsf(theta_e_err);
    theta_e_err_sq_sum += SQ(theta_e_err);
    curr_innov_sq_sum += SQ(innov[0])+SQ(innov[1]);
    NIS_sum += NIS;
    dt_sum += pkt->dt;

    fprintf(out_file, "{\"t_us\":%u, \"dt\":%9g, \"encoder_theta_e\":%9g, \"encoder_omega_e\":%9g, \"i_alpha_m\":%9g, \"i_beta_m\":%9g, \"u_alpha\":%9g, \"u_beta\":%9g, \"x\":[%9g, %9g, %9g, %9g, %9g], \"P\": [%9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g], \"theta_e_err\":%9g, \"omega_e_est\":%9g, \"omega_e_err\":%9g, \"i_d_m\": %9g, \"i_q_m\": %9g, \"NIS\": %9g}", pkt->tnow_us, pkt->dt, pkt->encoder_theta_e, pkt->encoder_omega_e, pkt->i_alpha_m, pkt->i_beta_m, pkt->u_alpha, pkt->u_beta, x[0], x[1], x[2], x[3], x[4], P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7], P[8], P[9], P[10], P[11], P[12], P[13], P[14], theta_e_err, omega_e_est, omega_e_err, i_d_m, i_q_m, NIS);
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
        if (feof(in_file)) {
            break;
        }
        pkt_buf[pkt_len++] = (uint8_t)byte;
        if (byte == SLIP_END) {
            uint8_t decoded_pkt[255];
            uint8_t decoded_pkt_len;
            decoded_pkt_len = slip_decode(pkt_len, pkt_buf, decoded_pkt);

            if (decoded_pkt_len == sizeof(struct packet_s)) {
                if (!first_frame) {
                    fprintf(out_file, ",\n");
                }
                handle_decoded_pkt(decoded_pkt_len, decoded_pkt, out_file);
                first_frame = false;
            } else {
                printf("frame %u length incorrect, %u\n", frame_num, decoded_pkt_len);
            }

            frame_num++;
            pkt_len = 0;
        }
    }

    double ISE = theta_e_err_sq_sum/dt_sum;

    if (isnan(ISE)) {
        ISE = DBL_MAX;
    }

    fprintf(out_file, "],\n");
    fprintf(out_file, "\"theta_IAE\": %9g,\n", (double)(theta_e_err_abs_sum/dt_sum));
    fprintf(out_file, "\"theta_ISE\": %9g\n", ISE);
    fprintf(out_file, "}\n");

    printf("dt_sum %9g\n", (double)dt_sum);
    printf("ISE %9g\n", ISE);
//     printf("IAE %9g\n", theta_e_err_abs_sum/dt_sum);
//     printf("NIS_sum/dt_sum %9g\n", NIS_sum/dt_sum);
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
