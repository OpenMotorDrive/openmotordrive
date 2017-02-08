// This program directly reads in raw data from the ESC

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

#define M_PI_F ((float)M_PI)
#define SQ(__X) (__X*__X)

struct ekf_state_s {
    float x[5];
    float P[15];
};
static struct ekf_state_s ekf_state[2];
static uint8_t ekf_idx = 0;

static float wrap_1(float x);
static float wrap_pi(float x);
static float sinf_fast(float x);
static float cosf_fast(float x);
static uint8_t slip_decode(uint8_t in_len, uint8_t *in_buf, uint8_t *out_buf);

static void ekf_init(void);
static void ekf_update(float dt, float u_alpha, float u_beta, float i_alpha_m, float i_beta_m);

static float R_s = .102;
static float L_d = 28e-6;
static float L_q = 44e-6;
static float K_v = 360;
static float J = 0.00002;
static float N_P = 7;
static float i_noise = 0.01;
static float u_noise = 0.6;
static float T_l_pnoise = 0.1;

struct packet_s {
    uint32_t tnow_us;
    float dt;
    float encoder_theta_e;
    float encoder_omega_e;
    float i_alpha_m;
    float i_beta_m;
    float u_alpha;
    float u_beta;
    uint32_t adc_err_cnt;
};

static void handle_decoded_pkt(uint8_t len, uint8_t* buf, FILE* out_file) {
    if (len != sizeof(struct packet_s)) {
        return;
    }
    struct packet_s* pkt = (struct packet_s*)buf;
    ekf_update(pkt->dt, pkt->u_alpha, pkt->u_beta, pkt->i_alpha_m, pkt->i_beta_m);
    float* x = ekf_state[ekf_idx].x;
    float* P = ekf_state[ekf_idx].P;
    fprintf(out_file, "{\"t_us\":%u, \"dt\":%9g, \"encoder_theta_e\":%9g, \"encoder_omega_e\":%9g, \"i_alpha_m\":%9g, \"i_beta_m\":%9g, \"u_alpha\":%9g, \"u_beta\":%9g, \"x\":[%9g, %9g, %9g, %9g, %9g], \"P\": [%9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g, %9g]}", pkt->tnow_us, pkt->dt, pkt->encoder_theta_e, pkt->encoder_omega_e, pkt->i_alpha_m, pkt->i_beta_m, pkt->u_alpha, pkt->u_beta, x[0], x[1], x[2], x[3], x[4], P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7], P[8], P[9], P[10], P[11], P[12], P[13], P[14]);
}

int main(int argc, char **argv) {
    if (argc != 3) {
        printf("%s <IN_FILE> <OUT_FILE>\n", argv[0]);
        return 1;
    }
    ekf_init();

    FILE* in_file = fopen(argv[1], "r");
    FILE* out_file = fopen(argv[2], "w+");

    if (in_file == NULL) {
        printf("could not open file %s\n", argv[1]);
        return 1;
    }

    if (out_file == NULL) {
        printf("could not open file %s\n", argv[2]);
        return 1;
    }

    fprintf(out_file, "{\"data\":[\n");

    uint8_t pkt_buf[255];
    uint8_t pkt_len = 0;
    uint32_t frame_num = 0;
    bool need_comma = false;

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
                if (need_comma) {
                    fprintf(out_file, ",\n");
                }
                handle_decoded_pkt(decoded_pkt_len, decoded_pkt, out_file);
                need_comma = true;
            } else {
                printf("frame %u length incorrect, %u\n", frame_num, decoded_pkt_len);
            }

            frame_num++;
            pkt_len = 0;
        }
    }
    fprintf(out_file, "]}\n");

    fclose(in_file);
    fclose(out_file);

    return 0;
}



static float wrap_1(float x)
{
    volatile float z = (x + 25165824.0f);
    x = x - (z - 25165824.0f);
    return x;
}

static float wrap_pi(float x)
{
    return wrap_1(x/M_PI_F)*M_PI_F;
}

static float wrap_2pi(float x)
{
    x = wrap_pi(x);

    if (x < 0) {
        x += 2*M_PI_F;
    }

    return x;
}

static float sinf_fast(float x)
{
    const float Q = 3.1f;
    const float P = 3.6f;
    float y;

    x = wrap_1(x/M_PI_F);
    y = x - x * fabsf(x);
    return y * (Q + P * fabsf(y));
}

static float cosf_fast(float x)
{
    return sinf_fast(x+M_PI_F/2.0f);
}

static void ekf_init(void) {
    float* state = ekf_state[ekf_idx].x;
    float* cov = ekf_state[ekf_idx].P;

    cov[0] = 100.000000000000;
    cov[1] = 0;
    cov[2] = 0;
    cov[3] = 0;
    cov[4] = 0;
    cov[5] = 9.86960440108936;
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

static void ekf_update(float dt, float u_alpha, float u_beta, float i_alpha_m, float i_beta_m) {
    uint8_t next_ekf_idx = (ekf_idx+1)%2;
    float* state = ekf_state[ekf_idx].x;
    float* cov = ekf_state[ekf_idx].P;
    float* state_n = ekf_state[next_ekf_idx].x;
    float* cov_n = ekf_state[next_ekf_idx].P;

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

static uint8_t slip_decode(uint8_t in_len, uint8_t *in_buf, uint8_t *out_buf) {
    uint8_t i;
    uint8_t out_len = 0;
    uint8_t esc_flag = 0;

    for (i=0; i<in_len; i++) {
        if (esc_flag) {
            if (in_buf[i] == SLIP_ESC_ESC) {
                out_buf[out_len++] = SLIP_ESC;
            } else if (in_buf[i] == SLIP_ESC_END) {
                out_buf[out_len++] = SLIP_END;
            } else {
                // invalid escape character
                return 0;
            }
            esc_flag = 0;
        } else if (in_buf[i] == SLIP_ESC) {
            esc_flag = 1;
        } else if (in_buf[i] == SLIP_END) {
            return out_len;
        } else {
            out_buf[out_len++] = in_buf[i];
        }
    }

    return 0;
}