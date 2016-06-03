#include "motor.h"

#include <string.h>
#include "helpers.h"
#include "pwm.h"
#include "adc.h"
#include "drv.h"
#include "encoder.h"
#include "timing.h"
#include "curr_pid.h"
#include "serial.h"

// config things - to be made params later
static uint8_t elec_rots_per_mech_rot = 7;
static float elec_theta_bias = 0.0f;
static bool swap_phases = false;
static const float curr_KR = 9.0f;
static const float curr_KP = 30.0f;
static const float curr_KI = 5000.0f;
static const float vsense_div = 20.0f;
static const float csa_G = 80.0f;
static const float csa_R = 0.001f;
// it takes approximately 50 timer clock cycles to sample the current sensors. PWM period is 2000 timer clock cycles
static const float max_duty = 0.95f;
static const float calibration_voltage = 10.0f;

static float csa_cal[3] = {0.0f, 0.0f, 0.0f}; // current sense amplifier calibration
static float vbatt_m = 0.0f; // battery voltage
static float ia_m = 0.0f, ib_m = 0.0f, ic_m = 0.0f; // phase currents
static float ialpha_m = 0.0f, ibeta_m = 0.0f, igamma_m = 0.0f; // alpha-beta-gamma (clarke) transform of phase currents
static float id_est = 0.0f, iq_est = 0.0f; // dqo transform of phase currents
static float mech_theta_m = 0.0f; // mechanical rotor angle
static float prev_mech_theta_m = 0.0f; // previous mechanical rotor angle for differentiation
static float elec_theta_m = 0.0f; // electrical rotor angle
static float mech_omega_est = 0.0f; // mechanical rotor angular velocity
static enum motor_mode_t motor_mode = MOTOR_MODE_DISABLED; // enables motor
static uint32_t last_print_ms;

struct {
    uint32_t start_time_us;
    uint8_t step;
    float mech_theta_0;
} encoder_calibration_state;

struct curr_pid_param_s iq_pid_param;
struct curr_pid_state_s iq_pid_state;

struct curr_pid_param_s id_pid_param;
struct curr_pid_state_s id_pid_state;

struct curr_pid_param_s io_pid_param;
struct curr_pid_state_s io_pid_state;

static void retrieve_adc_measurements(void);
static void retrieve_encoder_measurement(void);
static void update_estimates(float dt);
static void load_pid_configs(void);
static void transform_a_b_c_to_alpha_beta_gamma(float a, float b, float c, float* alpha, float* beta, float* gamma);
static void transform_d_q_to_alpha_beta(float d, float q, float* alpha, float* beta);
static void transform_alpha_beta_to_d_q(float alpha, float beta, float* d, float* q);
static void svgen(float alpha, float beta, float* a, float* b, float* c);

void motor_init(void)
{
    // calibrate phase currents
    uint8_t i;
    drv_write_register_bits(0xA, 8, 10, 0b111UL);
    csa_cal[0] = 0;
    csa_cal[1] = 0;
    csa_cal[2] = 0;
    for(i=0; i<100; i++) {
        float csa_v_a, csa_v_b, csa_v_c;
        adc_wait_for_sample();
        adc_get_csa_v(&csa_v_a, &csa_v_b, &csa_v_c);
        csa_cal[0] += csa_v_a;
        csa_cal[1] += csa_v_b;
        csa_cal[2] += csa_v_c;
    }
    drv_write_register_bits(0xA, 8, 10, 0b000UL);
    csa_cal[0] /= 100;
    csa_cal[1] /= 100;
    csa_cal[2] /= 100;

    // initialize encoder filter states
    retrieve_encoder_measurement();
    prev_mech_theta_m = mech_theta_m;
    mech_omega_est = 0.0f;
}

void motor_update_state(float dt)
{
    retrieve_adc_measurements();
    retrieve_encoder_measurement();
    update_estimates(dt);
}

void motor_run_commutation(float dt)
{
    load_pid_configs();

    id_pid_param.dt = iq_pid_param.dt  = dt;

    id_pid_param.i_meas = id_est;
    iq_pid_param.i_meas = iq_est;

    char buf[256];
    int n;

    float alpha, beta, theta, a, b, c, t;
    switch (motor_mode) {
        case MOTOR_MODE_DISABLED:
            // reset PID states and inputs
            memset(&id_pid_state,0,sizeof(id_pid_state));
            memset(&iq_pid_state,0,sizeof(iq_pid_state));
            id_pid_param.i_ref = 0.0f;
            iq_pid_param.i_ref = 0.0f;

            // the DRV is in 6 pwm mode - motor should be free-spinning at 0 duty cycle
            set_phase_duty(0.0f, 0.0f, 0.0f);
            break;

        case MOTOR_MODE_FOC_CURRENT:
            iq_pid_param.i_ref = 0.0f;
            iq_pid_param.output_limit = max_duty*vbatt_m;
            curr_pid_run(&iq_pid_param, &iq_pid_state);

            id_pid_param.output_limit = sqrtf(SQ(iq_pid_param.output_limit)-SQ(iq_pid_state.output));
            curr_pid_run(&id_pid_param, &id_pid_state);

            transform_d_q_to_alpha_beta(id_pid_state.output/vbatt_m, iq_pid_state.output/vbatt_m, &alpha, &beta);

            svgen(alpha, beta, &a, &b, &c);
            a -= (1.0f-max_duty)*0.5f;
            b -= (1.0f-max_duty)*0.5f;
            c -= (1.0f-max_duty)*0.5f;

            if (!swap_phases) {
                set_phase_duty(a, b, c);
            } else {
                set_phase_duty(a, c, b);
            }
            break;

        case MOTOR_MODE_ENCODER_CALIBRATION:
            t = (micros() - encoder_calibration_state.start_time_us)*1.0e-6f;

            switch(encoder_calibration_state.step) {
                case 0:
                    // theta is 0 and the motor is given 0.5 seconds to settle
                    theta = 0.0f;
                    if (t > 0.5f) {
                        encoder_calibration_state.mech_theta_0 = mech_theta_m;
                        encoder_calibration_state.step = 1;
                    }
                    break;
                case 1:
                    // theta rotates to 360deg at the 1.5 second mark and is given 0.25 seconds to settle
                    theta = constrain_float(2.0f*M_PI_F * (t-0.5)/1.0f, 0.0f, 2.0f*M_PI_F);
                    if (t > 1.75f) {
                        // elec_rots_per_mech_rot = delta_elec_angle/delta_mech_angle
                        elec_rots_per_mech_rot = (uint8_t)((2.0f*M_PI_F)/fabsf(wrap_pi(mech_theta_m - encoder_calibration_state.mech_theta_0)) + 0.5f);

                        // rotating the field in the positive direction should have rotated the encoder in the positive direction too
                        swap_phases = mech_theta_m < encoder_calibration_state.mech_theta_0;

                        elec_theta_bias = (swap_phases?1.0f:-1.0f)*wrap_pi(elec_rots_per_mech_rot * encoder_calibration_state.mech_theta_0);

                        motor_set_mode(MOTOR_MODE_DISABLED);
                    }
                    break;

            }

            alpha = constrain_float(calibration_voltage/vbatt_m, 0.0f, max_duty) * cosf(theta);
            beta = constrain_float(calibration_voltage/vbatt_m, 0.0f, max_duty) * sinf(theta);

            svgen(alpha, beta, &a, &b, &c);
            a -= (1.0f-max_duty)*0.5f;
            b -= (1.0f-max_duty)*0.5f;
            c -= (1.0f-max_duty)*0.5f;

            set_phase_duty(a, b, c);

            break;

        case MOTOR_MODE_TEST:
            theta = wrap_2pi(2.0f*M_PI_F*millis()*1e-3f);

            alpha = constrain_float(calibration_voltage/vbatt_m, 0.0f, max_duty) * cosf(theta);
            beta = constrain_float(calibration_voltage/vbatt_m, 0.0f, max_duty) * sinf(theta);

            svgen(alpha, beta, &a, &b, &c);
            a -= (1.0f-max_duty)*0.5f;
            b -= (1.0f-max_duty)*0.5f;
            c -= (1.0f-max_duty)*0.5f;

            set_phase_duty(a, b, c);

            uint32_t tnow_ms = millis();
            if (tnow_ms - last_print_ms > 50) {
                n = snprintf(buf, sizeof(buf), "%f,% f\n", mech_theta_m, 180.0f/M_PI_F * wrap_pi(M_PI_F/2.0f+atan2f(ibeta_m,ialpha_m)-elec_theta_m));
                serial_send_dma(n, buf);

                last_print_ms = tnow_ms;
            }

            break;
    }
}

void motor_set_mode(enum motor_mode_t mode)
{
    motor_mode = mode;

    if (motor_mode == MOTOR_MODE_ENCODER_CALIBRATION) {
        encoder_calibration_state.start_time_us = micros();
        encoder_calibration_state.step = 0;
    }
}

void motor_set_id_ref(float id_ref)
{
    id_pid_param.i_ref = id_ref;
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

static void retrieve_adc_measurements(void)
{
    // retrieve battery measurement
    vbatt_m = adc_get_vsense_v()*vsense_div;

    // retrieve current sense amplifier measurement
    float csa_v_a, csa_v_b, csa_v_c;
    adc_get_csa_v(&csa_v_a, &csa_v_b, &csa_v_c);
    ia_m = (csa_v_a-csa_cal[0])/(csa_G*csa_R);
    ib_m = (csa_v_b-csa_cal[1])/(csa_G*csa_R);
    ic_m = (csa_v_c-csa_cal[2])/(csa_G*csa_R);
}

static void retrieve_encoder_measurement(void)
{
    mech_theta_m = wrap_2pi(encoder_read_rad());
    elec_theta_m = wrap_2pi(mech_theta_m*elec_rots_per_mech_rot-elec_theta_bias);
}

static void update_estimates(float dt)
{
    const float tc = 0.0f;
    const float alpha = dt/(dt+tc);
    mech_omega_est += (wrap_pi(mech_theta_m-prev_mech_theta_m)/dt - mech_omega_est) * alpha;
    prev_mech_theta_m = mech_theta_m;

    // update the transformed current measurements
    transform_a_b_c_to_alpha_beta_gamma(ia_m, ib_m, ic_m, &ialpha_m, &ibeta_m, &igamma_m);
    transform_alpha_beta_to_d_q(ialpha_m, ibeta_m, &id_est, &iq_est);
}

static void load_pid_configs(void)
{
    id_pid_param.K_R = iq_pid_param.K_R = curr_KR;
    id_pid_param.K_P = iq_pid_param.K_P = curr_KP;
    id_pid_param.K_I = iq_pid_param.K_I = curr_KI;
}

static void transform_a_b_c_to_alpha_beta_gamma(float a, float b, float c, float* alpha, float* beta, float* gamma)
{
    *alpha = 0.816496580927726f*a - 0.408248290463863f*b - 0.408248290463863f*c;
    *beta = 0.707106781186547f*b - 0.707106781186547f*c;
    *gamma = 0.577350269189626f*a + 0.577350269189626f*b + 0.577350269189626f*c;
}

static void transform_d_q_to_alpha_beta(float d, float q, float* alpha, float* beta)
{
    *alpha = d*cosf(elec_theta_m) - q*sinf(elec_theta_m);
    *beta = d*sinf(elec_theta_m) + q*cosf(elec_theta_m);
}

static void transform_alpha_beta_to_d_q(float alpha, float beta, float* d, float* q)
{
    *d = alpha*cosf(elec_theta_m) + beta*sinf(elec_theta_m);
    *q = -alpha*sinf(elec_theta_m) + beta*cosf(elec_theta_m);
}

static void svgen(float alpha, float beta, float* a, float* b, float* c)
{
    float Va = beta;
    float Vb = -(beta/2.0f)+(alpha*sqrtf(3.0f)/2.0f);
    float Vc = -(beta/2.0f)-(alpha*sqrtf(3.0f)/2.0f);
    uint8_t sector = 0;
    if (Va > 0) sector |= 1<<0;
    if (Vb > 0) sector |= 1<<1;
    if (Vc > 0) sector |= 1<<2;
    float X = beta;
    float Y = (beta/2.0f)+(alpha*sqrtf(3.0f)/2.0f);
    float Z = (beta/2.0f)-(alpha*sqrtf(3.0f)/2.0f);
    switch(sector)
    {
        case 0:
            (*a) = 0.5f;
            (*b) = 0.5f;
            (*c) = 0.5f;
            break;
        case 1:
            (*b) = (1.0f-Z-Y)/2.0f;
            (*a) = (*b)+Z;
            (*c) = (*a)+Y;
            break;
        case 2:
            (*a) = (1.0f-Y+X)/2.0f;
            (*c) = (*a)+Y;
            (*b) = (*c)-X;
            break;
        case 3:
            (*a) = (1.0f+Z-X)/2.0f;
            (*b) = (*a)-Z;
            (*c) = (*b)+X;
            break;
        case 4:
            (*c) = (1.0f+X-Z)/2.0f;
            (*b) = (*c)-X;
            (*a) = (*b)+Z;
            break;
        case 5:
            (*b) = (1.0f-X+Y)/2.0f;
            (*c) = (*b)+X;
            (*a) = (*c)-Y;
            break;
        case 6:
            (*c) = (1.0f+Y+Z)/2.0f;
            (*a) = (*c)-Y;
            (*b) = (*a)-Z;
            break;
    }
}
