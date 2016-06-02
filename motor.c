#include "motor.h"

#include <string.h>
#include "helpers.h"
#include "pwm.h"
#include "adc.h"
#include "drv.h"
#include "encoder.h"
#include "timing.h"
#include "curr_pid.h"

// config things - to be made params later
static const uint8_t elec_rots_per_mech_rot = 7;
static const float theta_offset = 0.0f*M_PI_F/180.0f/7.0f;
static const float curr_KR = 9.0f;
static const float curr_KP = 30.0f;
static const float curr_KI = 10000.0f;
static const float vsense_div = 20.0f;
static const float csa_G = 80.0f;
static const float csa_R = 0.001f;
static const float max_duty = 0.95f;

static float csa_cal[3] = {0.0f, 0.0f, 0.0f}; // current sense amplifier calibration
static float vbatt_m = 0.0f; // battery voltage
static float ia_m = 0.0f, ib_m = 0.0f, ic_m = 0.0f; // phase currents
static float id_est = 0.0f, iq_est = 0.0f, io_est = 0.0f; // dqo transform of phase currents
static float mech_theta_m = 0.0f; // mechanical rotor angle
static float prev_mech_theta_m = 0.0f; // previous mechanical rotor angle for differentiation
static float elec_theta_m = 0.0f; // electrical rotor angle
static float mech_omega_est = 0.0f; // mechanical rotor angular velocity
static bool enable = false; // enables motor


struct curr_pid_param_s iq_pid_param;
struct curr_pid_state_s iq_pid_state;

struct curr_pid_param_s id_pid_param;
struct curr_pid_state_s id_pid_state;

static void retrieve_adc_measurements(void);
static void retrieve_encoder_measurement(void);
static void update_estimates(float dt);
static void load_pid_configs(void);

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

    id_pid_param.dt  = iq_pid_param.dt  = dt;

    id_pid_param.i_meas = id_est;
    iq_pid_param.i_meas = iq_est;

    if (enable) {
        iq_pid_param.output_limit = max_duty*vbatt_m;
        curr_pid_run(&iq_pid_param, &iq_pid_state);

        id_pid_param.output_limit = sqrtf(SQ(iq_pid_param.output_limit)-SQ(iq_pid_state.output));
        curr_pid_run(&id_pid_param, &id_pid_state);

        float alpha = id_pid_state.output*cosf(elec_theta_m) - iq_pid_state.output*sinf(elec_theta_m);
        float beta = iq_pid_state.output*cosf(elec_theta_m) + id_pid_state.output*sinf(elec_theta_m);

        float a,b,c;
        svgen(alpha, beta, &a, &b, &c);
        a -= (1.0f-max_duty)*0.5f;
        b -= (1.0f-max_duty)*0.5f;
        c -= (1.0f-max_duty)*0.5f;

        set_phase_duty(a, b, c);
    } else {
        // reset PID states and inputs
        memset(&id_pid_state,0,sizeof(id_pid_state));
        memset(&iq_pid_state,0,sizeof(iq_pid_state));
        id_pid_param.i_ref = 0.0f;
        iq_pid_param.i_ref = 0.0f;

        // the DRV is in 6 pwm mode - motor will be free-spinning at 0 duty cycle
        set_phase_duty(0.0f, 0.0f, 0.0f);
    }
}

void motor_set_enable(bool val)
{
    enable = val;

    if (enable) {
        drv_write_register_bits(0x7,7,8,0b01); // 3-PWM mode
    } else {
        drv_write_register_bits(0x7,7,8,0b00); // 6-PWM mode
    }
}

void motor_set_id_ref(float id_ref)
{
    id_pid_param.i_ref = id_ref;
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
    elec_theta_m = wrap_2pi(mech_theta_m*elec_rots_per_mech_rot+theta_offset);
}

static void update_estimates(float dt)
{
    const float tc = 0.0f;
    const float alpha = dt/(dt+tc);
    mech_omega_est += (wrap_pi(mech_theta_m-prev_mech_theta_m)/dt - mech_omega_est) * alpha;
    prev_mech_theta_m = mech_theta_m;

    // update the dqo transform
    dqo_transform(elec_theta_m, ia_m,ib_m,ic_m, &id_est,&iq_est,&io_est);
}

static void load_pid_configs(void)
{
    id_pid_param.K_R = iq_pid_param.K_R = curr_KR;
    id_pid_param.K_P = iq_pid_param.K_P = curr_KP;
    id_pid_param.K_I = iq_pid_param.K_I = curr_KI;
}
