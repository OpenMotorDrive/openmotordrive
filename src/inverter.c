#include "inverter.h"
#include <common/helpers.h>
#include "pwm.h"
#include <common/timing.h>
#include "adc.h"
#include "drv.h"
#include "param.h"
#include "serial.h"

#include <string.h>
#include <stdbool.h>

// double-buffered phase output
static volatile uint8_t phase_output_idx = 0;
static volatile struct {
    bool enabled;
    uint32_t t_us;
    float u_alpha;
    float u_beta;
    float omega;
} phase_output[2];

// double-buffered sense data
static volatile uint8_t sense_data_idx = 0;
static volatile struct inverter_sense_data_s sense_data[2];

// current sense amplifier calibration
static float csa_cal[3];

// configuration parameters
static struct {
    float pwm_deadtime;
    float vsense_div;
    float csa_R;
} params;

static void load_config(void);
static void update_sense_data(void);
static void new_adc_data(void);

void inverter_init(void)
{
    uint16_t i;

    load_config();

    inverter_disable_output();
    adc_set_new_sample_isr(new_adc_data);

    // calibrate phase currents
    volatile struct adc_sample_s* adc_sample;
    drv_csa_cal_mode_on();
    usleep(50);
    memset(csa_cal, 0, sizeof(csa_cal));
    for(i=0; i<1000; i++) {
        adc_wait_for_sample();
        adc_sample = adc_get_sample();
        csa_cal[0] += adc_sample->csa_v[0];
        csa_cal[1] += adc_sample->csa_v[1];
        csa_cal[2] += adc_sample->csa_v[2];
    }
    drv_csa_cal_mode_off();
    csa_cal[0] /= 1000;
    csa_cal[1] /= 1000;
    csa_cal[2] /= 1000;
}

void inverter_set_alpha_beta_output_voltages(uint32_t t_us, float u_alpha, float u_beta, float omega)
{
    if (!phase_output[phase_output_idx].enabled) {
        drv_3_pwm_mode();
    }

    uint8_t next_phase_output_idx = (phase_output_idx+1)%2;
    phase_output[next_phase_output_idx].enabled = true;
    phase_output[next_phase_output_idx].t_us = t_us;
    phase_output[next_phase_output_idx].omega = omega;
    phase_output[next_phase_output_idx].u_alpha = u_alpha;
    phase_output[next_phase_output_idx].u_beta = u_beta;
    phase_output_idx = next_phase_output_idx;
}

void inverter_disable_output(void)
{
    uint8_t next_phase_output_idx = (phase_output_idx+1)%2;
    phase_output[next_phase_output_idx].enabled = false;
    phase_output[next_phase_output_idx].t_us = micros();
    phase_output[next_phase_output_idx].omega = 0;
    phase_output[next_phase_output_idx].u_alpha = 0;
    phase_output[next_phase_output_idx].u_beta = 0;
    phase_output_idx = next_phase_output_idx;
    drv_6_pwm_mode();
}

volatile struct inverter_sense_data_s* inverter_get_sense_data(void)
{
    return &sense_data[sense_data_idx];
}

void inverter_get_alpha_beta_output_voltages(float* u_alpha, float* u_beta)
{
    *u_alpha = phase_output[phase_output_idx].u_alpha;
    *u_beta = phase_output[phase_output_idx].u_beta;
}

float inverter_get_sense_data_sample_period(void)
{
    return adc_get_smp_period();
}

static void load_config(void)
{
    params.pwm_deadtime = *param_retrieve_by_name("ESC_PWM_DEADTIME");
    params.vsense_div = *param_retrieve_by_name("ESC_HW_VSENSE_DIV");
    params.csa_R = *param_retrieve_by_name("ESC_HW_CSA_R");
}

static void update_sense_data(void)
{
    volatile struct adc_sample_s* adc_sample = adc_get_sample();
    uint8_t next_sense_data_idx = (sense_data_idx+1)%2;
    sense_data[next_sense_data_idx].t_us = adc_sample->t_us;
    sense_data[next_sense_data_idx].seq = adc_sample->seq;
    sense_data[next_sense_data_idx].v_bus = adc_sample->vsense_v * params.vsense_div;

    // Use the two current sensors on the phases with the lowest duty cycle
    float duty_a, duty_b, duty_c;
    pwm_get_phase_duty(&duty_a, &duty_b, &duty_c);

    if (duty_a > duty_b && duty_a > duty_c) {
        sense_data[next_sense_data_idx].i_b = (adc_sample->csa_v[1]-csa_cal[1])/(drv_get_csa_gain()*params.csa_R);
        sense_data[next_sense_data_idx].i_c = (adc_sample->csa_v[2]-csa_cal[2])/(drv_get_csa_gain()*params.csa_R);
        sense_data[next_sense_data_idx].i_a = -sense_data[next_sense_data_idx].i_b-sense_data[next_sense_data_idx].i_c;
    } else if (duty_b > duty_a && duty_b > duty_c) {
        sense_data[next_sense_data_idx].i_a = (adc_sample->csa_v[0]-csa_cal[0])/(drv_get_csa_gain()*params.csa_R);
        sense_data[next_sense_data_idx].i_c = (adc_sample->csa_v[2]-csa_cal[2])/(drv_get_csa_gain()*params.csa_R);
        sense_data[next_sense_data_idx].i_b = -sense_data[next_sense_data_idx].i_a-sense_data[next_sense_data_idx].i_c;
    } else {
        sense_data[next_sense_data_idx].i_a = (adc_sample->csa_v[0]-csa_cal[0])/(drv_get_csa_gain()*params.csa_R);
        sense_data[next_sense_data_idx].i_b = (adc_sample->csa_v[1]-csa_cal[1])/(drv_get_csa_gain()*params.csa_R);
        sense_data[next_sense_data_idx].i_c = -sense_data[next_sense_data_idx].i_a-sense_data[next_sense_data_idx].i_b;
    }
    sense_data_idx = next_sense_data_idx;
}

static void new_adc_data(void)
{
    update_sense_data();

    uint32_t dt_us = sense_data[sense_data_idx].t_us-phase_output[phase_output_idx].t_us;
    float dt = dt_us*1e-6f + pwm_get_period()/2;
    if (!phase_output[phase_output_idx].enabled || dt > 0.01f) {
        pwm_set_phase_duty(0.5, 0.5, 0.5);
        return;
    }

    // Rotate voltage by delta theta to account for delays
    float delta_theta = dt * phase_output[phase_output_idx].omega;
    float sin_theta = sinf_fast(delta_theta);
    float cos_theta = cosf_fast(delta_theta);
    float alpha = phase_output[phase_output_idx].u_alpha*cos_theta - phase_output[phase_output_idx].u_beta*sin_theta;
    float beta = phase_output[phase_output_idx].u_alpha*sin_theta + phase_output[phase_output_idx].u_beta*cos_theta;

    // Convert to duty cycle
    alpha /= sense_data[sense_data_idx].v_bus;
    beta /= sense_data[sense_data_idx].v_bus;

    // Space-vector generator
    // Per http://www.embedded.com/design/real-world-applications/4441150/2/Painless-MCU-implementation-of-space-vector-modulation-for-electric-motor-systems
    // Does not overmodulate, provided the input magnitude is <= 1.0/sqrt(2)
    float duty_a, duty_b, duty_c, neutral;
    transform_alpha_beta_to_a_b_c(alpha, beta, &duty_a, &duty_b, &duty_c);

    // Correct voltages for dead-time
    const float deadtime_correction = params.pwm_deadtime/(3*pwm_get_period());
    const float sign_i_a = SIGN(sense_data[sense_data_idx].i_a);
    const float sign_i_b = SIGN(sense_data[sense_data_idx].i_b);
    const float sign_i_c = SIGN(sense_data[sense_data_idx].i_c);
    duty_a += (2*sign_i_a-sign_i_b-sign_i_c)*deadtime_correction;
    duty_b += (2*sign_i_b-sign_i_a-sign_i_c)*deadtime_correction;
    duty_c += (2*sign_i_c-sign_i_a-sign_i_b)*deadtime_correction;

    neutral = 0.5f * (MAX(MAX(duty_a,duty_b),duty_c) + MIN(MIN(duty_a,duty_b),duty_c));
    duty_a += 0.5f-neutral;
    duty_b += 0.5f-neutral;
    duty_c += 0.5f-neutral;

    pwm_set_phase_duty(duty_a, duty_b, duty_c);
}
