#pragma once

#include <stdint.h>

struct inverter_sense_data_s {
    uint32_t t_us;
    uint8_t seq;
    float v_bus;
    float i_a;
    float i_b;
    float i_c;
};

void inverter_init(void);
void inverter_set_alpha_beta_output_voltages(uint32_t t_us, float u_alpha, float u_beta, float omega);
void inverter_get_alpha_beta_output_voltages(float* u_alpha, float* u_beta);
void inverter_disable_output(void);
volatile struct inverter_sense_data_s* inverter_get_sense_data(void);
float inverter_get_sense_data_sample_period(void);
