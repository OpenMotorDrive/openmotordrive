#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(void);
float csa_v_get(uint8_t phase);
float phase_v_get(uint8_t phase);
float vsense_v_get(void);
void wait_for_adc_sample(void);

#endif // ADC_H
