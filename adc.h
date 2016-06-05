#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(void);
void adc_get_csa_v(float *phaseA, float *phaseB, float *phaseC);
float adc_get_vsense_v(void);
void adc_wait_for_sample(void);
uint8_t adc_get_smpidx(void);
uint32_t adc_get_errcnt(void);

#endif // ADC_H
