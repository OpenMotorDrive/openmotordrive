#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(void);
float csa_v_get(uint8_t phase);
float vsense_v_get(void);

#endif // ADC_H
