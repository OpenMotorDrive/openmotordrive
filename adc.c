#include "adc.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

void adc_init(void)
{
    rcc_periph_clock_enable(RCC_ADC12);
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0|GPIO1|GPIO2);

    adc_power_off(ADC1);

    adc_set_right_aligned(ADC1);
    ADC_CFGR1(ADC1) |= 0b1010<<6; // EXTSEL 1010 TRGO2
    ADC_CFGR1(ADC1) |= 0b01<<10; // EXTEN
    ADC_SMPR1(ADC1) |= 0b100<<3; // set CH1 sample time
    ADC_SMPR1(ADC1) |= 0b100<<6; // set CH2 sample time
    ADC_SMPR1(ADC1) |= 0b100<<9; // set CH3 sample time
    adc_set_single_conversion_mode(ADC1);
    ADC_SQR1(ADC1) |= 3<<0; // 3 conversions in sequence
    ADC_SQR1(ADC1) |= 1<<6; // first channel
    ADC_SQR1(ADC1) |= 1<<12; // second channel
    ADC_SQR1(ADC1) |= 1<<18; // third channel
    adc_power_on(ADC1);
    int i;
    for (i = 0; i < 800000; i++)
        __asm__("nop");
}
