#include "adc.h"
#include "timing.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

void adc_init(void)
{
    rcc_periph_clock_enable(RCC_ADC12);
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0|GPIO1|GPIO2);

    // set ADC12 clock to AHB clock
    ADC12_CCR |= (0b01UL<<16);

    // right-aligned data
    ADC_CFGR1(ADC1) &= ~(1UL<<5);

    // EXTSEL (1010 == TRGO2)
    ADC_CFGR1(ADC1) |= 0b1010UL<<6;

    // EXTEN
    ADC_CFGR1(ADC1) |= 0b01<<10;

    // set CH1 sample time (100 == 19.5 ADC clock cycles)
    ADC_SMPR1(ADC1) |= 0b100UL<<3;

    // set CH2 sample time (100 == 19.5 ADC clock cycles)
    ADC_SMPR1(ADC1) |= 0b100UL<<6;

    // set CH3 sample time (100 == 19.5 ADC clock cycles)
    ADC_SMPR1(ADC1) |= 0b100UL<<9;
    adc_set_single_conversion_mode(ADC1);

    // number of conversions in sequence
    ADC_SQR1(ADC1) |= 1UL<<0;

    // sequence channels
    ADC_SQR1(ADC1) |= 1UL<<6;
    //ADC_SQR1(ADC1) |= 2UL<<12; // second channel
    //ADC_SQR1(ADC1) |= 3UL<<18; // third channel

    // set ADVREGEN to 00 (intermediate)
    ADC_CR(ADC1) &= ~(0b11UL<<28);

    // set ADVREGEN to 01 (enabled)
    ADC_CR(ADC1) |= 0b01UL<<28;

    // provide time for ADC regulator to start
    usleep(100);

    // set ADCAL to 1
    ADC_CR(ADC1) |= 1UL<<31;

    // wait until ADCAL is 0
    while((ADC_CR(ADC1) & (1UL<<31)) != 0);

    // set ADEN to 1 until ADRDY is 1
    while((ADC_ISR(ADC1) & (1UL<<0)) == 0) {
        ADC_CR(ADC1) |= (1UL<<0);
    };

    ADC_CR(ADC1) |= (1UL<<2); // ADSTART=1
}

uint16_t adc_get(void)
{
    return ADC_DR(ADC1);
}
