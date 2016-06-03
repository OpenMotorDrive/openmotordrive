#include "adc.h"
#include "timing.h"
#include <string.h>
#include <stdbool.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>

#define NUM_CONVERSIONS 6UL

static volatile float csa_v[3] = {0,0,0};
static volatile float vsense_v = 0.0f;
static volatile uint16_t adcbuf[NUM_CONVERSIONS];
static volatile uint8_t smpidx = 0;
static volatile uint32_t smperr = 0;

void adc_init(void)
{
    rcc_periph_clock_enable(RCC_ADC12);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_DMA1);

    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0|GPIO1|GPIO2|GPIO3|GPIO4|GPIO6|GPIO7);

    // set ADC12 clock to AHB clock
    ADC12_CCR |= 0b01UL<<16;

    // set phase current sample time
    ADC_SMPR1(ADC1) |= 0b011UL << 3; // SMP1 = 7.5 ADC clock cycles
    ADC_SMPR1(ADC1) |= 0b011UL << 6; // SMP2 = 7.5 ADC clock cycles
    ADC_SMPR1(ADC1) |= 0b011UL << 9; // SMP3 = 7.5 ADC clock cycles

    // set vsense sample time
    ADC_SMPR1(ADC1) |= 0b100UL << 12; // SMP4 = 19.5 ADC clock cycles

    // enable discontinuous mode
    ADC_CFGR1(ADC1) |= 1UL<<16;
    // discontinuous group size 3
    ADC_CFGR1(ADC1) |= (3UL-1)<<17;

    // right-aligned data
    ADC_CFGR1(ADC1) &= ~(1UL<<5);

    // EXTSEL (1010 == TRGO2)
    ADC_CFGR1(ADC1) |= 0b1010UL<<6;

    // EXTEN
    ADC_CFGR1(ADC1) |= 0b01UL<<10;

    // single conversion mode
    adc_set_single_conversion_mode(ADC1);

    // number of conversions in sequence
    ADC_SQR1(ADC1) |= (NUM_CONVERSIONS-1)<<0;

    // discontinuous group 1
    ADC_SQR1(ADC1) |= 4UL<<6; // Vsense
    ADC_SQR1(ADC1) |= 4UL<<12; // Vsense
    ADC_SQR1(ADC1) |= 4UL<<18; // Vsense

    // discontinuous group 2
    ADC_SQR1(ADC1) |= 1UL<<24; // phase current A
    ADC_SQR2(ADC1) |= 2UL<<0; // phase current B
    ADC_SQR2(ADC1) |= 3UL<<6; // phase current C


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
        ADC_CR(ADC1) |= 1UL<<0;
    };

    // set up DMA
    DMA_CPAR(DMA1,DMA_CHANNEL1) = (uint32_t)&ADC_DR(ADC1);
    DMA_CMAR(DMA1,DMA_CHANNEL1) = (uint32_t)&(adcbuf[0]);
    DMA_CNDTR(DMA1,DMA_CHANNEL1) = NUM_CONVERSIONS; // N transfers

    DMA_CCR(DMA1,DMA_CHANNEL1) |= 0b11UL<<12; // PL very high
    DMA_CCR(DMA1,DMA_CHANNEL1) |= 0b01UL<<10; // MSIZE 16 bits
    DMA_CCR(DMA1,DMA_CHANNEL1) |= 0b10UL<<8; // PSIZE 32 bits
    DMA_CCR(DMA1,DMA_CHANNEL1) |= 1UL<<7; // MINC=1
    DMA_CCR(DMA1,DMA_CHANNEL1) |= 1UL<<5; // CIRC=1
    DMA_CCR(DMA1,DMA_CHANNEL1) |= 1UL<<1; // TCIE=1
    DMA_CCR(DMA1,DMA_CHANNEL1) |= 1UL<<0; // EN=1
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
    ADC_CFGR1(ADC1) |= 1UL<<0; // DMAEN=1
    ADC_CFGR1(ADC1) |= 1UL<<1; // DMACFG=1
    ADC_IER(ADC1) |= 1UL<<4; // OVRIE=1
    nvic_enable_irq(NVIC_ADC1_2_IRQ);
    ADC_CR(ADC1) |= 1UL<<2; // ADSTART=1
}


void adc1_2_isr(void)
{
    if ((ADC_ISR(ADC1)&(1UL<<4)) != 0) {
        // the ADC has stopped generating DMA requests because the DMA was
        // unable to service a request before the next conversion starts.
        // we reset both the DMA and ADC to the beginning of the sequence so that
        // the data stays aligned in the buffer
        smperr++;
        ADC_CR(ADC1) |= 1UL<<4; // ADSTP=1
        while ((ADC_CR(ADC1)&(1UL<<4)) != 0); // wait for ADC to stop
        DMA_CCR(DMA1,DMA_CHANNEL1) &= ~(1UL<<0); // EN=0
        DMA_CNDTR(DMA1,DMA_CHANNEL1) = NUM_CONVERSIONS; // N transfers
        DMA_CCR(DMA1,DMA_CHANNEL1) |= 1UL<<0; // EN=1
        ADC_ISR(ADC1) |= (1UL<<4); // clear interrupt flag
        ADC_CR(ADC1) |= 1UL<<2; // ADSTART=1
    }
}

void dma1_channel1_isr(void)
{
    if ((DMA_ISR(DMA1)&(1UL<<1)) != 0) {
        // TCIF1 is asserted
        smpidx++;
        csa_v[0] = adcbuf[3]*3.3f/4096.0f;
        csa_v[1] = adcbuf[4]*3.3f/4096.0f;
        csa_v[2] = adcbuf[5]*3.3f/4096.0f;
        vsense_v = 0.0f;
        uint8_t i;
        for (i=0; i<3; i++) vsense_v += adcbuf[i];
        vsense_v *= 3.3f/4096.0f/3.0f;

        DMA_IFCR(DMA1) |= 1UL<<1; // clear interrupt flag
    }
}

void adc_wait_for_sample(void)
{
    uint8_t smpidx_prev = smpidx;
    while(smpidx==smpidx_prev);
}

void adc_get_csa_v(float *phaseA, float *phaseB, float *phaseC)
{
    *phaseA = csa_v[0];
    *phaseB = csa_v[1];
    *phaseC = csa_v[2];
}

float adc_get_vsense_v(void)
{
    return vsense_v;
}

uint8_t adc_get_smpidx(void)
{
    return smpidx;
}
