/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "adc.h"
#include <common/timing.h>
#include <string.h>
#include <stdbool.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>

#define SAMPLE_FREQ 18000.0f
#define SAMPLE_PERIOD (1.0f/SAMPLE_FREQ)
#define NUM_CONVERSIONS 4UL

static volatile uint16_t adcbuf[2][NUM_CONVERSIONS];
static volatile uint32_t errcnt = 0;
static volatile uint8_t sample_idx = 0;
static volatile struct adc_sample_s sample[2];
static ISR_ptr new_sample_isr;

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
    // discontinuous group size 4
    ADC_CFGR1(ADC1) |= (4UL-1)<<17;

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
    ADC_SQR1(ADC1) |= 1UL<<6; // phase current A
    ADC_SQR1(ADC1) |= 2UL<<12; // phase current B
    ADC_SQR1(ADC1) |= 3UL<<18; // phase current C

    // discontinuous group 2
    ADC_SQR1(ADC1) |= 4UL<<24; // Vbus

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
    DMA_CNDTR(DMA1,DMA_CHANNEL1) = NUM_CONVERSIONS*2; // N transfers

    DMA_CCR(DMA1,DMA_CHANNEL1) |= 0b11UL<<12; // PL very high
    DMA_CCR(DMA1,DMA_CHANNEL1) |= 0b01UL<<10; // MSIZE 16 bits
    DMA_CCR(DMA1,DMA_CHANNEL1) |= 0b10UL<<8; // PSIZE 32 bits
    DMA_CCR(DMA1,DMA_CHANNEL1) |= 1UL<<7; // MINC=1
    DMA_CCR(DMA1,DMA_CHANNEL1) |= 1UL<<5; // CIRC=1
    DMA_CCR(DMA1,DMA_CHANNEL1) |= 1UL<<2; // HTIE=1
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
        errcnt++;
        ADC_CR(ADC1) |= 1UL<<4; // ADSTP=1
        while ((ADC_CR(ADC1)&(1UL<<4)) != 0); // wait for ADC to stop
        DMA_CCR(DMA1,DMA_CHANNEL1) &= ~(1UL<<0); // EN=0
        DMA_CNDTR(DMA1,DMA_CHANNEL1) = NUM_CONVERSIONS; // N transfers
        DMA_CCR(DMA1,DMA_CHANNEL1) |= 1UL<<0; // EN=1
        ADC_ISR(ADC1) |= (1UL<<4); // clear interrupt flag
        ADC_CR(ADC1) |= 1UL<<2; // ADSTART=1
    }
}

static void write_sample_buffer(volatile uint16_t* in_buf)
{
    uint8_t write_idx = (sample_idx+1)&1U;

    sample[write_idx].seq = sample[sample_idx].seq+1;
    sample[write_idx].t_us = micros();
    sample[write_idx].csa_v[0] = in_buf[0]*3.3f/4096.0f;
    sample[write_idx].csa_v[1] = in_buf[1]*3.3f/4096.0f;
    sample[write_idx].csa_v[2] = in_buf[2]*3.3f/4096.0f;
    sample[write_idx].vsense_v = in_buf[3]*3.3f/4096.0f;

    sample_idx = write_idx;
}

void dma1_channel1_isr(void)
{
    if ((DMA_ISR(DMA1)&(1UL<<2)) != 0) {
        // HTIF1 is asserted
        write_sample_buffer(adcbuf[0]);
        DMA_IFCR(DMA1) |= 1UL<<2; // clear interrupt flag
    }
    if ((DMA_ISR(DMA1)&(1UL<<1)) != 0) {
        // TCIF1 is asserted
        write_sample_buffer(adcbuf[1]);
        DMA_IFCR(DMA1) |= 1UL<<1; // clear interrupt flag
    }

    if (new_sample_isr) {
        new_sample_isr();
    }
}

void adc_wait_for_sample(void)
{
    uint8_t seq_prev = sample[sample_idx].seq;
    while(sample[sample_idx].seq==seq_prev);
}

volatile struct adc_sample_s* adc_get_sample(void)
{
    return &sample[sample_idx];
}

uint32_t adc_get_errcnt(void)
{
    return errcnt;
}

float adc_get_smp_freq(void)
{
    return SAMPLE_FREQ;
}

float adc_get_smp_period(void)
{
    return SAMPLE_PERIOD;
}

void adc_set_new_sample_isr(ISR_ptr cb)
{
    new_sample_isr = cb;
}
