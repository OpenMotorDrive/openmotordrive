#include "serial.h"

#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>

#define TXBUF_LEN 256
#define RXBUF_LEN 256

static char rxbuf[RXBUF_LEN];
static char txbuf[TXBUF_LEN];
uint16_t txbuf_tail;

void serial_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_DMA1);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6|GPIO7);
    gpio_set_af(GPIOB, GPIO_AF7, GPIO6|GPIO7);
    const uint32_t baud = 921600;

    uint32_t clock = rcc_apb1_frequency;
    USART_BRR(USART1) = ((2 * clock) + baud) / (2 * baud);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    // set up DMA xmit
    USART_CR3(USART1) |= 1UL<<7; // enable DMA for transmission
    DMA_CPAR(DMA1,DMA_CHANNEL4) = (uint32_t)&USART_TDR(USART1);
    DMA_CCR(DMA1,DMA_CHANNEL4) |= 0b01UL<<12; // PL medium
    DMA_CCR(DMA1,DMA_CHANNEL4) |= 0b00UL<<10; // MSIZE 8 bits
    DMA_CCR(DMA1,DMA_CHANNEL4) |= 0b00UL<<8; // PSIZE 8 bits
    DMA_CCR(DMA1,DMA_CHANNEL4) |= 1UL<<4; // DIR=1 (read from memory)
    DMA_CCR(DMA1,DMA_CHANNEL4) |= 1UL<<7; // MINC=1

    // set up DMA recv
//     nvic_enable_irq(NVIC_USART1_EXTI25_IRQ);
//     USART_CR1(USART1) |= 1UL<<6; // TCIE=1
//     USART_CR3(USART1) |= 1UL<<6; // enable DMA for receive
//     DMA_CPAR(DMA1,DMA_CHANNEL5) = (uint32_t)&USART_RDR(USART1);
//     DMA_CMAR(DMA1,DMA_CHANNEL5) = (uint32_t)rxbuf;
//     DMA_CCR(DMA1,DMA_CHANNEL5) |= 0b01UL<<12; // PL medium
//     DMA_CCR(DMA1,DMA_CHANNEL5) |= 0b00UL<<10; // MSIZE 8 bits
//     DMA_CCR(DMA1,DMA_CHANNEL5) |= 0b00UL<<8; // PSIZE 8 bits
//     DMA_CCR(DMA1,DMA_CHANNEL5) |= 1UL<<7; // MINC=1
//     DMA_CCR(DMA1,DMA_CHANNEL5) |= 1UL<<5; // CIRC=1
//     DMA_CNDTR(DMA1,DMA_CHANNEL5) = RXBUF_LEN;
//     DMA_CCR(DMA1,DMA_CHANNEL5) |= 1UL<<0; // EN=1

    usart_enable(USART1);
}

bool serial_send_dma(uint16_t len, char* buf)
{
    if ((USART_ISR(USART1)&(1UL<<6)) != 0) {
        DMA_CCR(DMA1,DMA_CHANNEL4) &= ~(1UL<<0); // EN=0
        memcpy(txbuf, buf, len);
        DMA_CMAR(DMA1,DMA_CHANNEL4) = (uint32_t)&(txbuf[0]);
        DMA_CNDTR(DMA1,DMA_CHANNEL4) = len;
        USART_ICR(USART1) |= 1UL<<6; // TCCF = 1
        DMA_CCR(DMA1,DMA_CHANNEL4) |= 1UL<<0; // EN=1
        return true;
    }

    return false;
}

// void usart1_exti25_isr(void)
// {
//
// }
