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

#include <esc/serial.h>
#include <esc/ringbuf.h>

#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>

#define TXBUF_LEN 256
#define RXBUF_LEN 256

static volatile char _rxbuf[RXBUF_LEN];
static volatile struct ringbuf_t rxbuf = {_rxbuf, RXBUF_LEN, 0, 0};

static char txbuf[TXBUF_LEN];

void serial_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_DMA1);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6|GPIO7);
    gpio_set_af(GPIOB, GPIO_AF7, GPIO6|GPIO7);
//     const uint32_t baud = 4500000;
//     uint32_t clock = rcc_apb1_frequency;
    USART_BRR(USART1) = 0x10;//((2 * clock) + baud) / (2 * baud);
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
    DMA_CCR(DMA1,DMA_CHANNEL4) |= 1UL<<1; // TCIE=1
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);

    // set up recv interrupt
    nvic_enable_irq(NVIC_USART1_EXTI25_IRQ);
    USART_CR1(USART1) |= 1UL<<15; // OVER8
//     USART_CR1(USART1) |= 1UL<<5; // RXNEIE=1

    usart_enable(USART1);
}

void dma1_channel4_isr(void)
{
    if ((DMA_ISR(DMA1)&(1UL<<13)) != 0) {
        // TCIF4 is asserted
        DMA_CCR(DMA1,DMA_CHANNEL4) &= ~(1UL<<0); // EN=0
        DMA_IFCR(DMA1) |= 1UL<<13; // clear interrupt flag
    }
}

void usart1_exti25_isr(void)
{
    if ((USART_ISR(USART1)&(1UL<<5)) != 0) {
        // a byte has been received
        ringbuf_push(&rxbuf, USART_RDR(USART1));
    }
}

volatile struct ringbuf_t* serial_get_rxbuf(void)
{
    return &rxbuf;
}

char* serial_get_txbuf(void)
{
    return txbuf;
}

uint16_t serial_get_txbuf_len(void)
{
    return TXBUF_LEN;
}

bool serial_ready_to_send(void)
{
    return (DMA_CCR(DMA1,DMA_CHANNEL4) & (1UL<<0)) == 0;
}

bool serial_send_dma(uint16_t len, char* buf)
{
    if (len <= TXBUF_LEN && serial_ready_to_send()) {
        memcpy(txbuf, buf, len);
        return serial_send_dma_preloaded(len);
    }

    return false;
}

bool serial_send_dma_preloaded(uint16_t len)
{
    if (len <= TXBUF_LEN && serial_ready_to_send()) {
        DMA_CMAR(DMA1,DMA_CHANNEL4) = (uint32_t)&(txbuf[0]);
        DMA_CNDTR(DMA1,DMA_CHANNEL4) = len;
        USART_ICR(USART1) |= 1UL<<6; // TCCF = 1
        DMA_CCR(DMA1,DMA_CHANNEL4) |= 1UL<<0; // EN=1
        return true;
    }

    return false;
}
