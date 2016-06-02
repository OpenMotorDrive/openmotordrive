#include "drv.h"
#include "timing.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>

void drv_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOF);
    gpio_mode_setup(GPIOF, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO1); // nFault pin

    drv_write_register_bits(0x7,7,8,0b01); // 3-PWM mode
    drv_write_register_bits(0x9,7,7,0b1); // enable sense amplifier clamp (protects ADC pins on STM)
    drv_write_register_bits(0x5,4,7,0b1010); // high-side gate driver peak sink current = 1.0A
    drv_write_register_bits(0x5,0,3,0b1011); // high-side gate driver peak source current = 1.0A
    drv_write_register_bits(0x6,4,7,0b1010); // low-side gate driver peak sink current = 1.0A
    drv_write_register_bits(0x6,0,3,0b1011); // low-side gate driver peak source current = 1.0A
    drv_write_register_bits(0xA,0,5,0b111111); // all CS amplifier gains = 80
    drv_write_register_bits(0xA,6,7,0b00); // current shunt blanking time 00=0us 01=0.5us 10=2.5us 11=10us
    drv_write_register_bits(0xC,3,7,0b11101); // VDS comparator threshold 1.892V

    drv_write_register_bits(0x9,1,1,0b1); // clear faults
}

uint16_t drv_read_register(uint8_t reg)
{
    uint8_t i;
    spi_disable(SPI3);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO3); // SCK
    spi_set_clock_polarity_0(SPI3);
    spi_set_baudrate_prescaler(SPI3, SPI_CR1_BR_FPCLK_DIV_8); // 9MHz - datasheet says 100ns min clock period
    spi_enable(SPI3);

    uint16_t command = (1U<<15) | ((reg&0xFU)<<11);

    uint16_t ret;
    for(i=0;i<30;i++) __asm__("nop"); // min 400ns
    gpio_clear(GPIOB, GPIO0); // DRV CS down
    for(i=0;i<5;i++) __asm__("nop"); // min 50ns
    ret = spi_xfer(SPI3,command);
    for(i=0;i<5;i++) __asm__("nop"); // min 50ns
    gpio_set(GPIOB, GPIO0); // DRV CS up

    return ret&0x3FFU;
}

void drv_write_register(uint8_t reg, uint16_t val)
{
    uint8_t i;
    val &= 0x3FFU;

    spi_disable(SPI3);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3); // SCK
    spi_set_clock_polarity_0(SPI3);
    spi_set_baudrate_prescaler(SPI3, SPI_CR1_BR_FPCLK_DIV_8); // 9MHz - datasheet says 100ns min clock period
    spi_enable(SPI3);

    uint16_t command = ((reg&0xFU)<<11) | val;

    for(i=0;i<30;i++) __asm__("nop"); // min 400ns
    gpio_clear(GPIOB, GPIO0); // DRV CS down
    for(i=0;i<5;i++) __asm__("nop"); // min 50ns
    spi_xfer(SPI3,command);
    for(i=0;i<5;i++) __asm__("nop"); // min 50ns
    gpio_set(GPIOB, GPIO0); // DRV CS up
}

void drv_write_register_bits(uint8_t reg, uint8_t rng_begin, uint8_t rng_end, uint16_t val)
{
    uint8_t i;
    uint16_t mask = 0;

    for (i=rng_begin;i<=rng_end;i++) {
        mask |= (1U<<i);
    }

    mask &= 0x3FFU;
    val <<= rng_begin;
    val &= mask;

    drv_write_register(reg, (drv_read_register(reg)&~mask)|val);
}

void drv_print_register(uint8_t reg)
{
    uint8_t i;
    char buf[20];
    int n;
    n = sprintf(buf, "0x%X 0b", reg);
    for(i=0; i<n; i++) {
        usart_send_blocking(USART1, buf[i]);
    }

    uint16_t val = drv_read_register(reg);
    for(i=0; i<=10; i++) {
        usart_send_blocking(USART1, ((val&((1<<10)>>i)) != 0) ? '1' : '0');
    }
    usart_send_blocking(USART1, '\n');
}
