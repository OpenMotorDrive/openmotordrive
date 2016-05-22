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

    //     drv_write_register_bits(0x9,9,9,0b1); // PVDD_UVLO2 fault disabled
    //     drv_write_register_bits(0x9,8,8,0b1); // gate drive fault disabled
    //     drv_write_register_bits(0x9,4,4,0b1); // SNS overcurrent fault disabled
    //     drv_write_register_bits(0x9,2,2,0b0); // device awake
    //     drv_write_register_bits(0xB,2,2,0b1); // VREG undervoltage fault disabled
    //     drv_write_register_bits(0xC,0,2,0b010); // VDS protection disabled
    drv_write_register_bits(0x9,1,1,0b1); // clear faults
}

uint16_t drv_read_register(uint8_t reg)
{
    spi_disable(SPI3);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO3); // SCK
    spi_set_clock_polarity_0(SPI3);
    spi_enable(SPI3);

    uint16_t command = (1U<<15) | ((reg&0xFU)<<11);

    uint16_t ret;
    usleep(2);
    gpio_clear(GPIOB, GPIO0); // DRV CS down
    usleep(2);
    ret = spi_xfer(SPI3,command);
    usleep(2);
    gpio_set(GPIOB, GPIO0); // DRV CS up
    usleep(2);

    return ret&0x3FFU;
}

void drv_write_register(uint8_t reg, uint16_t val)
{
    val &= 0x3FFU;

    spi_disable(SPI3);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3); // SCK
    spi_set_clock_polarity_0(SPI3);
    spi_enable(SPI3);

    uint16_t command = ((reg&0xFU)<<11) | val;

    usleep(2);
    gpio_clear(GPIOB, GPIO0); // DRV CS down
    usleep(2);
    spi_xfer(SPI3,command);
    usleep(2);
    gpio_set(GPIOB, GPIO0); // DRV CS up
    usleep(2);
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
