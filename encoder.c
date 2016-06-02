#include "encoder.h"
#include "timing.h"
#include "helpers.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <math.h>

void encoder_write_register(uint8_t regidx, uint8_t value)
{
    uint8_t i;

    uint16_t cmd = (0b0010UL<<12)|(((uint16_t)regidx)<<8)|value;

    spi_disable(SPI3);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3); // SCK
    spi_set_clock_polarity_1(SPI3);
    spi_set_baudrate_prescaler(SPI3, SPI_CR1_BR_FPCLK_DIV_4); // 18MHz - datasheet says 40ns min clock period
    spi_enable(SPI3);

    for(i=0;i<2;i++) __asm__("nop"); // min 20ns
    gpio_clear(GPIOA, GPIO5); // MA700 CS down
    for(i=0;i<3;i++) __asm__("nop");  // min 25ns
    spi_xfer(SPI3,cmd);
    for(i=0;i<3;i++) __asm__("nop");  // min 25ns
    gpio_set(GPIOA, GPIO5); // MA700 CS up
}

uint8_t encoder_read_register(uint8_t regidx)
{
    uint8_t i;
    uint16_t cmd = (0b0001UL<<12)|(((uint16_t)regidx)<<8);
    uint8_t ret;

    spi_disable(SPI3);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3); // SCK
    spi_set_clock_polarity_1(SPI3);
    spi_set_baudrate_prescaler(SPI3, SPI_CR1_BR_FPCLK_DIV_4); // 18MHz - datasheet says 40ns min clock period
    spi_enable(SPI3);

    for(i=0;i<2;i++) __asm__("nop"); // min 20ns
    gpio_clear(GPIOA, GPIO5); // MA700 CS down
    for(i=0;i<3;i++) __asm__("nop");  // min 25ns
    ret = spi_xfer(SPI3,cmd);
    for(i=0;i<3;i++) __asm__("nop");  // min 25ns
    gpio_set(GPIOA, GPIO5); // MA700 CS up
    return ret;
}

float encoder_read_rad(void)
{
    uint8_t i;
    float ret;

    spi_disable(SPI3);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3); // SCK
    spi_set_clock_polarity_1(SPI3);
    //spi_set_baudrate_prescaler(SPI3, SPI_CR1_BR_FPCLK_DIV_4); // 18MHz - datasheet says 40ns min clock period
    spi_enable(SPI3);

    for(i=0;i<2;i++) __asm__("nop"); // min 20ns
    gpio_clear(GPIOA, GPIO5); // MA700 CS down
    for(i=0;i<3;i++) __asm__("nop");  // min 25ns
    ret = 2.0f*M_PI_F*spi_xfer(SPI3,0)/65536.0f;
    for(i=0;i<3;i++) __asm__("nop");  // min 25ns
    gpio_set(GPIOA, GPIO5); // MA700 CS up
    return ret;
}
