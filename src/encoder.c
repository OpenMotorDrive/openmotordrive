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

#include "encoder.h"
#include <common/timing.h>
#include <common/helpers.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <math.h>

static float angle_rad;
static uint32_t meas_t_us;

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

void encoder_read_angle(void)
{
    uint8_t i;

    spi_disable(SPI3);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3); // SCK
    spi_set_clock_polarity_1(SPI3);
    spi_set_baudrate_prescaler(SPI3, SPI_CR1_BR_FPCLK_DIV_4); // 18MHz - datasheet says 40ns min clock period
    spi_enable(SPI3);

    for(i=0;i<2;i++) __asm__("nop"); // min 20ns
    gpio_clear(GPIOA, GPIO5); // MA700 CS down
    for(i=0;i<3;i++) __asm__("nop");  // min 25ns
    meas_t_us = micros();
    angle_rad = 2.0f*M_PI_F*spi_xfer(SPI3,0)/65536.0f;
    for(i=0;i<3;i++) __asm__("nop");  // min 25ns
    gpio_set(GPIOA, GPIO5); // MA700 CS up
}

float encoder_get_angle_rad(void)
{
    return angle_rad;
}
