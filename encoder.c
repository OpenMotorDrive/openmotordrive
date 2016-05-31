#include "encoder.h"
#include "timing.h"
#include "helpers.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <math.h>

float encoder_read_rad(void)
{
    spi_disable(SPI3);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3); // SCK
    spi_set_clock_polarity_1(SPI3);
    spi_enable(SPI3);

    float ret;
    usleep(2);
    gpio_clear(GPIOA, GPIO5); // MA700 CS down
    usleep(2);
    ret = 2.0f*M_PI_F*spi_xfer(SPI3,0)/65536.0f;
    usleep(2);
    gpio_set(GPIOA, GPIO5); // MA700 CS up
    usleep(2);
    return ret;
}
