//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include "timing.h"
#include <stdio.h>
#include <math.h>

static volatile uint32_t system_millis;

static void clock_setup(void);
static void usart_setup(void);
static void spi_setup(void);
static void usleep(uint32_t delay);
static float read_encoder(void);
static uint16_t drv_read_register(uint8_t reg);
static void drv_write_register(uint8_t reg, uint16_t val);
static void drv_write_register_bits(uint8_t reg, uint8_t rng_begin, uint8_t rng_end, uint16_t val);
static void drv_print_register(uint8_t reg);
static void configure_drv(void);

static void usleep(uint32_t delay) {
    uint32_t tbegin = micros();
    while (micros()-tbegin < delay);
}

int main(void) {
    uint8_t i;
    clock_setup();
    timing_init();
    usart_setup();
    spi_setup();
    configure_drv();
    usleep(1000);
    usart_send_blocking(USART1, '\n');
    usart_send_blocking(USART1, '\n');
    drv_print_register(0x5);
    drv_print_register(0x6);

    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8|GPIO9|GPIO10);

    rcc_periph_clock_enable(RCC_GPIOF);
    gpio_mode_setup(GPIOF, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO1);

    while(1) {
        usleep(1000000);
        //drv_write_register_bits(0x9,1,1,0b1); // clear faults
        gpio_set(GPIOA,GPIO8);
        gpio_clear(GPIOA,GPIO9);
        gpio_clear(GPIOA,GPIO10);
        usleep(1000000);
        gpio_clear(GPIOA,GPIO8);
        gpio_set(GPIOA,GPIO9);
        gpio_clear(GPIOA,GPIO10);
        usleep(1000000);
        gpio_clear(GPIOA,GPIO8);
        gpio_clear(GPIOA,GPIO9);
        gpio_set(GPIOA,GPIO10);
        usart_send_blocking(USART1, '\n');
        drv_print_register(0x1);
        drv_print_register(0x2);
        drv_print_register(0x3);
        drv_print_register(0x4);


        /*char buf[50];
        int n;
        n = sprintf(buf, "%.2f\n", read_encoder()*180.0f/M_PI);
        for(i=0; i<n; i++) {
            usart_send_blocking(USART1, buf[i]);
        }*/
    }

    usleep(100);

    /*while(1) {

        n = sprintf(buf, "%u\n", drv_read_register(0x1));
        for(i=0; i<n; i++) {
            usart_send_blocking(USART1, buf[i]);
        }
        usleep(1000);
    }*/
    return 0;
}

static void configure_drv(void)
{
    drv_write_register_bits(0x7,7,8,0b01); // 3-PWM mode
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

static uint16_t drv_read_register(uint8_t reg)
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

static void drv_write_register(uint8_t reg, uint16_t val)
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

static void drv_write_register_bits(uint8_t reg, uint8_t rng_begin, uint8_t rng_end, uint16_t val)
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

static void drv_print_register(uint8_t reg)
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

static float read_encoder(void)
{
    spi_disable(SPI3);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3); // SCK
    spi_set_clock_polarity_1(SPI3);
    spi_enable(SPI3);

    float ret;
    usleep(2);
    gpio_clear(GPIOA, GPIO5); // MA700 CS down
    usleep(2);
    ret = 2.0f*M_PI*spi_xfer(SPI3,0)/65536.0f;
    usleep(2);
    gpio_set(GPIOA, GPIO5); // MA700 CS up
    usleep(2);
    return ret;
}

static void clock_setup(void)
{
    rcc_osc_on(RCC_HSE);
    rcc_wait_for_osc_ready(RCC_HSE);
    rcc_set_sysclk_source(RCC_CFGR_SW_HSE);
    rcc_wait_for_sysclk_status(RCC_HSE);

    rcc_osc_off(RCC_PLL);
    rcc_wait_for_osc_not_ready(RCC_PLL);
    rcc_set_prediv(RCC_CFGR2_PREDIV_NODIV); // 8 Mhz
    rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_PREDIV);
    rcc_set_pll_multiplier(RCC_CFGR_PLLMUL_PLL_IN_CLK_X9); // 72 MHz

    rcc_set_hpre(RCC_CFGR_HPRE_DIV_NONE); // 72 MHz
    rcc_ahb_frequency = 72000000;

    rcc_set_ppre1(RCC_CFGR_PPRE1_DIV_2); // 36 MHz
    rcc_apb1_frequency = 36000000;

    rcc_set_ppre2(RCC_CFGR_PPRE2_DIV_NONE); // 72 MHz
    rcc_apb2_frequency = 72000000;

    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);
    flash_set_ws(FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2WS);
    rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
    rcc_wait_for_sysclk_status(RCC_PLL);

}

static void usart_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6|GPIO7);
    gpio_set_af(GPIOB, GPIO_AF7, GPIO6|GPIO7);
    const uint32_t baud = 115200;
    uint32_t clock = rcc_apb1_frequency;
    USART_BRR(USART1) = ((2 * clock) + baud) / (2 * baud);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}

static void spi_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_SPI3);

    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5); // MA700 CS
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0); // DRV CS
    gpio_set(GPIOA, GPIO5); // MA700 CS up
    gpio_set(GPIOB, GPIO0); // DRV CS up

    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4|GPIO5); // MISO,MOSI
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3); // SCK
    gpio_set_af(GPIOB, GPIO_AF6, GPIO3|GPIO4|GPIO5);

    spi_set_baudrate_prescaler(SPI3, SPI_CR1_BR_FPCLK_DIV_4); // 18MHz
    spi_set_clock_polarity_1(SPI3);
    spi_set_clock_phase_1(SPI3);
    spi_set_full_duplex_mode(SPI3);
    spi_set_unidirectional_mode(SPI3);
    spi_set_data_size(SPI3, SPI_CR2_DS_16BIT);
    spi_send_msb_first(SPI3);
    spi_enable_software_slave_management(SPI3);
    spi_set_nss_high(SPI3);
    spi_fifo_reception_threshold_16bit(SPI3);
    spi_set_master_mode(SPI3);
    spi_enable(SPI3);
}

void usart1_exti25_isr(void) {
    if (usart_get_interrupt_source(USART1, USART_ISR_RXNE)) {
        uint16_t data = usart_recv(USART1);
        usart_send(USART1, data);
    }
    usart_enable_rx_interrupt(USART1);
}
