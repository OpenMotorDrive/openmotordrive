#include "init.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/flash.h>

void clock_init(void)
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

void usart_init(void)
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

void spi_init(void)
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

