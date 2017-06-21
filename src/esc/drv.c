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

#include <esc/drv.h>
#include <esc/timing.h>
#include <esc/uavcan.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>

enum drv_reg0x1_bit_t {
    DRV_REG0x1_BIT_OTW=0,
    DRV_REG0x1_BIT_TEMP_FLAG3=1,
    DRV_REG0x1_BIT_TEMP_FLAG2=2,
    DRV_REG0x1_BIT_TEMP_FLAG1=3,
    DRV_REG0x1_BIT_VCHP_UVFL=4,
    DRV_REG0x1_BIT_VDS_STATUS=5,
    DRV_REG0x1_BIT_PVDD_OVFL=6,
    DRV_REG0x1_BIT_PVDD_UVFL=7,
    DRV_REG0x1_BIT_TEMP_FLAG4=8,
    // 9 reserved
    DRV_REG0x1_BIT_FAULT=10,
};

enum drv_reg0x2_bit_t {
    DRV_REG0x2_BIT_SNS_A_OCP=0,
    DRV_REG0x2_BIT_SNS_B_OCP=1,
    DRV_REG0x2_BIT_SNS_C_OCP=2,
    // 3 reserved
    // 4 reserved
    DRV_REG0x2_BIT_VDS_LC=5,
    DRV_REG0x2_BIT_VDS_HC=6,
    DRV_REG0x2_BIT_VDS_LB=7,
    DRV_REG0x2_BIT_VDS_HB=8,
    DRV_REG0x2_BIT_VDS_LA=9,
    DRV_REG0x2_BIT_VDS_HA=10,
};

enum drv_reg0x3_bit_t {
    DRV_REG0x3_BIT_VCPH_OVLO_ABS=0,
    DRV_REG0x3_BIT_VCPH_OVLO=1,
    DRV_REG0x3_BIT_VCPH_UVLO2=2,
    // 3 reserved
    DRV_REG0x3_BIT_VCP_LSD_UVLO2=4,
    DRV_REG0x3_BIT_AVDD_UVLO=5,
    DRV_REG0x3_BIT_VREG_UV=6,
    // 7 reserved
    DRV_REG0x3_BIT_OTSD=8,
    DRV_REG0x3_BIT_WD_FAULT=9,
    DRV_REG0x3_BIT_PVDD_UVLO2=10,
};

enum drv_reg0x4_bit_t {
    // 0 reserved
    // 1 reserved
    // 2 reserved
    // 3 reserved
    // 4 reserved
    DRV_REG0x4_BIT_VGS_LC=5,
    DRV_REG0x4_BIT_VGS_HC=6,
    DRV_REG0x4_BIT_VGS_LB=7,
    DRV_REG0x4_BIT_VGS_HB=8,
    DRV_REG0x4_BIT_VGS_LA=9,
    DRV_REG0x4_BIT_VGS_HA=10,
};

static const char* drv_reg0x1_names[] = {
    "OTW",
    "TEMP_FLAG3",
    "TEMP_FLAG2",
    "TEMP_FLAG1",
    "VCHP_UVFL",
    "VDS_STATUS",
    "PVDD_OVFL",
    "PVDD_UVFL",
    "TEMP_FLAG4",
    "reg1bit9",
    "FAULT",
};

static const char* drv_reg0x2_names[] = {
    "SNS_A_OCP",
    "SNS_B_OCP",
    "SNS_C_OCP",
    "reg2bit3",
    "reg2bit4",
    "VDS_LC",
    "VDS_HC",
    "VDS_LB",
    "VDS_HB",
    "VDS_LA",
    "VDS_HA",
};

static const char* drv_reg0x3_names[] = {
    "VCPH_OVLO_ABS",
    "VCPH_OVLO",
    "VCPH_UVLO2",
    "reg3bit3",
    "VCP_LSD_UVLO2",
    "AVDD_UVLO",
    "VREG_UV",
    "reg3bit7",
    "OTSD",
    "WD_FAULT",
    "PVDD_UVLO2",
};

static const char* drv_reg0x4_names[] = {
    "reg4bit0",
    "reg4bit1",
    "reg4bit2",
    "reg4bit3",
    "reg4bit4",
    "VGS_LC",
    "VGS_HC",
    "VGS_LB",
    "VGS_HB",
    "VGS_LA",
    "VGS_HA",
};

void drv_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOF);
    gpio_mode_setup(GPIOF, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO1); // nFault pin

    drv_write_register_bits(0x7,7,8,0b00);     // 6-PWM mode
    drv_write_register_bits(0x9,7,7,0b1);      // enable sense amplifier clamp (protects ADC pins on STM)
    drv_write_register_bits(0x5,4,7,0b0111);   // high-side gate driver peak sink current = 0.25A
    drv_write_register_bits(0x5,0,3,0b0111);   // high-side gate driver peak source current = 0.125A
    drv_write_register_bits(0x6,4,7,0b0111);   // low-side gate driver peak sink current = 0.25A
    drv_write_register_bits(0x6,0,3,0b0111);   // low-side gate driver peak source current = 0.125A
    drv_write_register_bits(0x7,4,6,0b000);    // Dead time 35ns
    drv_write_register_bits(0xA,0,5,0b000000); // all CS amplifier gains = 10
    drv_write_register_bits(0xA,6,7,0b00);     // current shunt blanking time 00=0us 01=0.5us 10=2.5us 11=10us
    drv_write_register_bits(0xC,0,2,0b000);    // Latched shutdown when over-current detected
    drv_write_register_bits(0xC,3,7,0b10101);  // VDS comparator threshold 0.730V

    drv_write_register_bits(0x9,1,1,0b1);      // clear faults
}

float drv_get_csa_gain()
{
    return 10;
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

void drv_print_faults(void) {
    uint8_t i;
    uint16_t reg_val;

    reg_val = drv_read_register(0x1);
    for(i=0; i<=10; i++) {
        if ((reg_val&(1<<i)) != 0) {
            uavcan_send_debug_logmessage(UAVCAN_LOGLEVEL_ERROR, "", drv_reg0x1_names[i]);
        }
    }

    reg_val = drv_read_register(0x2);
    for(i=0; i<=10; i++) {
        if ((reg_val&(1<<i)) != 0) {
            uavcan_send_debug_logmessage(UAVCAN_LOGLEVEL_ERROR, "", drv_reg0x2_names[i]);
        }
    }

    reg_val = drv_read_register(0x3);
    for(i=0; i<=10; i++) {
        if ((reg_val&(1<<i)) != 0) {
            uavcan_send_debug_logmessage(UAVCAN_LOGLEVEL_ERROR, "", drv_reg0x3_names[i]);
        }
    }

    reg_val = drv_read_register(0x4);
    for(i=0; i<=10; i++) {
        if ((reg_val&(1<<i)) != 0) {
            uavcan_send_debug_logmessage(UAVCAN_LOGLEVEL_ERROR, "", drv_reg0x4_names[i]);
        }
    }
}

void drv_print_register(uint8_t reg)
{
    uint16_t val = drv_read_register(reg);
    char msg[32];
    snprintf(msg, 32, "0x%02X 0x%04X\n", reg, val);
    uavcan_send_debug_logmessage(UAVCAN_LOGLEVEL_DEBUG, "", msg);
}

bool drv_get_fault(void)
{
    return (drv_read_register(0x1) & (1<<10)) != 0;
    //return !gpio_get(GPIOF, GPIO1);
}

void drv_csa_cal_mode_on(void)
{
    drv_write_register_bits(0xA, 8, 10, 0b111UL);
}

void drv_csa_cal_mode_off(void)
{
    drv_write_register_bits(0xA, 8, 10, 0b000UL);
}

void drv_3_pwm_mode(void)
{
    drv_write_register_bits(0x7,7,8,0b01); // 3-PWM mode
}

void drv_6_pwm_mode(void)
{
    drv_write_register_bits(0x7,7,8,0b00); // 6-PWM mode
}
