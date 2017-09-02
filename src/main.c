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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <common/timing.h>
#include <common/init.h>
#include <common/helpers.h>
#include <common/shared_app_descriptor.h>
#include "serial.h"
#include "param.h"
#include "adc.h"
#include "pwm.h"
#include "drv.h"
#include "inverter.h"
#include "motor.h"
#include "encoder.h"
#include <common/can.h>
#include "uavcan.h"
#include <stdio.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

static volatile const struct shared_app_parameters_s shared_app_parameters __attribute__((section(".app_descriptor"),used)) = {
    .boot_delay_sec = 5,
    .canbus_disable_auto_baud = true,
    .canbus_baudrate = 1000000,
    .canbus_local_node_id = 0
};

static volatile const struct shared_app_descriptor_s shared_app_descriptor __attribute__((section(".app_descriptor"),used)) = {
    .signature = SHARED_APP_DESCRIPTOR_SIGNATURE,
    .image_crc = 0,
    .image_size = 0,
    .vcs_commit = GIT_HASH,
    .major_version = 1,
    .minor_version = 0,
    .parameters_fmt = SHARED_APP_PARAMETERS_FMT,
    .parameters_ignore_crc64 = true,
    .parameters = {&shared_app_parameters, 0}
};

static bool restart_req = false;
static uint32_t restart_req_us = 0;

static bool restart_request_handler(void)
{
    restart_req = true;
    restart_req_us = micros();
    return true;
}

static uint32_t tbegin_us;
static bool waiting_to_start = false;
static bool started = false;
static float t_max = 20.0f;

static void spi_init(void)
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


static void test_steps_and_reversals_setup(void)
{
    tbegin_us = micros();
    serial_send_dma(1, "\x55");
}

static void test_steps_and_reversals_loop(void)
{
    if (motor_get_mode() == MOTOR_MODE_ENCODER_CALIBRATION) {
        return;
    }
    if (motor_get_mode() != MOTOR_MODE_DISABLED) {
        motor_print_data();
    }

    uint32_t tnow = micros();
    float t = (tnow-tbegin_us)*1.0e-6f;

    if (motor_get_mode() == MOTOR_MODE_DISABLED && !waiting_to_start && !started) {
        waiting_to_start = true;
        tbegin_us = micros();
    } else if (waiting_to_start && !started && t > 0.1f) {
        tbegin_us = micros();
        started = true;
        motor_set_mode(MOTOR_MODE_FOC_DUTY);
    } else if (started && t >= t_max && motor_get_mode() != MOTOR_MODE_DISABLED) {
        motor_set_mode(MOTOR_MODE_DISABLED);
    }

    if (t < 3) {
        motor_set_duty_ref(0.08f);
    } else if (t < 11.5) {
        float thr = ((uint32_t)((t-1)*2))*0.025f;
        motor_set_duty_ref((((uint32_t)(t*4))%2)==0 ? 0.08f : thr);
    } else if (t < 20.0) {
        float thr = ((uint32_t)((t-9.5)*2))*0.025f;
        motor_set_duty_ref((((uint32_t)(t*4))%2)==0 ? thr : -thr);
    }
}

static uint8_t esc_index;
static uint32_t last_command_us;
static uint32_t last_status_us;

static void handle_uavcan_esc_rawcommand(uint8_t len, int16_t* commands) {
    const float min_duty = 0.08;
    if (esc_index < len) {
        if (commands[esc_index] == 0) {
            motor_set_mode(MOTOR_MODE_DISABLED);
        } else {
            motor_set_mode(MOTOR_MODE_FOC_DUTY);
            if (commands[esc_index] > 0) {
                motor_set_duty_ref(commands[esc_index]/8191.0f*(1.0f-min_duty)+min_duty);
            } else {
                motor_set_duty_ref(commands[esc_index]/8191.0f*(1.0f-min_duty)-min_duty);
            }
        }
        last_command_us = micros();
    }
}

static void uavcan_esc_setup(void)
{
    esc_index = (uint8_t)*param_retrieve_by_name("uavcan.id-uavcan.equipment.esc-esc_index");
    uavcan_set_esc_rawcommand_cb(handle_uavcan_esc_rawcommand);

    motor_set_mode(MOTOR_MODE_DISABLED);
}

static void uavcan_esc_loop(void)
{
    uint32_t tnow_us = micros();

    if (tnow_us-last_command_us > 0.5*1e6) {
        motor_set_mode(MOTOR_MODE_DISABLED);
    }

    if (tnow_us - last_status_us > 0.25*1e6) {
        // TODO: send ESC status message
    }
}

enum program_t {
    PROGRAM_NONE = 0,
    PROGRAM_UAVCAN_ESC,
    PROGRAM_ENCODER_CALIBRATION,
};

static enum program_t program_selected;

static void setup(void)
{
    float* program_sel_param = param_retrieve_by_name("ESC_PROGRAM_SELECT");
    program_selected = (enum program_t)*program_sel_param;

    switch (program_selected) {
        case PROGRAM_NONE:
            break;
        case PROGRAM_UAVCAN_ESC:
            uavcan_esc_setup();
            break;
        case PROGRAM_ENCODER_CALIBRATION:
            motor_set_mode(MOTOR_MODE_ENCODER_CALIBRATION);
            *program_sel_param = PROGRAM_NONE;
            break;
    }
}

static void loop(void)
{
    switch (program_selected) {
        case PROGRAM_NONE:
            break;
        case PROGRAM_UAVCAN_ESC:
            uavcan_esc_loop();
            break;
        case PROGRAM_ENCODER_CALIBRATION:
            break;
    }

}

int main(void)
{
    uint32_t last_print_ms = 0;

    init_clock();
    timing_init();
    param_init();
    serial_init();
    canbus_init(1000000, false);
    uavcan_init();
    spi_init();
    drv_init();
    adc_init();
    pwm_init();
    usleep(100000);
    inverter_init();
    motor_init();

    uavcan_set_restart_cb(restart_request_handler);

    setup();

    // main loop
    while(1) {
        // wait specified time for adc measurement
        if (motor_update()) {
            loop();
        }
        uavcan_update();

        uint32_t tnow_ms = millis();
        if (tnow_ms-last_print_ms >= 2000) {
            drv_print_faults();
            last_print_ms = tnow_ms;
        }

        if (restart_req && (micros() - restart_req_us) > 1000) {
            // reset
            scb_reset_system();
        }
    }

    return 0;
}
