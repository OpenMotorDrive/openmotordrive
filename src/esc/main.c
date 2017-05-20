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
#include <esc/timing.h>
#include <esc/init.h>
#include <esc/helpers.h>
#include <esc/serial.h>
#include <esc/param.h>
#include <esc/adc.h>
#include <esc/pwm.h>
#include <esc/drv.h>
#include <esc/inverter.h>
#include <esc/motor.h>
#include <esc/encoder.h>
#include <esc/can.h>
#include <esc/semihost_debug.h>
#include <esc/uavcan.h>
#include <stdio.h>
#include <libopencm3/cm3/scb.h>

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

static uint8_t gimbal_axis;
static uint32_t gimbal_last_command_us;
static float gimbal_rate_error;
static float gimbal_kP;
static float gimbal_kI;
static float gimbal_integrator;

struct gimbal_rate_error_msg_s {
    int16_t rate_error_int[3];
};

static void gimbal_handle_canbus_msg(struct canbus_msg msg)
{
    if (msg.id == 42 && msg.dlc == sizeof(struct gimbal_rate_error_msg_s)) {
        struct gimbal_rate_error_msg_s* msg_decoded = (struct gimbal_rate_error_msg_s*)msg.data;
        gimbal_last_command_us = micros();
        gimbal_rate_error = ((float)msg_decoded->rate_error_int[gimbal_axis])*500.0/32767.0;

        // respond with joint angle
        float joint_angle = encoder_get_angle_rad();
        struct canbus_msg resp_msg;
        resp_msg.id = 50+gimbal_axis;
        resp_msg.ide = false;
        resp_msg.rtr = false;
        memcpy(resp_msg.data, &joint_angle, sizeof(joint_angle));
        resp_msg.dlc = sizeof(joint_angle);
    }
}

static void gimbal_setup(void)
{
    gimbal_axis = (uint8_t)*param_retrieve_by_name("GIMBAL_AXIS");
    gimbal_kP = *param_retrieve_by_name("GIMBAL_K_P");
    gimbal_kI = *param_retrieve_by_name("GIMBAL_K_I");

    uavcan_set_unhandled_can_frame_cb(gimbal_handle_canbus_msg);
}

static void gimbal_loop(void)
{
    uint32_t tnow_us = micros();

    if (tnow_us-gimbal_last_command_us > 0.5*1e6) {
        motor_set_mode(MOTOR_MODE_DISABLED);
    } else {
        motor_set_mode(MOTOR_MODE_FOC_CURRENT);

        if (!motor_get_saturation_flag()) {
            gimbal_integrator += gimbal_kI*gimbal_rate_error*motor_get_dt();
        }

        motor_set_iq_ref(gimbal_kP * gimbal_rate_error + gimbal_integrator);
    }
}

enum program_t {
    PROGRAM_NONE = 0,
    PROGRAM_UAVCAN_ESC,
    PROGRAM_ENCODER_CALIBRATION,
    PROGRAM_GIMBAL,
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
        case PROGRAM_GIMBAL:
            gimbal_setup();
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
        case PROGRAM_GIMBAL:
            gimbal_loop();
            break;
    }

}

int main(void)
{
    uint32_t last_print_ms = 0;

    clock_init();
    timing_init();
    param_init();
    serial_init();
    canbus_init();
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
