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
#include "timing.h"
#include "init.h"
#include "helpers.h"
#include "serial.h"
#include "param.h"
#include "adc.h"
#include "pwm.h"
#include "drv.h"
#include "motor.h"
#include "encoder.h"
#include "can.h"

#include "config.h"

// Called by the scheduler once at startup
static void prg_setup(void) {
    // TODO separate main files for each program
    switch(CONFIG_PROGRAM) {
        case PROGRAM_CAN_SERVO:
        case PROGRAM_SERVO_TEST:
        case PROGRAM_SPIN_TEST:
            motor_set_mode(MOTOR_MODE_ENCODER_CALIBRATION);
            break;
        case PROGRAM_PHASE_OUTPUT_TEST:
            motor_set_mode(MOTOR_MODE_PHASE_VOLTAGE_TEST);
            break;
    }
}

static void prg_servo_run(float dt, float theta) {
    static float id_ref_filt;
    const float tc = 0.002f;
    const float ang_P = 3.0f;
    const float ang_D = 0.01f;
    float alpha = dt/(dt+tc);

    id_ref_filt += ((wrap_pi(theta-motor_get_phys_rotor_angle())*ang_P-motor_get_phys_rotor_ang_vel()*ang_D) - id_ref_filt) * alpha;
    motor_set_id_ref(id_ref_filt);

    if (motor_get_mode() == MOTOR_MODE_DISABLED) {
        motor_set_mode(MOTOR_MODE_FOC_CURRENT);
    }

}

// Called by the scheduler when there is new ADC data
static void prg_loop(float dt) {
    motor_update_state(dt);

    // TODO separate main files for each program
    switch(CONFIG_PROGRAM) {
        case PROGRAM_PHASE_OUTPUT_TEST:
        case PROGRAM_PRINT_DRV_FAULTS: {
            static uint32_t last_print_ms = 0;
            uint32_t tnow_ms = millis();
            if (tnow_ms-last_print_ms > 500) {
                char buf[200];
                int n = 0;
                uint8_t reg;
                for (reg=1; reg<=4; reg++) {
                    uint16_t reg_val = drv_read_register(reg);
                    n += sprintf(buf+n, "0x%X 0b", reg);
                    int8_t i;
                    for (i=10; i>=0; i--) {
                        buf[n++] = ((reg_val>>i)&1) ? '1' : '0';
                    }
                    buf[n++] = '\n';
                }
                buf[n++] = '\n';
                serial_send_dma(n, buf);
                last_print_ms = tnow_ms;
            }
            break;
        }
        case PROGRAM_PRINT_ENCODER: {
            static uint32_t last_print_ms = 0;
            uint32_t tnow_ms = millis();
            if (tnow_ms-last_print_ms > 100) {
                char buf[20];
                int n;
                n = sprintf(buf, "% .2f\n", encoder_get_angle_rad() * 180.0f/M_PI_F);
                serial_send_dma(n, buf);
                last_print_ms = tnow_ms;
            }
            break;
        }
        case PROGRAM_SPIN_TEST: {
            motor_set_id_ref(0.0f);

            if (motor_get_mode() == MOTOR_MODE_DISABLED) {
                motor_set_mode(MOTOR_MODE_FOC_CURRENT);
            }
            break;
        }
        case PROGRAM_CAN_SERVO: {
            static float theta = 0;

            struct canbus_msg m;
            while (canbus_recv_message(&m)) {
                if (m.id == 500 && !m.rtr && m.dlc == 4) {
                    memcpy(&theta, m.data, 4);
                }
            }

            prg_servo_run(dt, theta);

            break;
        }
        case PROGRAM_SERVO_TEST: {
            prg_servo_run(dt, (millis()/1000)%2 == 0 ? 0.0f : M_PI_F/2.0f);

            break;
        }
        case PROGRAM_PRINT_INPUT_VOLTAGE: {
            break;
        }
        case PROGRAM_SEND_CAN_ANGLE: {
            static uint32_t last_send_us = 0;
            uint32_t tnow_us = micros();
            if (tnow_us-last_send_us > 1000) {
                struct canbus_msg m;
                m.id = 500;
                m.ide = false;
                m.rtr = false;
                m.dlc = 4;
                float theta = encoder_get_angle_rad();
                memcpy(m.data, &theta, 4);

                canbus_send_message(&m);
                last_send_us = tnow_us;
            }
            break;
        }
    }

    motor_run_commutation(dt);
}

int main(void)
{
    uint32_t last_t_us = 0;
    uint8_t prev_smpidx = 0;

    clock_init();
    timing_init();
    serial_init();
    canbus_init();
    param_init();
    spi_init();
    drv_init();
    pwm_init();
    adc_init();
    usleep(100000);
    motor_init();

    prg_setup();

    // main loop
    while(1) {
        // wait specified time for adc measurement
        uint8_t smpidx, d_smp;
        uint32_t t1_us = micros();
        do {
            encoder_read_angle();
            smpidx = adc_get_smpidx();
            d_smp = smpidx-prev_smpidx;
        } while (d_smp < 3);
        prev_smpidx = smpidx;
        float dt = d_smp*adc_get_smp_period();

        uint32_t tnow_us = micros();
        float dt_real = (tnow_us-last_t_us)*1.0e-6f;
        last_t_us = tnow_us;
        uint8_t usagepct = (1.0f - (tnow_us-t1_us)*1.0e-6f / dt)*100.0f;

        prg_loop(dt);
    }

    return 0;
}
