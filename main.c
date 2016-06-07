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
#include <stdio.h>
#include <math.h>
#include "timing.h"
#include "init.h"
#include "helpers.h"
#include "serial.h"
#include "adc.h"
#include "pwm.h"
#include "drv.h"
#include "motor.h"
#include "encoder.h"

static void idle_task(void) {
    // will read UART and process commands here
    char byte;
    if (serial_recv_peek(&byte)) {
        byte += 1;
        if (serial_send_dma(1, &byte)) {
            serial_recv_pop(&byte);
        }
    }
    encoder_read_angle();
}

static void main_loop(float dt) {
    static float id_ref_filt;
    motor_update_state(dt);

//     const float tc = 0.002f;
//     const float ang_P = 20.0f;
//     const float ang_D = 0.1f;
//     float alpha = dt/(dt+tc);
//     float pos_dem = M_PI_F/2.0f*sinf(2.0f*2.0f*M_PI_F*millis()*1.0e-3f);
//     float pos_dem = (millis()/500)%2 ? 0.0f : M_PI_F/2.0f;
//     float pos_dem = 0.0f;
//     id_ref_filt += ((wrap_pi(pos_dem-motor_get_phys_rotor_angle())*ang_P-motor_get_phys_rotor_ang_vel()*ang_D) - id_ref_filt) * alpha;
//     motor_set_id_ref(id_ref_filt);
    motor_set_id_ref(0.5f);

    motor_run_commutation(dt);

    if (motor_get_mode() == MOTOR_MODE_DISABLED) {
        motor_set_mode(MOTOR_MODE_FOC_CURRENT);
    }
}

int main(void)
{
    uint32_t last_t_us = 0;
    uint32_t last_print_t = 0;
    uint8_t prev_smpidx = 0;

    clock_init();
    timing_init();
    serial_init();
    param_init();
    spi_init();
    drv_init();
    pwm_init();
    adc_init();
    usleep(100000);
    motor_init();
    motor_set_mode(MOTOR_MODE_ENCODER_CALIBRATION);

    // main loop
    while(1) {
        // wait specified time for adc measurement
        uint8_t smpidx, d_smp;
        uint32_t t1_us = micros();
        do {
            idle_task();
            smpidx = adc_get_smpidx();
            d_smp = smpidx-prev_smpidx;
        } while (d_smp < 3);
        prev_smpidx = smpidx;
        float dt = d_smp*1.0f/18000.0f;

        uint32_t tnow_us = micros();
        float dt_real = (tnow_us-last_t_us)*1.0e-6f;
        last_t_us = tnow_us;
        uint8_t usagepct = (1.0f-(tnow_us-t1_us)*1.0e-6f / dt)*100.0f;

        uint32_t t2 = micros();
        main_loop(dt);
        uint32_t t3 = micros();

        if (millis()-last_print_t > 100) {
            last_print_t = millis();

            char buf[256];
            int n;
            n = snprintf(buf, sizeof(buf), "1 0x%04X\n2 0x%04X\n3 0x%04X\n4 0x%04X\n\n", drv_read_register(0x1),drv_read_register(0x2),drv_read_register(0x3),drv_read_register(0x4));
            serial_send_dma(n, buf);
        }
    }

    return 0;
}
