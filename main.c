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

int main(void)
{
    uint32_t last_t_us = 0;
    uint32_t last_print_t = 0;
    uint8_t prev_smpidx = 0;

    clock_init();
    timing_init();
    serial_init();
    spi_init();
    drv_init();
    pwm_init();
    adc_init();
    motor_init();

    // main loop
    while(1) {
        // wait specified time for adc measurement
        uint8_t smpidx, d_smp;
        uint32_t t1_us = micros();
        do {
            smpidx = adc_get_smpidx();
            d_smp = smpidx-prev_smpidx;
        } while (d_smp < 3);
        prev_smpidx = smpidx;
        float dt = d_smp*1.0f/18000.0f;

        uint32_t tnow_us = micros();
        float dt_real = (tnow_us-last_t_us)*1.0e-6f;
        last_t_us = tnow_us;
        uint8_t usagepct = (1.0f-(micros()-t1_us)*1.0e-6f / dt)*100.0f;

        motor_update_state(dt);

        const float ang_P = 2.0f;
        const float ang_D = 0.15f;
        //float pos_dem = sinf(2.0f*M_PI_F*millis()*1.0e-3f);
        float pos_dem = (millis()/500)%2 ? 0.0f : M_PI_F/2.0f;
        motor_set_id_ref(wrap_pi(pos_dem-motor_get_phys_rotor_angle())*ang_P-motor_get_phys_rotor_ang_vel()*ang_D);

        motor_run_commutation(dt);

        if (millis()-last_print_t > 10) {
            last_print_t = millis();

            char buf[256];
            int n;
            n = sprintf(buf, "% f,% f,% f,% f\n", motor_get_phys_rotor_angle()*180.0f/M_PI_F, motor_get_elec_rotor_angle()*180.0f/M_PI_F, wrap_2pi(2.0f*M_PI_F*millis()*1e-3f)*180.0f/M_PI_F, wrap_pi(motor_get_elec_rotor_angle()-wrap_2pi(2.0f*M_PI_F*millis()*1e-3f))*180.0f/M_PI_F);
            serial_send_dma(n, buf);
        }
    }

    return 0;
}
