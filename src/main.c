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
#include <esc/motor.h>
#include <esc/encoder.h>
#include <esc/can.h>

#include <esc/programs.h>

#ifndef CONFIG_PROGRAM
    #define CONFIG_PROGRAM PROGRAM_SERVO_TEST
#endif

int main(void)
{
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

    program_init(CONFIG_PROGRAM);

    // main loop
    while(1) {
        // wait specified time for adc measurement
        uint8_t smpidx, d_smp;
        do {
            encoder_read_angle();
            smpidx = adc_get_smpidx();
            d_smp = smpidx-prev_smpidx;
        } while (d_smp < 3);
        prev_smpidx = smpidx;
        float dt = d_smp*adc_get_smp_period();

        program_event_adc_sample(dt);
    }

    return 0;
}
