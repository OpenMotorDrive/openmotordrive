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


#include <esc/program.h>

#include <esc/helpers.h>
#include <esc/motor.h>
#include <esc/timing.h>
#include <esc/serial.h>
#include <esc/drv.h>
#include <libopencm3/stm32/timer.h>

#include <stdio.h>

void program_init(void) {
    motor_set_mode(MOTOR_MODE_PHASE_VOLTAGE_TEST);
}

void program_event_adc_sample(float dt, struct adc_sample_s* adc_sample) {
    motor_update(dt, adc_sample);
//     char msg[50];

//     int len = snprintf(msg, 50, "a=%d b=%d c=%d\n", (int16_t)(TIM1_CCR1-TIM1_CCR2), (int16_t)(TIM1_CCR2-TIM1_CCR3), (int16_t)(TIM1_CCR3-TIM1_CCR1));
//     serial_send_dma(len,msg);


//     motor_print_data(dt);
}
