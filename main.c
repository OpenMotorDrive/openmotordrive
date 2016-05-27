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
#include "timing.h"
#include "init.h"
#include "drv.h"
#include "pwm.h"
#include "encoder.h"
#include "helpers.h"
#include "adc.h"
#include <math.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>
#include <libopencm3/stm32/adc.h>

int main(void) {
    int32_t adc_cal = 0;
    char buf[30];
    int n;
    uint8_t i;

    clock_init();
    timing_init();
    usart_init();
    spi_init();
    drv_init();
    pwm_init();
    adc_init();
    usleep(1000000);

    drv_write_register_bits(0xA, 8, 10, 0b111UL);
    for(i=0; i<10; i++) {
        usleep(1000);
        adc_cal += adc_get();
    }
    adc_cal /= 10;
    drv_write_register_bits(0xA, 8, 10, 0b000UL);

    float cal_angle = -10.0f*M_PI/180.0f;
    while(1) {
        float angle_rad = read_encoder_rad()*7.0f-cal_angle;
        float a=0.0f,b=0.0f,c=0.0f;
        float phase = angle_rad+90.0f*M_PI/180.0f;
        float alpha = .25f*sinf(phase);
        float beta = .25f*cosf(phase);

        if ((millis()/1000) % 2 == 0) {
            alpha = beta = 0;
        }
        svgen(alpha, beta, &a, &b, &c);

        a = constrain_float(a, 0.0f, 1.0f);
        b = constrain_float(b, 0.0f, 1.0f);
        c = constrain_float(c, 0.0f, 1.0f);

        set_pwm_duty(TIM_OC1, a);
        set_pwm_duty(TIM_OC2, b);
        set_pwm_duty(TIM_OC3, c);

        n = sprintf(buf, "%d\n", ((int32_t)adc_get())-adc_cal);
        for(i=0; i<n; i++) {
            usart_send_blocking(USART1, buf[i]);
        }

        usleep(1000);
    }

    return 0;
}
