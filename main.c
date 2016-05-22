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
#include <math.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>

static volatile uint32_t system_millis;

int main(void) {
    clock_init();
    timing_init();
    usart_init();
    spi_init();
    drv_init();
    pwm_init();

    while(1) {
        float angle_rad = read_encoder_rad();
        float a=0.0f,b=0.0f,c=0.0f;
        float freq = 5.0f;
        float phase = freq*2.0f*M_PI*micros()*1e-6f;
        float alpha = sinf(phase);
        float beta = cosf(phase);
        svgen(alpha, beta, &a, &b, &c);

        /*char buf[20];
        int n;
        n = sprintf(buf, "%.3f %.3f %.3f\n", a,b,c);
        uint8_t i;
        for(i=0; i<n; i++) {
            usart_send_blocking(USART1, buf[i]);
        }*/

        a = constrain_float(a,0.0f,1.0f);
        b = constrain_float(b,0.0f,1.0f);
        c = constrain_float(c,0.0f,1.0f);

        set_pwm_duty(TIM_OC1, a);
        set_pwm_duty(TIM_OC2, b);
        set_pwm_duty(TIM_OC3, c);


        drv_print_register(0x1);
        drv_print_register(0x2);
        drv_print_register(0x3);
    }

    return 0;
}
