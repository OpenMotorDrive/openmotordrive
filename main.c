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

static volatile uint32_t system_millis;

int main(void) {
    clock_init();
    timing_init();
    usart_init();
    spi_init();
    drv_init();
    pwm_init();

    while(1) {
        float a=0.0f,b=0.0f,c=0.0f;
        float freq = 5.0f;
        float phase = freq*2.0f*M_PI*micros()*1e-6f;
        float alpha = sinf(phase);
        float beta = cosf(phase);
        svgen(alpha, beta, &a, &b, &c);
        set_pwm_duty(TIM_OC1, a);
        set_pwm_duty(TIM_OC2, b);
        set_pwm_duty(TIM_OC3, c);
    }

    return 0;
}
