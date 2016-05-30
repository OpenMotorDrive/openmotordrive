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
#include "serial.h"
#include <math.h>
#include <libopencm3/stm32/usart.h>
#include <stdio.h>
#include <libopencm3/stm32/adc.h>

static float csa_cal[3] = {0,0,0};

static void calibrate_phase_currents(void)
{
    uint8_t i;
    drv_write_register_bits(0xA, 8, 10, 0b111UL);
    for(i=0; i<10; i++) {
        usleep(1000);
        csa_cal[0] += csa_v_get(0);
        csa_cal[1] += csa_v_get(1);
        csa_cal[2] += csa_v_get(2);
    }
    drv_write_register_bits(0xA, 8, 10, 0b000UL);
    csa_cal[0] /= 10;
    csa_cal[1] /= 10;
    csa_cal[2] /= 10;
}

static void get_phase_currents(float* ia, float* ib, float* ic)
{
    float G = 80.0f;
    float R = 0.001f;
    *ia = (csa_v_get(0)-csa_cal[0])/(G*R);
    *ib = (csa_v_get(2)-csa_cal[2])/(G*R);
    *ic = (csa_v_get(1)-csa_cal[1])/(G*R);
}

static void get_phase_volts(float* va, float* vb, float* vc)
{
    float ratio = 18.0f;
    *va = phase_v_get(0)*ratio;
    *vb = phase_v_get(2)*ratio;
    *vc = phase_v_get(1)*ratio;
}

int main(void) {

    char buf[100];
    int n;
    uint8_t i;

    clock_init();
    timing_init();
    serial_init();
    spi_init();
    drv_init();
    pwm_init();
    adc_init();
    usleep(1000000);
    calibrate_phase_currents();

    float cal_angle = -20.0f*M_PI/180.0f;
    float id_ref = 1.4f;
    float iq_ref = 0.0f;

    uint32_t last_print_t = 0;

    while(1) {
        float ia_m, ib_m, ic_m, id_m, iq_m, va_m, vb_m, vc_m, a, b, c;

        // retrieve ADC measurements
        wait_for_adc_sample();
        float vbatt = vsense_v_get()*20.0f;
        get_phase_currents(&ia_m, &ib_m, &ic_m);
        get_phase_volts(&va_m, &vb_m, &vc_m);

        // retrieve encoder measurement
        float d_axis_angle_rad = fmodf(read_encoder_rad()*7.0f+cal_angle, 2.0f*M_PI);
        if (d_axis_angle_rad < 0) {
            d_axis_angle_rad += 2.0f*M_PI;
        }

        dqo_transform(d_axis_angle_rad, ia_m,ib_m,ic_m, &id_m,&iq_m,NULL);

        float vd_err = (id_ref-id_m);
        float vq_err = (iq_ref-iq_m);

        float vd = vd_err*2.0f;
        float vq = vq_err*2.0f;
        float v_len = sqrtf(vd*vd+vq*vq);
        if (v_len > 0.9f)
        {
            vd *= 0.9f/v_len;
            vq *= 0.9f/v_len;
        }

        float alpha = vd*cosf(d_axis_angle_rad) - vq*sin(d_axis_angle_rad);
        float beta = vq*cos(d_axis_angle_rad) + vd*sin(d_axis_angle_rad);

        svgen(alpha, beta, &a, &b, &c);

        set_pwm_duty(0, a);
        set_pwm_duty(2, b);
        set_pwm_duty(1, c);

        if (millis()-last_print_t > 100) {
            last_print_t = millis();

            n = sprintf(buf, "ia =% .2f ib =% .2f ic =% .2f va =% .2f vb =% .2f vc =% .2f vbatt =% .2f\n", ia_m, ib_m, ic_m, va_m, vb_m, vc_m, vbatt);
            for(i=0; i<n; i++) {
                usart_send_blocking(USART1, buf[i]);
            }
        }
    }

    return 0;
}
