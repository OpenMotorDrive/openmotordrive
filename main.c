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

static const float theta_offset = 0.0f*M_PI_F/180.0f/7.0f;
static const float curr_KR = 9.0f;
static const float curr_KP = 30.0f;
static const float curr_KI = 10000.0f;
static const float vsense_div = 20.0f;
static const float csa_G = 80.0f;
static const float csa_R = 0.001f;
static const float max_duty = 0.95f; // required for the current sense amplifiers to work
static const uint8_t n_poles = 7;

static float csa_cal[3] = {0,0,0};
static float dt;
static float id_ref;
static float ia_m, ib_m, ic_m, id_m, iq_m, io_m, phys_theta_m, elec_theta_m, phys_omega_m, vbatt_m;
static float vd, vq;
static float prev_phys_theta_m;
static bool prev_phys_theta_m_set = false;
static bool vd_sat_pos = false;
static bool vd_sat_neg = false;
static bool vq_sat_pos = false;
static bool vq_sat_neg = false;


static float id_integ = 0.0f;
static float iq_integ = 0.0f;

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
    *ia = (csa_v_get(0)-csa_cal[0])/(csa_G*csa_R);
    *ib = (csa_v_get(2)-csa_cal[2])/(csa_G*csa_R);
    *ic = (csa_v_get(1)-csa_cal[1])/(csa_G*csa_R);
}

static void retrieve_measurements(void)
{
    // retrieve ADC measurements
    vbatt_m = vsense_v_get()*vsense_div;
    get_phase_currents(&ia_m, &ib_m, &ic_m);

    // retrieve encoder measurement
    phys_theta_m = wrap_2pi(encoder_read_rad());
    elec_theta_m = wrap_2pi(phys_theta_m*n_poles+theta_offset);

    if (prev_phys_theta_m_set) {
        const float tc = 0.02f;
        const float alpha = dt/(dt+tc);
        phys_omega_m += (wrap_pi(phys_theta_m-prev_phys_theta_m)/dt - phys_omega_m) * alpha;
        prev_phys_theta_m = phys_theta_m;
    } else {
        prev_phys_theta_m = phys_theta_m;
        prev_phys_theta_m_set = true;
    }

    dqo_transform(elec_theta_m, ia_m,ib_m,ic_m, &id_m,&iq_m,&io_m);
}

static void run_commutation(void)
{
    float iq_ref = 0.0f;
    float id_err = (id_ref-id_m);
    float iq_err = (iq_ref-iq_m);

    if ((!vd_sat_pos || id_err < 0) && (!vd_sat_neg || id_err > 0)) {
        id_integ += id_err*dt*curr_KI;
    }

    if ((!vq_sat_pos || iq_err < 0) && (!vq_sat_neg || iq_err > 0)) {
        iq_integ += iq_err*dt*curr_KI;
    }

    vd = (id_ref*curr_KR + id_err*curr_KP + id_integ)/vbatt_m;
    vq = (iq_err*15.0f + iq_integ)/vbatt_m;

    if (vq > max_duty) {
        vq = max_duty;
        vq_sat_pos = true;
        vq_sat_neg = false;
    } else if (vq < -max_duty) {
        vq = -max_duty;
        vq_sat_pos = false;
        vq_sat_neg = true;
    } else {
        vq_sat_pos = false;
        vq_sat_neg = false;
    }

    float vd_lim = sqrtf(max_duty*max_duty-vq*vq);
    if (vd > vd_lim) {
        vd = vd_lim;
        vd_sat_pos = true;
        vd_sat_neg = false;
    } else if (vq < -vd_lim) {
        vd = -vd_lim;
        vd_sat_pos = false;
        vd_sat_neg = true;
    } else {
        vd_sat_pos = false;
        vd_sat_neg = false;
    }

    float alpha = vd*cosf(elec_theta_m) - vq*sinf(elec_theta_m);
    float beta = vq*cosf(elec_theta_m) + vd*sinf(elec_theta_m);

    alpha = 0.5*cosf(2.0f*M_PI_F*millis()*1e-3f);
    beta = -0.5*sinf(2.0f*M_PI_F*millis()*1e-3f);

    float a,b,c;
    svgen(alpha, beta, &a, &b, &c);
    a -= (1.0f-max_duty)*0.5f;
    b -= (1.0f-max_duty)*0.5f;
    c -= (1.0f-max_duty)*0.5f;

    set_phase_duty(a, b, c);
}

int main(void)
{
    clock_init();
    timing_init();
    serial_init();
    spi_init();
    drv_init();
    pwm_init();
    adc_init();
    usleep(1000000);
    calibrate_phase_currents();

    uint32_t last_t_us = 0;
    uint32_t last_print_t = 0;
    uint8_t prev_smpidx = 0;

    // main loop
    while(1) {
        // wait specified time for adc measurement
        uint8_t smpidx, d_smp;
        uint32_t t1_us = micros();
        do {
            smpidx = get_adc_smpidx();
            d_smp = smpidx-prev_smpidx;
        } while (d_smp < 3);
        prev_smpidx = smpidx;
        dt = d_smp*1.0f/18000.0f;
        uint32_t tnow_us = micros();
        float dt_real = (tnow_us-last_t_us)*1.0e-6f;
        last_t_us = tnow_us;
        uint8_t usagepct = (1.0f-(micros()-t1_us)*1.0e-6f / dt)*100.0f;

        retrieve_measurements();

        const float ang_P = 2.0f;
        const float ang_D = 0.15f;
        //float pos_dem = sinf(2.0f*M_PI_F*millis()*1.0e-3f);
        float pos_dem = (millis()/500)%2 ? 0.0f : M_PI_F/2.0f;
        id_ref = wrap_pi(pos_dem-phys_theta_m)*2.0f-phys_omega_m*0.15f;

        run_commutation();

        if (millis()-last_print_t > 10) {
            last_print_t = millis();

            char buf[256];
            int n;
            n = sprintf(buf, "% f,% f,% f,% f\n", phys_theta_m*180.0f/M_PI_F, elec_theta_m*180.0f/M_PI_F, wrap_2pi(2.0f*M_PI_F*millis()*1e-3f)*180.0f/M_PI_F, wrap_pi(elec_theta_m-wrap_2pi(2.0f*M_PI_F*millis()*1e-3f))*180.0f/M_PI_F);
            serial_send_dma(n, buf);
        }
    }

    return 0;
}
