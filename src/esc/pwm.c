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

#include <esc/pwm.h>
#include <esc/helpers.h>

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <esc/serial.h>

#define TIM_CCMR3(tim_base)          MMIO32((tim_base) + 0x54)
#define TIM1_CCMR3                   TIM_CCMR3(TIM1)
#define TIM_CCR5(tim_base)           MMIO32((tim_base) + 0x58)
#define TIM1_CCR5                    TIM_CCR5(TIM1)

static pwm_phase_duty_callback_type phase_duty_callback;

void pwm_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_TIM1);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8|GPIO9|GPIO10);
    gpio_set_af(GPIOA, GPIO_AF6, GPIO8|GPIO9|GPIO10);

    timer_reset(TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_3, TIM_CR1_DIR_UP);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM2);
    timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM2);
    timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM2);
    TIM1_CCMR3 |= (0b111 << 4); // OC5M = PWM2
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_oc_output(TIM1, TIM_OC2);
    timer_enable_oc_output(TIM1, TIM_OC3);
    timer_enable_oc_output(TIM1, TIM_OC4);
    TIM1_CCER |= (1<<16); // CC5E
    timer_enable_break_main_output(TIM1);
    timer_set_period(TIM1, 2000);
    TIM1_CCR4 = TIM1_ARR;
    TIM1_CCR5 = 24;
//     TIM1_CR2 |= 0b0111 << 20; // MMS2 OCREF4
    TIM1_CR2 |= 0b1000 << 20; // MMS2 OCREF5
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);
    TIM1_DIER |= (1<<4); // CC4IE
    timer_enable_counter(TIM1);
}

void tim1_cc_isr(void) {
    if (TIM1_SR & (1<<4)) {
        TIM1_SR &= ~(1<<4); // CC4IF
        pwm_update();
    }
}

void pwm_update(void) {
    if (phase_duty_callback) {
        float phaseA, phaseB, phaseC;
        phase_duty_callback(&phaseA, &phaseB, &phaseC);

        TIM1_CCR1 = TIM1_ARR-roundf(TIM1_ARR*phaseA);
        TIM1_CCR2 = TIM1_ARR-roundf(TIM1_ARR*phaseB);
        TIM1_CCR3 = TIM1_ARR-roundf(TIM1_ARR*phaseC);
    }
}

void pwm_set_phase_duty_callback(pwm_phase_duty_callback_type cb) {
    phase_duty_callback = cb;
}

void pwm_get_phase_duty(float* phaseA, float* phaseB, float* phaseC)
{
    *phaseA = ((float)(TIM1_ARR-TIM1_CCR1))/TIM1_ARR;
    *phaseB = ((float)(TIM1_ARR-TIM1_CCR2))/TIM1_ARR;
    *phaseC = ((float)(TIM1_ARR-TIM1_CCR3))/TIM1_ARR;
}

float pwm_get_freq(void) {
    return rcc_apb1_frequency/TIM1_ARR;
}

float pwm_get_period(void) {
    return 1/pwm_get_freq();
}
