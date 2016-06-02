#include "pwm.h"

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "helpers.h"

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
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_oc_output(TIM1, TIM_OC2);
    timer_enable_oc_output(TIM1, TIM_OC3);
    timer_enable_oc_output(TIM1, TIM_OC4);
    timer_enable_break_main_output(TIM1);
    timer_set_period(TIM1, 1000);
    timer_set_oc_value(TIM1, TIM_OC4, 12);
    TIM1_CR2 |= 0b0111 << 20; // MMS2 OCREF4
    timer_enable_counter(TIM1);
}

void set_phase_duty(float phaseA, float phaseB, float phaseC)
{
    phaseA = constrain_float(phaseA, 0.0f,1.0f);
    timer_set_oc_value(TIM1, TIM_OC1, ((TIM1_ARR-0.5f)*phaseA)+0.5f);
    phaseB = constrain_float(phaseB, 0.0f,1.0f);
    timer_set_oc_value(TIM1, TIM_OC1, ((TIM1_ARR-0.5f)*phaseB)+0.5f);
    phaseC = constrain_float(phaseC, 0.0f,1.0f);
    timer_set_oc_value(TIM1, TIM_OC1, ((TIM1_ARR-0.5f)*phaseC)+0.5f);
}
