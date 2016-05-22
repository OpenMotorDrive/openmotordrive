#include "pwm.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

void pwm_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_TIM1);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8|GPIO9|GPIO10);
    gpio_set_af(GPIOA, GPIO_AF6, GPIO8|GPIO9|GPIO10);

    timer_reset(TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM2);
    timer_enable_oc_output(TIM1, TIM_OC2);
    timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM2);
    timer_enable_oc_output(TIM1, TIM_OC3);
    timer_enable_break_main_output(TIM1);
    timer_set_period(TIM1, 1024);
    TIM1_CR2 |= 0b0011 << 20; // MMS2 "compare pulse"
    timer_enable_counter(TIM1);
}

void set_pwm_duty(enum tim_oc_id oc_id, float duty)
{
    timer_set_oc_value(TIM1, oc_id, ((TIM1_ARR-0.5f)*duty)+0.5f);
}
