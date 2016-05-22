#ifndef PWM_H
#define PWM_H

#include <libopencm3/stm32/timer.h>

void pwm_init(void);
void set_pwm_duty(enum tim_oc_id oc_id, float duty);

#endif // PWM_H
