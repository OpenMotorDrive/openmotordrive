#ifndef PWM_H
#define PWM_H

#include <stdint.h>

void pwm_init(void);
void set_pwm_duty(uint8_t phase, float duty);

#endif // PWM_H
