#ifndef PWM_H
#define PWM_H

#include <stdint.h>

void pwm_init(void);
void set_phase_duty(float phaseA, float phaseB, float phaseC);

#endif // PWM_H
