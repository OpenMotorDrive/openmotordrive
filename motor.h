#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

void motor_init(void);
void motor_set_enable(bool val);
void motor_update_state(float dt);
void motor_run_commutation(float dt);
void motor_set_id_ref(float id_ref);
float motor_get_phys_rotor_angle(void);
float motor_get_phys_rotor_ang_vel(void);
float motor_get_elec_rotor_angle(void);

#endif // MOTOR_H
