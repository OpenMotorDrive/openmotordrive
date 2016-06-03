#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

enum motor_mode_t {
    MOTOR_MODE_DISABLED = 0,
    MOTOR_MODE_FOC_CURRENT,
    MOTOR_MODE_ENCODER_CALIBRATION,
    MOTOR_MODE_TEST,
};

void motor_init(void);
void motor_update_state(float dt);
void motor_run_commutation(float dt);
void motor_set_mode(enum motor_mode_t mode);
void motor_set_id_ref(float id_ref);
enum motor_mode_t motor_get_mode(void);
float motor_get_phys_rotor_angle(void);
float motor_get_phys_rotor_ang_vel(void);
float motor_get_elec_rotor_angle(void);

#endif // MOTOR_H
