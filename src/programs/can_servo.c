#include <esc/program.h>

#include <esc/helpers.h>
#include <esc/motor.h>
#include <esc/timing.h>

#include "servo.h"

void program_init(void) {
    // Calibrate the encoder
    motor_set_mode(MOTOR_MODE_ENCODER_CALIBRATION);
}

void program_event_adc_sample(float dt) {
    motor_update_state(dt);

    servo_run(dt, (millis()/1000)%2 == 0 ? 0.0f : M_PI_F/2.0f);

    motor_run_commutation(dt);
}
