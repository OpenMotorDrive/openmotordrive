#include <programs/program_list.h>

#include <esc/helpers.h>
#include <esc/motor.h>
#include <esc/timing.h>

DEFINE_PROGRAM(SPIN_TEST)

void init_handler(void) {
    // Calibrate the encoder
    motor_set_mode(MOTOR_MODE_ENCODER_CALIBRATION);
}

void adc_sample_handler(float dt) {
    motor_update_state(dt);

    motor_set_id_ref(0.5f);

    if (motor_get_mode() == MOTOR_MODE_DISABLED) {
        motor_set_mode(MOTOR_MODE_FOC_CURRENT);
    }

    motor_run_commutation(dt);
}
