#include <programs/program_list.h>

#include <esc/helpers.h>
#include <esc/motor.h>
#include <esc/timing.h>

DEFINE_PROGRAM(CAN_SERVO)

static void servo_run(float dt, float theta) {
    static float id_ref_filt;
    const float tc = 0.002f;
    const float ang_P = 3.0f;
    const float ang_D = 0.01f;
    float alpha = dt/(dt+tc);

    id_ref_filt += ((wrap_pi(theta-motor_get_phys_rotor_angle())*ang_P-motor_get_phys_rotor_ang_vel()*ang_D) - id_ref_filt) * alpha;
    motor_set_id_ref(id_ref_filt);

    if (motor_get_mode() == MOTOR_MODE_DISABLED) {
        motor_set_mode(MOTOR_MODE_FOC_CURRENT);
    }
}

void init_handler(void) {
    // Calibrate the encoder
    motor_set_mode(MOTOR_MODE_ENCODER_CALIBRATION);
}

void adc_sample_handler(float dt) {
    motor_update_state(dt);

    servo_run(dt, (millis()/1000)%2 == 0 ? 0.0f : M_PI_F/2.0f);

    motor_run_commutation(dt);
}
