#pragma once

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
