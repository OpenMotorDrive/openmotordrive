#include <esc/program.h>

#include <esc/helpers.h>
#include <esc/motor.h>
#include <esc/timing.h>

void program_init(void) {}

void program_event_adc_sample(float dt) {
    motor_update_state(dt);

    // TODO

    motor_run_commutation(dt);
}
