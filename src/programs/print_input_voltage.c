#include <programs/program_list.h>

#include <esc/helpers.h>
#include <esc/motor.h>
#include <esc/timing.h>

DEFINE_PROGRAM(PRINT_INPUT_VOLTAGE)

void init_handler(void) {}

void adc_sample_handler(float dt) {
    motor_update_state(dt);

    // TODO

    motor_run_commutation(dt);
}
