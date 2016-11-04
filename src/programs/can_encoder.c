#include <programs/program_list.h>

#include <esc/helpers.h>
#include <esc/motor.h>
#include <esc/timing.h>
#include <esc/can.h>
#include <esc/encoder.h>
#include <string.h>

DEFINE_PROGRAM(CAN_ENCODER)

void init_handler(void) {}

void adc_sample_handler(float dt) {
    motor_update_state(dt);

    static uint32_t last_send_us = 0;
    uint32_t tnow_us = micros();
    if (tnow_us-last_send_us > 1000) {
        struct canbus_msg m;
        m.id = 500;
        m.ide = false;
        m.rtr = false;
        m.dlc = 4;
        float theta = encoder_get_angle_rad();
        memcpy(m.data, &theta, 4);

        canbus_send_message(&m);
        last_send_us = tnow_us;
    }

    motor_run_commutation(dt);
}
