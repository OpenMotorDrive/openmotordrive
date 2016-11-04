#include <programs/program_list.h>

#include <esc/helpers.h>
#include <esc/motor.h>
#include <esc/timing.h>
#include <esc/serial.h>
#include <esc/encoder.h>

#include <stdio.h>

DEFINE_PROGRAM(PRINT_ENCODER)

void init_handler(void) {
    // Calibrate the encoder
    motor_set_mode(MOTOR_MODE_ENCODER_CALIBRATION);
}

void adc_sample_handler(float dt) {
    motor_update_state(dt);

    static uint32_t last_print_ms = 0;
    uint32_t tnow_ms = millis();
    if (tnow_ms-last_print_ms > 100) {
        char buf[20];
        int n;
        n = sprintf(buf, "% .2f\n", encoder_get_angle_rad() * 180.0f/M_PI_F);
        serial_send_dma(n, buf);
        last_print_ms = tnow_ms;
    }

    motor_run_commutation(dt);
}
