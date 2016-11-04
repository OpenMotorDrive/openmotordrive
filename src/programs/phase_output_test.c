#include <programs/program_list.h>

#include <esc/helpers.h>
#include <esc/motor.h>
#include <esc/timing.h>
#include <esc/serial.h>
#include <esc/drv.h>

#include <stdio.h>

DEFINE_PROGRAM(PHASE_OUTPUT_TEST)

void init_handler(void) {
    motor_set_mode(MOTOR_MODE_PHASE_VOLTAGE_TEST);}

void adc_sample_handler(float dt) {
    motor_update_state(dt);

    static uint32_t last_print_ms = 0;
    uint32_t tnow_ms = millis();
    if (tnow_ms-last_print_ms > 500) {
        char buf[200];
        int n = 0;
        uint8_t reg;
        for (reg=1; reg<=4; reg++) {
            uint16_t reg_val = drv_read_register(reg);
            n += sprintf(buf+n, "0x%X 0b", reg);
            int8_t i;
            for (i=10; i>=0; i--) {
                buf[n++] = ((reg_val>>i)&1) ? '1' : '0';
            }
            buf[n++] = '\n';
        }
        buf[n++] = '\n';
        serial_send_dma(n, buf);
        last_print_ms = tnow_ms;
    }

    motor_run_commutation(dt);
}
