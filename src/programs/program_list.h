#pragma once

#include <esc/programs.h>

#define PROG_INFO(_NAME) PROG_INFO_ ## _NAME

#define DECLARE_PROGRAM(_NAME) extern const struct program_info_s PROG_INFO_ ## _NAME;

#define DEFINE_PROGRAM(_NAME) \
static void init_handler(void); \
static void adc_sample_handler(float dt); \
const struct program_info_s PROG_INFO_ ## _NAME = { \
    .id = PROGRAM_ ## _NAME, \
    .name = #_NAME, \
    .init_handler = init_handler, \
    .adc_sample_handler = adc_sample_handler \
};

typedef void(*init_handler_cb)(void);
typedef void(*adc_sample_handler_cb)(float);

struct program_info_s {
    enum program_id_t id;
    char* name;
    init_handler_cb init_handler;
    adc_sample_handler_cb adc_sample_handler;
};

DECLARE_PROGRAM(SERVO_TEST)
DECLARE_PROGRAM(SPIN_TEST)
DECLARE_PROGRAM(PHASE_OUTPUT_TEST)
DECLARE_PROGRAM(PRINT_INPUT_VOLTAGE)
DECLARE_PROGRAM(PRINT_ENCODER)
DECLARE_PROGRAM(CAN_SERVO)
DECLARE_PROGRAM(CAN_ENCODER)

#define GET_PROG_INFO_TABLE() { \
    &PROG_INFO(SERVO_TEST), \
    &PROG_INFO(SPIN_TEST), \
    &PROG_INFO(PHASE_OUTPUT_TEST), \
    &PROG_INFO(PRINT_INPUT_VOLTAGE), \
    &PROG_INFO(PRINT_ENCODER), \
    &PROG_INFO(CAN_SERVO), \
    &PROG_INFO(CAN_ENCODER) \
}
