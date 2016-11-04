#include <esc/programs.h>
#include <programs/program_list.h>

#define NUM_PROGRAMS (sizeof(PROG_INFO_TABLE)/sizeof(*PROG_INFO_TABLE))

static const struct program_info_s* _curr_prog;

const struct program_info_s* PROG_INFO_TABLE[] = GET_PROG_INFO_TABLE();

void program_init(enum program_id_t id) {
    uint8_t i;
    for (i = 0; i<NUM_PROGRAMS; i++) {
        if (PROG_INFO_TABLE[i]->id == id) {

            _curr_prog = PROG_INFO_TABLE[i];

            if (_curr_prog->init_handler) {
                _curr_prog->init_handler();
            }

            return;
        }
    }
}

void program_event_adc_sample(float dt) {
    if (_curr_prog && _curr_prog->adc_sample_handler) {
        _curr_prog->adc_sample_handler(dt);
    }
}
