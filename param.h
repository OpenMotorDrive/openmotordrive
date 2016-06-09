#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <stdbool.h>

enum param_key_t {
    PARAM_ESC_MOT_KV=0,
    PARAM_ESC_MOT_POLES,
    PARAM_ESC_MOT_R,
    PARAM_ESC_FOC_P,
    PARAM_ESC_FOC_I,
    PARAM_ESC_ENC_PBIAS,
    PARAM_ESC_ENC_EBIAS,
};

void param_init(void);
bool param_set_and_save(enum param_key_t key, float value);
float param_retrieve(enum param_key_t key);
float param_retrieve_by_index(uint16_t idx);
bool param_get_name_value_by_index(uint8_t idx, char* name, float* value);
void param_erase(void);
void param_squash(void);
uint8_t param_get_num_params(void);
bool param_index_in_range(uint8_t idx);

#endif // PARAM_H
