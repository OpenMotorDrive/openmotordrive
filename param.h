#ifndef PARAM_H
#define PARAM_H

#include <stdbool.h>

enum param_key_t {
    PARAM_ID_TEST=0,
    NUM_PARAMS
};

void param_init(void);
bool param_set_and_save(enum param_key_t key, float value);
float param_retrieve(enum param_key_t key);
void param_erase(void);
void param_squash(void);

#endif // PARAM_H
