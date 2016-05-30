#ifndef HELPERS_H
#define HELPERS_H

#include <stdint.h>

void svgen(float alpha, float beta, float* a, float* b, float* c);
float constrain_float(float val, float min_val, float max_val);
uint8_t get_sector(float alpha, float beta);
void dqo_transform(float theta, float a, float b, float c, float* d, float* q, float* o);

#endif // HELPERS_H
