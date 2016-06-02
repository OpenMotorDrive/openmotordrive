#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <stdint.h>

#define M_SQRT2_F ((float)M_SQRT2)
#define M_PI_F ((float)M_PI)

#define SQ(__X) (__X*__X)

void svgen(float alpha, float beta, float* a, float* b, float* c);
float constrain_float(float val, float min_val, float max_val);
uint8_t get_sector(float alpha, float beta);
void dqo_transform(float theta, float a, float b, float c, float* d, float* q, float* o);
void dqo_transform_inverse(float theta, float d, float q, float o, float* a, float* b, float* c);
float wrap_2pi(float val);
float wrap_pi(float val);

#endif // HELPERS_H
