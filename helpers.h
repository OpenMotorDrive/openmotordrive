#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <stdint.h>

#define M_SQRT2_F ((float)M_SQRT2)
#define M_PI_F ((float)M_PI)

#define SQ(__X) (__X*__X)

float constrain_float(float val, float min_val, float max_val);
float wrap_2pi(float val);
float wrap_pi(float val);
uint16_t crc16_ccitt(const char *buf, uint32_t len, uint16_t crc);

#endif // HELPERS_H
