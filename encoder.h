#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void encoder_write_register(uint8_t regidx, uint8_t value);
float encoder_read_rad(void);

#endif // ENCODER_H
