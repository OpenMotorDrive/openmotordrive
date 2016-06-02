#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

uint8_t encoder_read_register(uint8_t regidx);
void encoder_write_register(uint8_t regidx, uint8_t value);
float encoder_read_rad(void);

#endif // ENCODER_H
