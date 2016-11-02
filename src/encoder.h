#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

uint8_t encoder_read_register(uint8_t regidx);
void encoder_write_register(uint8_t regidx, uint8_t value);
void encoder_read_angle(void);
float encoder_get_angle_rad(void);

#endif // ENCODER_H
