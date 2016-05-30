#ifndef DRV_H
#define DRV_H
#include <stdint.h>

void drv_init(void);
uint16_t drv_read_register(uint8_t reg);
void drv_write_register(uint8_t reg, uint16_t val);
void drv_write_register_bits(uint8_t reg, uint8_t rng_begin, uint8_t rng_end, uint16_t val);
void drv_print_register(uint8_t reg);

#endif // DRV_H
