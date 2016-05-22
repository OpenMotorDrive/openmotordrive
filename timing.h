#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>

void timing_init(void);
uint32_t millis(void);
uint32_t micros(void);
void usleep(uint32_t delay);

#endif // TIMING_H
