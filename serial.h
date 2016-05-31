#ifndef USART_H
#define USART_H

#include <stdbool.h>
#include <stdint.h>

void serial_init(void);
uint16_t serial_available(void);
bool serial_send_dma(uint16_t len, char* buf);

#endif // USART_H
