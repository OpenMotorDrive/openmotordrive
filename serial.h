#ifndef USART_H
#define USART_H

#include <stdbool.h>
#include <stdint.h>

void serial_init(void);
uint16_t serial_available(void);
bool serial_send_dma(uint16_t len, char* buf);
bool serial_send_dma_slip(uint16_t len, char* buf);
bool serial_ready_to_send(void);
bool serial_recv_peek(char* byte);
bool serial_recv_pop(char* byte);

#endif // USART_H
