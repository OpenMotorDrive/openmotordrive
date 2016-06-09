#ifndef USART_H
#define USART_H

#include <stdbool.h>
#include <stdint.h>
#include "ringbuf.h"

void serial_init(void);
uint16_t serial_available(void);
bool serial_send_dma(uint16_t len, char* buf);
bool serial_send_dma_preloaded(uint16_t len);
bool serial_ready_to_send(void);
char* serial_get_txbuf(void);
uint16_t serial_get_txbuf_len(void);
volatile struct ringbuf_t* serial_get_rxbuf(void);

#endif // USART_H
