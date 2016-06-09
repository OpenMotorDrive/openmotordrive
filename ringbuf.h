#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>
#include <stdbool.h>

struct ringbuf_t
{
    volatile char* const buf;
    const uint16_t max_size;
    volatile uint16_t head;
    volatile uint16_t tail;
};

bool ringbuf_push(volatile struct ringbuf_t* buf, char value);
bool ringbuf_peek(volatile struct ringbuf_t* buf, uint16_t idx, char* value);
bool ringbuf_pop(volatile struct ringbuf_t* buf, char* value);
void ringbuf_clear(volatile struct ringbuf_t* buf);
uint16_t ringbuf_size(volatile struct ringbuf_t* b);

#endif // RINGBUF_H
