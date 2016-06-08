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
bool ringbuf_peek(volatile struct ringbuf_t* buf, char* value);
bool ringbuf_pop(volatile struct ringbuf_t* buf, char* value);

#endif // RINGBUF_H
