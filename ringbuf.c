#include "ringbuf.h"

bool ringbuf_push(volatile struct ringbuf_t* b, char value)
{
    uint16_t next_head = (b->head+1) % b->max_size;
    if (next_head != b->tail) {
        b->buf[next_head] = value;
        b->head = next_head;
        return true;
    }
    return false;
}

bool ringbuf_pop(volatile struct ringbuf_t* b, char* value)
{
    if (b->head == b->tail) {
        return false;
    }

    *value = b->buf[b->tail];
    b->tail = (b->tail+1) % b->max_size;
    return true;
}

bool ringbuf_peek(volatile struct ringbuf_t* b, uint16_t idx, char* value)
{
    if (idx >= ringbuf_size(b)) {
        return false;
    }

    *value = b->buf[(b->tail+idx) % b->max_size];
    return true;
}

uint16_t ringbuf_size(volatile struct ringbuf_t* b)
{
    if (b->head >= b->tail) {
        return b->head - b->tail;
    } else {
        return b->tail + b->max_size - b->head;
    }
}

void ringbuf_clear(volatile struct ringbuf_t* b)
{
    b->tail = b->head;
}
