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

bool ringbuf_peek(volatile struct ringbuf_t* b, char* value)
{
    if (b->head == b->tail) {
        return false;
    }

    *value = b->buf[b->tail];
    return true;
}
