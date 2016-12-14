/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <esc/ringbuf.h>

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
    return ((b->head-b->tail) % b->max_size) - 1;
}

void ringbuf_clear(volatile struct ringbuf_t* b)
{
    b->tail = b->head;
}
