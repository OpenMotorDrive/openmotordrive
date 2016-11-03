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

#pragma once

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
