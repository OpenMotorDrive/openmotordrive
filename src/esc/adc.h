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

typedef void (*ISR_ptr)(void);

struct adc_sample_s {
    uint8_t seq;
    uint32_t t_us;
    float vsense_v;
    float csa_v[3];
};

void adc_init(void);
struct adc_sample_s* adc_get_sample(void);
void adc_wait_for_sample(void);
void adc_set_new_sample_isr(ISR_ptr cb);
uint32_t adc_get_errcnt(void);
float adc_get_smp_freq(void);
float adc_get_smp_period(void);
