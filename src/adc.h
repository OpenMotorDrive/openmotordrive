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

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void adc_init(void);
void adc_get_csa_v(float *phaseA, float *phaseB, float *phaseC);
float adc_get_vsense_v(void);
void adc_wait_for_sample(void);
uint8_t adc_get_smpidx(void);
uint32_t adc_get_errcnt(void);
float adc_get_smp_freq(void);
float adc_get_smp_period(void);

#endif // ADC_H
