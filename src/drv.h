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

#ifndef DRV_H
#define DRV_H
#include <stdint.h>
#include <stdbool.h>

void drv_init(void);
uint16_t drv_read_register(uint8_t reg);
void drv_write_register(uint8_t reg, uint16_t val);
void drv_write_register_bits(uint8_t reg, uint8_t rng_begin, uint8_t rng_end, uint16_t val);
void drv_print_register(uint8_t reg);
bool drv_get_fault(void);
void drv_csa_cal_mode_on(void);
void drv_csa_cal_mode_off(void);
void drv_6_pwm_mode(void);
void drv_3_pwm_mode(void);

#endif // DRV_H
