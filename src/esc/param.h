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


#define PARAM_MAX_NAME_LEN 92

struct param_info_s {
    const char* name;
    float default_val;
    float min_val;
    float max_val;
    bool int_val;
};

void param_init(void);
void param_write(void);

float* param_retrieve_by_index(uint16_t idx);
float* param_retrieve_by_name(const char* name);
int16_t param_get_index_by_name(const char* name);
bool param_get_info_by_index(uint8_t idx, const struct param_info_s** info);
void param_erase(void);
void param_squash(void);
uint8_t param_get_num_params(void);
bool param_index_in_range(uint8_t idx);
