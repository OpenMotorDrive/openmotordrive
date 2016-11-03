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

enum param_key_t {
    PARAM_ESC_MOT_KV=0,
    PARAM_ESC_MOT_POLES,
    PARAM_ESC_MOT_R,
    PARAM_ESC_FOC_P,
    PARAM_ESC_FOC_I,
    PARAM_ESC_ENC_PBIAS,
    PARAM_ESC_ENC_EBIAS,
};

void param_init(void);
bool param_set_and_save(enum param_key_t key, float value);
float param_retrieve(enum param_key_t key);
float param_retrieve_by_index(uint16_t idx);
bool param_get_name_value_by_index(uint8_t idx, char* name, float* value);
void param_erase(void);
void param_squash(void);
uint8_t param_get_num_params(void);
bool param_index_in_range(uint8_t idx);
