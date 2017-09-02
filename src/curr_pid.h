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

#include <stdbool.h>

struct curr_pid_param_s {
    float i_ref;
    float i_meas;
    float ff;
    float dt;
    float output_limit;
    float K_R;
    float K_P;
    float K_I;
};

struct curr_pid_state_s {
    bool sat_pos;
    bool sat_neg;
    float integrator;
    float output;
};

void curr_pid_run(struct curr_pid_param_s *param, struct curr_pid_state_s *state);
