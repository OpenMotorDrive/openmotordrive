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

#include "curr_pid.h"

#include <string.h>

void curr_pid_run(struct curr_pid_param_s *param, struct curr_pid_state_s *state)
{
    float err = param->i_ref - param->i_meas;

    if ((!state->sat_pos || err < 0) && (!state->sat_neg || err > 0)) {
        state->integrator += err * param->dt * param->K_I;
    }

    state->output  = param->i_ref * param->K_R;
    state->output += param->ff;
    state->output += err * param->K_P;
    state->output += state->integrator;

    if (state->output > param->output_limit) {
        state->output = param->output_limit;
        state->sat_pos = true;
        state->sat_neg = false;
    } else if (state->output < -param->output_limit) {
        state->output = -param->output_limit;
        state->sat_pos = false;
        state->sat_neg = true;
    } else {
        state->sat_pos = false;
        state->sat_neg = false;
    }
}
