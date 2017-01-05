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

static void servo_run(float dt, float theta) {
    static float iq_ref_filt;
    const float tc = 0.005f;
    const float ang_P = 50.0f;
    const float ang_D = 0.5f;
    float alpha = dt/(dt+tc);

    iq_ref_filt += ((wrap_pi(theta-motor_get_phys_rotor_angle())*ang_P-motor_get_phys_rotor_ang_vel()*ang_D) - iq_ref_filt) * alpha;
    motor_set_iq_ref(iq_ref_filt);
}
