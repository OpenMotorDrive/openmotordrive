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

#ifndef CONFIG_H
#define CONFIG_H

enum program_t {
    PROGRAM_SERVO_TEST,
    PROGRAM_SPIN_TEST,
    PROGRAM_PHASE_OUTPUT_TEST,
    PROGRAM_PRINT_DRV_FAULTS,
    PROGRAM_PRINT_INPUT_VOLTAGE,
    PROGRAM_PRINT_ENCODER,
    PROGRAM_CAN_SERVO,
    PROGRAM_SEND_CAN_ANGLE
};

#ifndef CONFIG_PROGRAM
#define CONFIG_PROGRAM PROGRAM_CAN_SERVO
#endif

#endif // CONFIG_H
