#ifndef CURR_PID_H
#define CURR_PID_H

#include <stdbool.h>

struct curr_pid_param_s {
    float i_ref;
    float i_meas;
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

#endif // CURR_PID_H
