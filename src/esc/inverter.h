#pragma once

struct inverter_sense_data_s {
    uint32_t t_us;
    float v_bus;
    float i_a;
    float i_b;
    float i_c;
};
