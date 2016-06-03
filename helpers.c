#include "helpers.h"
#include <math.h>
#include <stdint.h>

float constrain_float(float val, float min_val, float max_val)
{
    if (val < min_val) {
        return min_val;
    }
    if (val > max_val) {
        return max_val;
    }
    return val;
}

float wrap_2pi(float val)
{
    val = fmodf(val, 2.0f*M_PI_F);
    if (val < 0) {
        val += 2.0f*M_PI_F;
    }
    return val;
}

float wrap_pi(float val){
    val = fmod(val + M_PI_F,2.0f*M_PI_F);
    if (val < 0) {
        val += 2.0f*M_PI_F;
    }
    return val - M_PI_F;
}
