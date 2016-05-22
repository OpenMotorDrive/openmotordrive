#include "helpers.h"
#include <math.h>
#include <stdint.h>

void svgen(float alpha, float beta, float* a, float* b, float* c)
{
    float Va = beta;
    float Vb = -(beta/2.0f)+(alpha*sqrtf(3.0f)/2.0f);
    float Vc = -(beta/2.0f)-(alpha*sqrtf(3.0f)/2.0f);
    float X = beta;
    float Y = (beta/2.0f)+(alpha*sqrtf(3.0f)/2.0f);
    float Z = (beta/2.0f)-(alpha*sqrtf(3.0f)/2.0f);
    uint8_t sector = 0;
    if (Va > 0) sector |= 1<<0;
    if (Vb > 0) sector |= 1<<1;
    if (Vc > 0) sector |= 1<<2;
    switch(sector)
    {
        case 0:
            (*a) = 0.5f;
            (*b) = 0.5f;
            (*c) = 0.5f;
            break;
        case 1:
            (*b) = (1.0f-Z-Y)/2.0f;
            (*a) = (*b)+Z;
            (*c) = (*a)+Y;
            break;
        case 2:
            (*a) = (1.0f-Y+X)/2.0f;
            (*c) = (*a)+Y;
            (*b) = (*c)-X;
            break;
        case 3:
            (*a) = (1.0f+Z-X)/2.0f;
            (*b) = (*a)-Z;
            (*c) = (*b)+X;
            break;
        case 4:
            (*c) = (1.0f+X-Z)/2.0f;
            (*b) = (*c)-X;
            (*a) = (*b)+Z;
            break;
        case 5:
            (*b) = (1.0f-X+Y)/2.0f;
            (*c) = (*b)+X;
            (*a) = (*c)-Y;
            break;
        case 6:
            (*c) = (1.0f+Y+Z)/2.0f;
            (*a) = (*c)-Y;
            (*b) = (*a)-Z;
            break;
    }
    (*a) = constrain_float((*a),0.0f,1.0f);
    (*b) = constrain_float((*b),0.0f,1.0f);
    (*c) = constrain_float((*c),0.0f,1.0f);
}

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
