#pragma once

#define M_PI_F ((float)M_PI)
#define SQ(__X) ((__X)*(__X))

static float wrap_1(float x)
{
    volatile float z = (x + 25165824.0f);
    x = x - (z - 25165824.0f);
    return x;
}

static float wrap_pi(float x)
{
    return wrap_1(x/M_PI_F)*M_PI_F;
}

static float wrap_2pi(float x)
{
    x = wrap_pi(x);

    if (x < 0) {
        x += 2*M_PI_F;
    }

    return x;
}

static float sinf_fast(float x)
{
    const float Q = 3.1f;
    const float P = 3.6f;
    float y;

    x = wrap_1(x/M_PI_F);
    y = x - x * fabsf(x);
    return y * (Q + P * fabsf(y));
}

static float cosf_fast(float x)
{
    return sinf_fast(x+M_PI_F/2.0f);
}
