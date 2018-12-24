#define _USE_MATH_DEFINES

#include "math.h"

#include <cmath>

using namespace std;

double pad(double t, double f, double b, double pad)
{
    return (b * (t + pad) - f * (t - pad) - 2 * pad * t) / (b - f);
}

array<double, 2> atan2v(double y, double x)
{
    auto u1 = atan2(y, x);
    if (u1 < 0) {
        u1 += M_PI;
    }
    auto u2 = u1 + M_PI;
    return {u1, u2};
}