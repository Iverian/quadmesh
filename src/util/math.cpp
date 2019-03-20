#define _USE_MATH_DEFINES

#include "math.h"

#include <cmath>

using namespace std;

double pad(double t, double f, double b, double pad)
{
    return (b * (t + pad) - f * (t - pad) - 2 * pad * t) / (b - f);
}
