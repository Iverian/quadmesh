#ifndef QUADMESH_SRC_UTIL_MATH_H_
#define QUADMESH_SRC_UTIL_MATH_H_

#include <gm/surf_point.hpp>

#include <array>
#include <vector>

double pad(double t, double f, double b, double pad = 1e-9);

template <class T>
T sqr(T x)
{
    return T(x * x);
}

#endif // QUADMESH_SRC_UTIL_MATH_H_
