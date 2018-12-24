#ifndef QUADMESH_SRC_UTIL_MATH_H_
#define QUADMESH_SRC_UTIL_MATH_H_

#include <geom_model/parametric_point.h>

#include <array>
#include <vector>

double pad(double t, double f, double b, double pad = 1e-9);
std::array<double, 2> atan2v(double y, double x);

template <class T>
T sqr(T x)
{
    return T(x * x);
}

template <class T, class Function>
T diff(Function f, double t)
{
    double h = 1e-4;
    return (-3 * f(t) + 4 * f(t + h) - f(t + 2 * h)) / (2 * h);
}

template <class T, class Function>
T diff2(Function f, double t)
{
    double h = 1e-2;
    return (-f(t + 2 * h) + f(t + h) * 16 - f(t) * 30 + f(t - h) * 16
            - f(t - 2 * h))
        / (12 * sqr(h));
}

template <class T, class Function>
T diff11(Function f, ParametricPoint t)
{
    double h = 1e-4;
    return (-3
                * (-3 * f({t.u, t.v}) + 4 * f({t.u, t.v + h})
                   - f({t.u, t.v + 2 * h}))
            + 4
                * (-3 * f({t.u + h, t.v}) + 4 * f({t.u + h, t.v + h})
                   - f({t.u + h, t.v + 2 * h}))
            - (-3 * f({t.u + 2 * h, t.v}) + 4 * f({t.u + 2 * h, t.v + h})
               - f({t.u + 2 * h, t.v + 2 * h})))
        / (4 * sqr(h));
}

template <class T, class U>
T trapz(const std::vector<T>& vals, U step)
{
    T result = T();
    for (auto it = std::cbegin(vals); it != std::prev(std::cend(vals)); ++it)
        result += step * (*it + *std::next(it)) / 2;
    return result;
}

#endif // QUADMESH_SRC_UTIL_MATH_H_
