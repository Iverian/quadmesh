#include "minimizer.hpp"

#include <gm/point.hpp>

namespace qmsh {

/*
    f[0] = f
    f[1] = dfu
    f[2] = dfv
    f[3] = dfuu
    f[4] = dfuv
    f[5] = dfvv
*/

SurfMinimizer::SurfMinimizer(std::array<SurfDistFunc, 6>&& f,
                             std::pair<gm::SurfPoint, gm::SurfPoint> lim)
    : f_(std::move(f))
    , lim_(std::move(lim))
{
}

std::optional<gm::SurfPoint> SurfMinimizer::
operator()(gm::SurfPoint init) const
{
    std::optional<gm::SurfPoint> result = std::nullopt;

    gm::SurfPoint r, s = init;
    for (size_t i = 0; i < max_iter; ++i) {
        auto f = f_[0](r), du = f_[1](r), dv = f_[2](r);
        s = box_check(r + next_step(r, du, dv));
        if (gm::cmp::zero(f)
            || gm::cmp::zero((s.u - r.u) * du + (s.v - r.v) * dv)) {
            result = r;
            break;
        }
    }

    return result;
}

#define sqr(x) ((x) * (x))

gm::SurfPoint SurfMinimizer::next_step(gm::SurfPoint r, double du,
                                       double dv) const noexcept
{
    double a[] = {f_[3](r), f_[4](r), f_[5](r), -du, -dv};
    double d[] = {a[0] * a[2] - sqr(a[1]), a[3] * a[2] - a[4] * a[1],
                  a[4] * a[0] - a[3] * a[1]};

    return {d[1] / d[0], d[2] / d[0]};
}

#undef sqr

gm::SurfPoint SurfMinimizer::box_check(gm::SurfPoint r) const noexcept
{
    if (r.u < lim_.first.u) {
        r.u = lim_.first.u;
    }
    if (r.u > lim_.second.u) {
        r.u = lim_.second.u;
    }
    if (r.v < lim_.first.v) {
        r.v = lim_.first.v;
    }
    if (r.v > lim_.second.v) {
        r.v = lim_.second.v;
    }

    return r;
}
}
