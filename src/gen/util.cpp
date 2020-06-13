#include "util.hpp"

#include <cmms/cyclic_iterator.hpp>
#include <gm/misc.hpp>
#include <gm/point.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace qmsh {

std::array<double, 2> solve_2d(std::array<double, 4>&& a,
                               std::array<double, 2>&& b);

bool is_convex(const gm::Plane& p, const ElementPtr& ptr)
{
    std::vector<gm::SurfPoint> q(ptr.size());
    std::transform(std::begin(ptr), std::end(ptr), std::begin(q),
                   [&p](auto& i) { return p.project(i->value()); });
    auto c = gm::graham_scan(q);

    return c.size() == elem_vtx;
}

std::optional<gm::Point> line_intersect(const gm::Line& a, const gm::Line& b)
{
    auto [u, v] = line_solve(a, b);
    auto pu = a.f(u);
    auto pv = b.f(v);
    return gm::cmp::near(pu, pv) ? std::make_optional(pu) : std::nullopt;
}

DistResult line_dist(const gm::Line& a, const gm::Line& b)
{
    auto [u, v] = line_solve(a, b);
    return {gm::dist(a.f(u), b.f(v)), u, v, ProximityCase::MID_TO_MID};
}

DistResult unary_segment_dist(const gm::Line& a, const gm::Line& b)
{
    auto r = line_dist(a, b);

    auto fa = gm::cmp::le(0, r.param_a) && gm::cmp::le(r.param_a, 1);
    auto fb = gm::cmp::le(0, r.param_b) && gm::cmp::le(r.param_b, 1);
    if (!fa && fb) {
        r.param_a = (gm::dist(a.f(0), b.f(r.param_b))
                     < gm::dist(a.f(1), b.f(r.param_b)))
            ? 0
            : 1;
        r.pcase = ProximityCase::END_TO_MID;
    } else if (fa && !fb) {
        r.param_b = (gm::dist(a.f(r.param_a), b.f(0))
                     < gm::dist(a.f(r.param_a), b.f(1)))
            ? 0
            : 1;
        r.pcase = ProximityCase::END_TO_MID;
    } else if (!fa && !fb) {
        auto d = std::numeric_limits<double>::max();
        for (auto i : {0, 1}) {
            for (auto j : {0, 1}) {
                if (auto cd = gm::dist(a.f(i), b.f(j)); cd < d) {
                    d = cd;
                    r.param_a = i;
                    r.param_b = j;
                }
            }
        }
        r.pcase = ProximityCase::END_TO_END;
    }

    r.dist = gm::dist(a.f(r.param_a), b.f(r.param_b));
    return r;
}

std::array<double, 2> line_solve(const gm::Line& a, const gm::Line& b)
{
    auto p = a.dir();
    auto q = b.dir();
    auto pq = gm::dot(p, q);
    auto c = a.c() - b.c();
    return solve_2d({gm::sqr(p), -pq, -pq, gm::sqr(q)},
                    {-gm::dot(p, c), -gm::dot(q, c)});
}

inline std::array<double, 2> solve_2d(std::array<double, 4>&& a,
                                      std::array<double, 2>&& b)
{
    double d[] = {a[0] * a[3] - a[1] * a[2], b[0] * a[3] - a[1] * b[1],
                  a[0] * b[1] - b[0] * a[2]};
    return {d[1] / d[0], d[2] / d[0]};
}

} // namespace qmsh
