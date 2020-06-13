#ifndef QUADMESH_SRC_MESH_UTIL_HPP_
#define QUADMESH_SRC_MESH_UTIL_HPP_

#include <gm/line.hpp>
#include <gm/plane.hpp>
#include <qmsh/mesh.hpp>

#include <array>

namespace qmsh {

enum class ProximityCase : int {
    MID_TO_MID = 0,
    END_TO_MID = 1,
    END_TO_END = 2,
};

struct DistResult {
    double dist;
    double param_a;
    double param_b;
    ProximityCase pcase;
};

bool is_convex(const gm::Plane& p, const ElementPtr& ptr);
std::array<double, 2> line_solve(const gm::Line& a, const gm::Line& b);
std::optional<gm::Point> line_intersect(const gm::Line& a, const gm::Line& b);
DistResult line_dist(const gm::Line& a, const gm::Line& b);
DistResult unary_segment_dist(const gm::Line& a, const gm::Line& b);

} // namespace qmsh

#endif // QUADMESH_SRC_MESH_UTIL_HPP_
