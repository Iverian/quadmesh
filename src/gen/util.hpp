#ifndef QUADMESH_SRC_MESH_UTIL_HPP_
#define QUADMESH_SRC_MESH_UTIL_HPP_

#include <gm/line.hpp>
#include <gm/plane.hpp>
#include <qmsh/mesh.hpp>

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

bool is_convex(const gm::Plane& p, const Mesh::ElemPtr& ptr);
DistResult line_dist(const gm::Line& a, const gm::Line& b);
DistResult unary_segment_dist(const gm::Line& a, const gm::Line& b);

} // namespace qmsh

#endif // QUADMESH_SRC_MESH_UTIL_HPP_
