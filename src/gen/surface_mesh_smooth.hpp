#ifndef QUADMESH_SRC_GEN_SURFACE_MESH_SMOOTH_HPP_
#define QUADMESH_SRC_GEN_SURFACE_MESH_SMOOTHER_HPP_

#include "surface_mesh.hpp"

#include <gm/abstract_surface.hpp>
#include <gm/point.hpp>

#include <list>
#include <optional>
#include <unordered_map>
#include <vector>

namespace qmsh {

void surface_mesh_smooth(SurfaceMesh& mesh, const gm::AbstractSurface& surf,
                         std::vector<SurfaceMesh::Id> vertices, size_t depth);
std::optional<gm::Point> smooth_boundary_node(SurfaceMesh& mesh,
                                              SurfaceMesh::VtxPtr vtx);
std::optional<gm::Point> smooth_internal_node(SurfaceMesh& mesh,
                                              SurfaceMesh::VtxPtr vtx);

} // namespace qmsh

#endif // QUADMESH_SRC_GEN_SURFACE_MESH_SMOOTH_HPP_