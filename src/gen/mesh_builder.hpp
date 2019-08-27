#ifndef QUADMESH_SRC_GEN_MESH_BUILDER_HPP_
#define QUADMESH_SRC_GEN_MESH_BUILDER_HPP_

#include <gm/edge.hpp>
#include <gm/point.hpp>
#include <gm/shell.hpp>
#include <mesh/curve_discretize.hpp>
#include <qmsh/config.hpp>
#include <qmsh/mesh.hpp>

#include <cmms/logging.hpp>

#include <unordered_map>

namespace qmsh {

class MeshBuilder {
public:
    static constexpr auto logger_id = "quadmesh";

    MeshBuilder(const gm::Shell& shell, const qmsh::Config& conf);

    Mesh get();

private:
    void mesh_face(const gm::Face& obj);

    Mesh::VtxPtr mesh_vertex(gm::Point value);
    const std::vector<Mesh::VtxPtr>& mesh_edge(const gm::Edge* obj);

    const gm::Shell& shell_;
    const qmsh::Config& conf_;

    CurveDiscretize cdisc_;
    std::unordered_map<gm::Point, Mesh::VtxPtr> vtxs_;
    std::unordered_map<const gm::Edge*, std::vector<Mesh::VtxPtr>> edges_;

    bool is_complete_;
    cmms::Logger log_;
    Mesh mesh_;
};

} // namespace qmsh

#endif // QUADMESH_SRC_GEN_MESH_BUILDER_HPP_
