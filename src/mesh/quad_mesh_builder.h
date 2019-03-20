#ifndef QUADMESH_SRC_MESH_QUAD_MESH_BUILDER_H_
#define QUADMESH_SRC_MESH_QUAD_MESH_BUILDER_H_

#include <gm/edge.hpp>
#include <gm/shell.hpp>
#include <quadmesh/quad_mesh.h>
#include <quadmesh/quadmesh_config.h>

#include <spdlog/logger.h>

class QuadMeshBuilder {
public:
    QuadMeshBuilder(const gm::Shell& shell, const QuadmeshConfig& conf);
    ~QuadMeshBuilder();

    QuadMeshBuilder& get();

    const QuadmeshConfig& conf() const;
    std::shared_ptr<QuadMesh> mesh() const;

    void build_face_mesh(const gm::Face& f);
    const std::vector<QuadMesh::NodePtr>& discretize(const gm::Edge& e);
    QuadMesh::NodePtr get_vertex(const gm::Point& p);
    std::vector<gm::Point> discretize_points(const gm::Edge& e);

private:
    gm::Shell shell_;
    QuadmeshConfig conf_;
    std::shared_ptr<QuadMesh> mesh_;
    std::shared_ptr<spdlog::logger> log_;

    std::unordered_map<gm::Edge, std::vector<QuadMesh::NodePtr>> edges_;
    std::vector<std::pair<gm::Point, QuadMesh::NodePtr>> vertices_;
};

#endif // QUADMESH_SRC_MESH_QUAD_MESH_BUILDER_H_