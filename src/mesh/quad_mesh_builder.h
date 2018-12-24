#ifndef QUADMESH_SRC_MESH_QUAD_MESH_BUILDER_H_
#define QUADMESH_SRC_MESH_QUAD_MESH_BUILDER_H_

#include <geom_model/edge.h>
#include <geom_model/shell.h>
#include <quadmesh/quad_mesh.h>
#include <quadmesh/quadmesh_config.h>

#include <spdlog/logger.h>

class QuadMeshBuilder {
public:
    QuadMeshBuilder(const Shell& shell, const QuadmeshConfig& conf);
    ~QuadMeshBuilder();

    QuadMeshBuilder& get();

    const QuadmeshConfig& conf() const;
    std::shared_ptr<QuadMesh> mesh() const;

    void build_face_mesh(const Face& f);
    const std::vector<QuadMesh::NodePtr>& discretize(const Edge& e);
    QuadMesh::NodePtr get_vertex(const Point& p);
    std::vector<Point> discretize_points(const Edge& e);

private:
    Shell shell_;
    QuadmeshConfig conf_;
    std::shared_ptr<QuadMesh> mesh_;
    std::shared_ptr<spdlog::logger> log_;

    std::unordered_map<Edge, std::vector<QuadMesh::NodePtr>> edges_;
    std::vector<std::pair<Point, QuadMesh::NodePtr>> vertices_;
};

#endif // QUADMESH_SRC_MESH_QUAD_MESH_BUILDER_H_