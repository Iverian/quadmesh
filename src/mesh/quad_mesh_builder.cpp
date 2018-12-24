#include "quad_mesh_builder.h"
#include "quad_surface_mesh_builder.h"

#include <geom_model/geom_util.h>
#include <util/debug.h>
#include <util/logging.h>

#include <stdexcept>

using namespace std;

QuadMeshBuilder::QuadMeshBuilder(const Shell& shell,
                                 const QuadmeshConfig& conf)
    : shell_(shell)
    , conf_(conf)
    , mesh_(make_shared<QuadMesh>())
    , log_(init_logger("mesh_builder"))
    , edges_()
    , vertices_()
{
}

QuadMeshBuilder::~QuadMeshBuilder()
{
    log_->flush();
    spdlog::shutdown();
}

QuadMeshBuilder& QuadMeshBuilder::get()
{
    try {
        for (auto& f : shell_.faces()) {
            build_face_mesh(f);
        }
    } catch (const std::exception& ex) {
        log_->critical(ex.what());
        throw ex;
    }
    return *this;
}

const QuadmeshConfig& QuadMeshBuilder::conf() const
{
    return conf_;
}

std::shared_ptr<QuadMesh> QuadMeshBuilder::mesh() const
{
    return mesh_;
}

void QuadMeshBuilder::build_face_mesh(const Face& f)
{
    auto n = f.boundaries().size();
    vector<vector<QuadMesh::NodePtr>> bounds(n);
    auto surf_builder = QuadSurfaceMeshBuilder(*this, log_)
                            .set_surface(f.surface().shared_from_this());

    for (size_t i = 0; i < n; ++i) {
        auto& loop = f.boundaries()[i];
        auto& bound = bounds[i];
        for (auto& oedge : loop) {
            auto& d = discretize(oedge.edge());
            if (oedge.orienation()) {
                bound.insert(end(bound), begin(d), prev(end(d)));
            } else {
                bound.insert(end(bound), rbegin(d), prev(rend(d)));
            }
        }
        for (auto j = next(begin(bound)); j != end(bound); ++j) {
            set_adjacent(*prev(j), *j);
        }
        set_adjacent(bound.front(), bound.back());
    }

    surf_builder.set_bounds(move(bounds));
    surf_builder.get();
}

const vector<QuadMesh::NodePtr>& QuadMeshBuilder::discretize(const Edge& e)
{
    if (auto i = edges_.find(e); i == end(edges_)) {
        vector<QuadMesh::NodePtr> result;
        auto d = discretize_points(e);
        auto n = d.size();

        result.resize(n);
        result[0] = get_vertex(d[0]);
        for (size_t k = 1; k < n - 1; ++k) {
            result[k] = mesh_->append(d[k]);
        }
        result[n - 1] = get_vertex(d[n - 1]);

        auto j = edges_.emplace(e, move(result));
        return j.first->second;
    } else {
        return i->second;
    }
}

QuadMesh::NodePtr QuadMeshBuilder::get_vertex(const Point& p)
{
    static constexpr auto eps = 1e-5;
    QuadMesh::NodePtr result = nullptr;

    for (auto& i : vertices_) {
        if (isnear(p, i.first, eps)) {
            log_->info("matched existing vertex {} with point {}", i.first, p);
            result = i.second;
            break;
        }
    }
    if (result == nullptr) {
        log_->info("appended geometry vertex: {}", p);
        result = mesh_->append(p);
        vertices_.emplace_back(p, result);
        result = vertices_.back().second;
    }
    return result;
}

vector<Point> QuadMeshBuilder::discretize_points(const Edge& e)
{
    return e.curve().discretize(e.pfront(), e.pback());
}
