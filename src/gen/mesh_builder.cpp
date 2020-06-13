#include "mesh_builder.hpp"
#include "surface_mesh_builder.hpp"

#include <mesh/curve_discretize.hpp>
#include <qmsh/mesh.hpp>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <util/cyclic_iterator.hpp>
#include <util/debug.hpp>

#include <algorithm>
#include <ctime>
#include <iterator>
#include <random>
#include <vector>

namespace qmsh {

MeshBuilder::MeshBuilder(const gm::Shell& shell, const qmsh::Config& conf)
    : shell_(shell)
    , conf_(conf)
    , cdisc_(conf_)
    , vtxs_()
    , edges_()
    , is_complete_(false)
    , log_(cmms::setup_logger(logger_id))
    , mesh_()
{
}

Mesh MeshBuilder::get()
{
    check_if(!is_complete_, "getting value from same mesh builder twice");
    auto rd = std::random_device {};
    auto rng = std::default_random_engine {};
    rng.seed(rd());

    auto f = shell_.faces();
    // std::shuffle(std::begin(f), std::end(f), rng);

    for (auto& face : f) {
        mesh_face(face);
    }

    is_complete_ = true;
    return std::move(mesh_);
}

void MeshBuilder::mesh_face(const gm::Face& obj)
{
    auto m = obj.boundaries().size();
    std::vector<std::vector<VtxPtr>> bounds(m);

    for (size_t i = 0; i < m; ++i) {
        auto& loop = obj.boundaries()[i];
        auto& bound = bounds[i];

        for (auto& j : loop) {
            auto& d = mesh_edge(j.edge());
            if (j.orientation()) {
                bound.insert(std::end(bound), std::begin(d),
                             std::prev(std::end(d)));
            } else {
                bound.insert(std::end(bound), std::rbegin(d),
                             std::prev(std::rend(d)));
            }
        }

        auto j = CyclicIterator(std::begin(bound), std::end(bound));
        do {
            mesh_.insert_edge({{*j, *std::next(j)}});
        } while (++j, j.iter() != j.first());
    }

    log_->debug("meshing surface {} / {}", size_t(&obj.surface()),
                obj.surface());
    SurfaceMeshBuilder(log_.get(), obj.surface(), obj.same_sense(), mesh_,
                       conf_, std::move(bounds))
        .get();
    log_->debug("successfully meshed surface {}", size_t(&obj.surface()));
}

VtxPtr MeshBuilder::mesh_vertex(gm::Point value)
{
    for (auto& i : vtxs_) {
        if (gm::cmp::isnear(value, i.first, gm::Tolerance::ZERO)) {
            log_->debug("point matched existing vertex / [{}, {}]", value,
                        i.first);
            return i.second;
        }
    }
    vtxs_.emplace_back(value, mesh_.insert_external_vertex(value, true));
    log_->debug("inserted new vertex / {}", value);
    return vtxs_.back().second;
}

const std::vector<VtxPtr>& MeshBuilder::mesh_edge(const gm::Edge& edge)
{
    if (edges_.find(edge) == std::end(edges_)) {
        std::vector<VtxPtr> r;
        auto v = cdisc_(edge);
        r.reserve(v.size());

        r.emplace_back(mesh_vertex(v.front()));
        for (auto j = std::next(std::begin(v)); j != std::prev(std::end(v));
             ++j) {
            r.emplace_back(mesh_.insert_external_vertex(*j, false));
        }
        r.emplace_back(mesh_vertex(v.back()));
        edges_.try_emplace(edge, std::move(r));
        log_->debug("inserted new edge / {}", edge);
    } else {
        log_->debug("found existing edge / {}", edge);
    }

    return edges_[edge];
}

} // namespace qmsh
