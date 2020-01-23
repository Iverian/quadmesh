#include "mesh_builder.hpp"
#include "surface_mesh_builder.hpp"

#include <mesh/curve_discretize.hpp>
#include <qmsh/mesh.hpp>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <util/cyclic_iterator.hpp>
#include <util/debug.hpp>

#include <iterator>
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

    for (auto& face : shell_.faces()) {
        mesh_face(face);
    }

    is_complete_ = true;
    return std::move(mesh_);
}

void MeshBuilder::mesh_face(const gm::Face& obj)
{
    auto m = obj.boundaries().size();
    std::vector<std::vector<Mesh::VtxPtr>> bounds(m);

    for (size_t i = 0; i < m; ++i) {
        auto& loop = obj.boundaries()[i];
        auto& bound = bounds[i];

        for (auto& j : loop) {
            auto& d = mesh_edge(&j.edge());
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
            mesh_.add_edge({{*j, *std::next(j)}});
        } while (++j, j.iter() != j.first());
    }

    log_->debug("meshing surface {} / {}", size_t(&obj.surface()),
                obj.surface());
    SurfaceMeshBuilder(log_.get(), obj.surface(), obj.same_sense(), mesh_,
                       conf_, std::move(bounds))
        .get();
    log_->debug("successfully meshed surface {}", size_t(&obj.surface()));
}

Mesh::VtxPtr MeshBuilder::mesh_vertex(gm::Point value)
{
    auto [it, flag] = vtxs_.emplace(value, nullptr);
    if (flag) {
        it->second = mesh_.add_vertex(value, true);
        log_->debug("inserted new vertex / {}", value);
    } else {
        log_->debug("point matched existing vertex / [{}, {}]", value,
                    it->first);
    }
    return it->second;
}

const std::vector<Mesh::VtxPtr>&
MeshBuilder::mesh_edge(const gm::Edge* edge_ptr)
{
    auto [it, flag] = edges_.emplace(edge_ptr, std::vector<Mesh::VtxPtr>());
    if (flag) {
        auto& r = it->second;
        auto v = cdisc_(*edge_ptr);
        r.reserve(v.size());

        r.emplace_back(mesh_vertex(v.front()));
        for (auto j = std::next(std::begin(v)); j != std::prev(std::end(v));
             ++j) {
            r.emplace_back(mesh_.add_vertex(*j, true));
        }
        r.emplace_back(mesh_vertex(v.back()));
        log_->debug("inserted new edge / {}", *edge_ptr);
    } else {
        log_->debug("found existing edge / {}", *edge_ptr);
    }

    return it->second;
}

} // namespace qmsh
