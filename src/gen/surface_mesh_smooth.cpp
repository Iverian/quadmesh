#include "surface_mesh_smooth.hpp"

#include <gm/point.hpp>
#include <list>
#include <optional>
#include <unordered_map>
#include <util/debug.hpp>

namespace qmsh {

using Vertices = std::unordered_map<SurfaceMesh::Id, size_t>;
using Queue = std::list<Vertices::iterator>;

gm::Vec angular_smooth_delta(gm::Point vtx, double vtx_projected_length,
                             std::array<gm::Point, 3> surround, gm::Vec norm);

void surface_mesh_smooth(SurfaceMesh& mesh, const gm::AbstractSurface& surf,
                         std::vector<SurfaceMesh::Id> vertices, size_t depth)
{
    Vertices vtxs;
    Queue queue;
}

std::optional<gm::Point> smooth_boundary_node(SurfaceMesh& mesh, const gm::AbstractSurface&
                                              SurfaceMesh::VtxPtr vtx)
{
    auto& p0 = vtx->value();
    if (vtx->external() || vtx->adjacent().size() < 3) {
        return std::nullopt;
    }

    gm::Point p1;
    for (auto& i : vtx->adjacent()) {
        p1 += mesh[i]->value();
    }
    p1 /= vtx->adjacent().size();

    gm::Point delta;
    if (vtx->adjacent().size() == 3) {
        auto prev = SurfaceMesh::VtxPtr();
        for (auto& i : vtx->adjacent()) {
            auto j = mesh[i];
            if (j->boundary()) {
                prev = j;
                break;
            }
        }
        check_if(prev, "no boundary neighbours");

        auto triplets = mesh.element_triplets_by_vertex(vtx, prev);

        auto& pl = triplets[0][0]->value();
        auto& pc = triplets[0][2]->value();
        auto& pr = triplets[1][2]->value();

        auto ld = vtx->size();
        auto la = gm::dist(pc, p1);

        auto d0 = p1 - p0;
        auto d1 = pc - p0 + (d0 + p0 - pc) * (ld / la);
        auto d2 = angular_smooth_delta(p0, ld, {pl, pc, pr},
                                       s_.normal(s_.project(pc)));
        delta = (d1 + d2) / 2;
    } else {
        delta = p1 - p0;
    }
}

gm::Point smooth_internal_node(SurfaceMesh& mesh, SurfaceMesh::VtxPtr vtx)
{
}

gm::Vec angular_smooth_delta(gm::Point vtx, double vtx_projected_length,
                             std::array<gm::Point, 3> surround, gm::Vec norm)
{
    auto v0 = gm::Vec(surround[1], surround[0]);
    auto v1 = gm::Vec(surround[1], vtx);
    auto v2 = gm::Vec(surround[1], surround[2]);

    auto c = gm::bisect({v0, v2, v1});
    auto line = gm::Line(c, surround[1]);
    auto [u, _] = line_solve(
        line, gm::Line(gm::Vec(surround[0], surround[2]), surround[0]));
    auto lq = gm::dist(line.f(u), surround[1]);
    c *= (vtx_projected_length > lq) ? ((vtx_projected_length + lq) / 2)
                                     : vtx_projected_length;
    return c - v1;
}

} // namespace qmsh