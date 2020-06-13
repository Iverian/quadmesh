#ifndef QUADMESH_SRC_GEN_SURFACE_MESH_BUILDER_HPP_
#define QUADMESH_SRC_GEN_SURFACE_MESH_BUILDER_HPP_

#include "generation_front.hpp"
#include "local_adjacent.hpp"
#include "util.hpp"

#include <cmms/logging.hpp>
#include <gm/abstract_surface.hpp>
#include <gm/point.hpp>
#include <gm/surf_point.hpp>
#include <qmsh/config.hpp>
#include <qmsh/mesh.hpp>
#include <qmsh/serialize.hpp>

#include <array>
#include <map>
#include <vector>

namespace qmsh {

struct Triple {
    std::array<gm::Point, 3> p;
    gm::Vec norm;

    double iangle(FrontType t) const;
    gm::Point& operator[](size_t i) noexcept;
    const gm::Point& operator[](size_t i) const noexcept;

    rapidjson::Value& serialize(rapidjson::Value& result,
                                rapidjson::Value::AllocatorType& alloc) const;
};

class TempVertexBuffer {
public:
    static constexpr auto init_size = 20;

    TempVertexBuffer();

    VtxPtr operator()(gm::Point vertex, double projected_length);
    void clear() noexcept;

private:
    Mesh::Vertices buf_;
};

class SurfaceMeshBuilder {
public:
    SurfaceMeshBuilder(cmms::LoggerRef log, const gm::AbstractSurface& s,
                       bool same_sence, Mesh& mesh, const Config& conf,
                       std::vector<std::vector<VtxPtr>> boundaries);

    void get();

private:
    static constexpr int smooth_depth = 3;

    struct AddElementResult {
        bool face_inserted;
        bool seam_added;
        bool iterators_valid;
        operator bool() const noexcept
        {
            return face_inserted;
        }
    };

    std::pair<Triple, gm::Plane> triple(const FrontCycle& it) const;
    std::pair<Triple, gm::Plane> triple(const FrontCycler& c,
                                        const FrontIter& i) const;
    std::pair<Triple, gm::Plane> triple(FrontIter i, FrontIter j,
                                        FrontIter k) const;
    std::pair<Triple, gm::Plane> triple(gm::Point i, gm::Point j,
                                        gm::Point k) const;

    void eval_iangle(GenerationFront& front);
    void eval_and_resolve(GenerationFront& front);

    bool place_seams(GenerationFront& front);

    VtxPtr tmp(gm::Point vertex, double projected_length);
    std::array<VtxPtr, 1> project_bisect(FrontType t, const Triple& tr);
    std::array<VtxPtr, 3> project_trisect(FrontType t, const Triple& tr);
    std::array<VtxPtr, 5> project_pentasect(FrontType t, const Triple& tr);

    FrontIter build_row(FrontIter first, GenerationFront& front);
    AddElementResult add_element(GenerationFront& front, FrontIter cur,
                                 FrontIter prev2, ElementPtr& tmp,
                                 const gm::Plane& tan);

    void smooth_boundary_nodes(FrontIter first, FrontIter last,
                               GenerationFront& front,
                               int depth = smooth_depth) const;
    void smooth_boundary_node(const FrontCycler& cycle, FrontIter it) const;
    void smooth_internal_node(VtxPtr vtx, const GenerationFront& front,
                              GenerationFront::VtxCache& smoothed_nodes,
                              int depth = smooth_depth) const;

    void append_to_mesh(ElementPtr& tmp);

    bool closure_check(GenerationFront& front);
    void six_vertices_closure(GenerationFront& front);

    std::optional<DistResult> edge_intersection(const EdgePtr& a,
                                                const EdgePtr& b);

    [[nodiscard]] std::pair<GenerationFront, GenerationFront>
    split_front(GenerationFront& front, FrontIter first, FrontIter last,
                FrontIter last2);
    void merge_fronts(GenerationFront& a, FrontIter apos, GenerationFront& b,
                      FrontIter bpos);

    const gm::AbstractSurface& s_;
    Mesh& mesh_;
    const Config& conf_;

    TempVertexBuffer buf_;
    LocalAdjacent adj_;
    std::list<GenerationFront> fronts_;
    cmms::LoggerRef log_;
};

} // namespace qmsh

#endif // QUADMESH_SRC_GEN_SURFACE_MESH_BUILDER_HPP_
