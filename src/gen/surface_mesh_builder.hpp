#ifndef QUADMESH_SRC_GEN_SURFACE_MESH_BUILDER_HPP_
#define QUADMESH_SRC_GEN_SURFACE_MESH_BUILDER_HPP_

#include "generation_front.hpp"
#include "local_adjacent.hpp"

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

    Mesh::VtxPtr operator()(gm::Point vertex);
    void clear() noexcept;

private:
    std::vector<Mesh::Vtx> buf_;
    size_t pos_;
};

class SurfaceMeshBuilder {
public:
    SurfaceMeshBuilder(cmms::LoggerRef log, const gm::AbstractSurface& s,
                       bool same_sence, Mesh& mesh, const Config& conf,
                       std::vector<std::vector<Mesh::VtxPtr>> boundaries);

    void get();

private:
    struct AddElementResult {
        bool face_inserted;
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

    void eval_iangle(GenerationFront& front);
    void eval_and_resolve(GenerationFront& front);

    Mesh::VtxPtr tmp(gm::Point vertex);
    std::array<Mesh::VtxPtr, 1> project_bisect(FrontType t, const Triple& tr);
    std::array<Mesh::VtxPtr, 3> project_trisect(FrontType t, const Triple& tr);
    std::array<Mesh::VtxPtr, 5> project_pentasect(FrontType t,
                                                  const Triple& tr);

    FrontIter build_row(FrontIter first, GenerationFront& front);
    AddElementResult add_element(GenerationFront& front, FrontIter cur,
                                 FrontIter prev2, Mesh::ElemPtr& tmp,
                                 const gm::Plane& tan);

    void append_to_mesh(Mesh::ElemPtr& tmp);

    bool closure_check(GenerationFront& front);
    void six_vertices_closure(GenerationFront& front);

    std::optional<std::pair<gm::Point, gm::Point>>
    edge_intersection(const Mesh::EdgePtr& a, const Mesh::EdgePtr& b);

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
