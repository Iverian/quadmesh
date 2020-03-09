#ifndef QUADMESH_INCLUDE_QMSH_MESH_HPP_
#define QUADMESH_INCLUDE_QMSH_MESH_HPP_

#include "config.hpp"
#include "exports.hpp"
#include "serialize.hpp"

#include <gm/point.hpp>
#include <gm/shell.hpp>

#include <rapidjson/document.h>
#include <rapidjson/rapidjson.h>

#include <algorithm>
#include <array>
#include <list>
#include <ostream>
#include <set>
#include <type_traits>

namespace qmsh {

static constexpr ptrdiff_t elem_vtx = 4;
static constexpr ptrdiff_t edge_vtx = 2;

class QMSH_EXPORT Mesh {
public:
    using VtxId = ptrdiff_t;
    static constexpr VtxId npos = -1;

    class Vtx;

    using VertexContainer = std::list<Vtx>;
    using VtxPtr = VertexContainer::pointer;
    using ConstVtxPtr = VertexContainer::const_pointer;

    template <ptrdiff_t N>
    struct VarElement : std::array<VtxId, N> {
        rapidjson::Value&
        serialize(rapidjson::Value& result,
                  rapidjson::Value::AllocatorType& alloc) const
        {
            result.SetArray();
            for (auto& i : *this) {
                result.PushBack(i, alloc);
            }
            return result;
        }

        bool is_inserted() const noexcept
        {
            return std::all_of(std::begin(*this), std::end(*this),
                               [](auto& i) { return i->is_inserted(); });
        }
    };

    template <typename T, ptrdiff_t N>
    struct VarElemT : std::array<T, N> {
        rapidjson::Value&
        serialize(rapidjson::Value& result,
                  rapidjson::Value::AllocatorType& alloc) const
        {
            result.SetArray();
            for (auto& i : *this) {
                rapidjson::Value v;
                result.PushBack(i->serialize(v, alloc), alloc);
            }
            return result;
        }
    };

    using Element = VarElement<elem_vtx>;
    using EdgeElement = VarElement<edge_vtx>;

    using ElementContainer = std::vector<Element>;
    using EdgeElementContainer = std::vector<EdgeElement>;

    using ElemPtr = VarElemT<VtxPtr, elem_vtx>;
    using ElemTripletPtr = VarElemT<VtxPtr, elem_vtx - 1>;
    using EdgePtr = VarElemT<VtxPtr, edge_vtx>;
    using ConstElemPtr = VarElemT<ConstVtxPtr, elem_vtx>;
    using ConstElemTripletPtr = VarElemT<ConstVtxPtr, elem_vtx - 1>;
    using ConstEdgePtr = VarElemT<ConstVtxPtr, edge_vtx>;

    Mesh();

    VtxPtr add_vertex(Vtx vtx);
    VtxPtr add_vertex(gm::Point value, bool external = false);
    VtxPtr replace_vertex(VtxPtr from, VtxPtr to);
    void remove_obsolete_vertices();

    ElemPtr add_element(ElemPtr elem);
    EdgePtr add_edge(EdgePtr edge);

    // TODO: оптимизировать структуру сетки для ускорения поиска элемента по
    // узлу
    ConstVtxPtr vertex_by_index(VtxId id) const;
    std::vector<ConstElemPtr> elements_by_vertex(ConstVtxPtr vtx) const;
    std::vector<ConstElemTripletPtr>
    element_triplets_by_vertex(ConstVtxPtr vtx) const;

    const VertexContainer& vtx_view() const noexcept;
    const ElementContainer& elem_view() const noexcept;
    const EdgeElementContainer& edge_view() const noexcept;

    static void set_adjacent(VtxPtr lhs, VtxPtr rhs);
    static std::array<EdgePtr, elem_vtx> edges(ElemPtr ptr);

    rapidjson::Value& serialize(rapidjson::Value& result,
                                rapidjson::Value::AllocatorType& alloc) const;

private:
    VertexContainer vertices_;
    ElementContainer elems_;
    EdgeElementContainer edges_;
};

class QMSH_EXPORT Mesh::Vtx {
    friend class Mesh;

public:
    using AdjacentContainer = std::set<Mesh::VtxPtr>;

    explicit Vtx(gm::Point vertex = {}) noexcept;

    bool is_inserted() const noexcept;
    bool is_external() const noexcept;
    VtxId id() const noexcept;

    const gm::Point& value() const noexcept;
    Vtx& set_value(gm::Point new_value) noexcept;
    AdjacentContainer::size_type adj_count() const noexcept;

    AdjacentContainer::const_iterator begin() const;
    AdjacentContainer::const_iterator end() const;
    AdjacentContainer::const_iterator cbegin() const;
    AdjacentContainer::const_iterator cend() const;

    rapidjson::Value& serialize(rapidjson::Value& result,
                                rapidjson::Value::AllocatorType& alloc) const;

private:
    Vtx(VtxId id, bool external, gm::Point vertex) noexcept;
    void set_id(VtxId new_id);

    VtxId id_;
    bool external_;
    gm::Point vertex_;
    AdjacentContainer adjacent_;
};

} // namespace qmsh

#endif // QUADMESH_INCLUDE_QMSH_MESH_HPP_
