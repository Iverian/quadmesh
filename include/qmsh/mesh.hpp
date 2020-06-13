#ifndef QUADMESH_INCLUDE_QMSH_MESH_HPP_
#define QUADMESH_INCLUDE_QMSH_MESH_HPP_

#include "config.hpp"
#include "exports.hpp"
#include "holed_vector.hpp"
#include "serialize.hpp"

#include <gm/point.hpp>
#include <gm/shell.hpp>

#include <rapidjson/document.h>
#include <rapidjson/rapidjson.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <list>
#include <ostream>
#include <set>
#include <unordered_map>
#include <vector>

namespace qmsh {

static constexpr ptrdiff_t elem_vtx = 4;
static constexpr ptrdiff_t edge_vtx = 2;

class Vtx;
class VtxPtr;
class ConstVtxPtr;

template <size_t N>
class VarElement : public std::array<size_t, N> {
    using Super = std::array<size_t, N>;

public:
    static constexpr typename Super::size_type npos =
        typename Super::size_type(-1);

    VarElement()
        : Super {}
    {
    }
    VarElement(const Super& list)
        : Super {list}
    {
    }
    VarElement(Super&& list) noexcept
        : Super {list}
    {
    }

    typename Super::size_type
    has_item(typename Super::const_reference item) const
    {
        for (typename Super::size_type i = 0; i < N; ++i) {
            if ((*this)[i] == item) {
                return i;
            }
        }
        return npos;
    }

    rapidjson::Value& serialize(rapidjson::Value& result,
                                rapidjson::Value::AllocatorType& alloc) const
    {
        result.SetArray();
        for (auto& i : *this) {
            result.PushBack(i, alloc);
        }
        return result;
    }
};

template <class T, size_t N>
class VarElementPtr : public std::array<T, N> {
    using Super = std::array<T, N>;

public:
    static constexpr typename Super::size_type npos =
        typename Super::size_type(-1);

    VarElementPtr()
        : Super {}
    {
    }
    VarElementPtr(const Super& list)
        : Super {list}
    {
    }
    VarElementPtr(Super&& list) noexcept
        : Super {list}
    {
    }

    typename Super::size_type
    has_item(typename Super::const_reference item) const
    {
        for (typename Super::size_type i = 0; i < N; ++i) {
            if ((*this)[i] == item) {
                return i;
            }
        }
        return npos;
    }

    rapidjson::Value& serialize(rapidjson::Value& result,
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
using ElementPtr = VarElementPtr<VtxPtr, elem_vtx>;
using ConstElementPtr = VarElementPtr<ConstVtxPtr, elem_vtx>;

using Edge = VarElement<edge_vtx>;
using EdgePtr = VarElementPtr<VtxPtr, edge_vtx>;
using ConstEdgePtr = VarElementPtr<ConstVtxPtr, edge_vtx>;

using VtxTriplet = VarElementPtr<VtxPtr, elem_vtx - 1>;
using ConstVtxTriplet = VarElementPtr<ConstVtxPtr, elem_vtx - 1>;

class QMSH_EXPORT Mesh {
public:
    using Vertices = HoledVector<Vtx>;
    using LookupMap = std::unordered_multimap<size_t, size_t>;
    using LookupRange
        = std::pair<LookupMap::const_iterator, LookupMap::const_iterator>;

    VtxPtr operator[](size_t id);
    ConstVtxPtr operator[](size_t id) const;

    Element& element(size_t id) noexcept;
    const Element& element(size_t id) const noexcept;
    ConstElementPtr element_ptr(size_t id) const noexcept;

    Vertices& vertices();
    const Vertices& vertices() const;

    LookupRange element_lookup(size_t id) const;
    std::array<ConstElementPtr, edge_vtx> element_by_edge(EdgePtr ptr) const;

    VtxPtr insert_vertex(Vtx vertex);
    VtxPtr insert_external_vertex(gm::Point value, bool corner);
    VtxPtr insert_internal_vertex(gm::Point value, double projected_length);
    EdgePtr insert_edge(EdgePtr edge);
    ElementPtr insert_element(ElementPtr element);

    VtxPtr replace_vertex(VtxPtr from, VtxPtr to);
    std::vector<ConstVtxTriplet>
    element_triplets_by_vertex(ConstVtxPtr vertex) const;
    std::vector<ConstVtxTriplet>
    element_triplets_by_vertex(ConstVtxPtr vtx, ConstVtxPtr left) const;
    static void set_adjacent(VtxPtr lhs, VtxPtr rhs);

    rapidjson::Value& serialize(rapidjson::Value& result,
                                rapidjson::Value::AllocatorType& alloc) const;

    std::ostream& to_gmsh(std::ostream& os) const;

private:
    Vertices vertices_;
    std::vector<Edge> edges_;
    std::vector<Element> elements_;
    LookupMap element_lookup_;
};

enum class VtxBoundaryCode : int {
    INTERNAL = 0,
    EDGE = 1,
    CORNER = 2,
};

class QMSH_EXPORT Vtx {
public:
    using Adjacent = std::vector<size_t>;

    Vtx() noexcept;
    explicit Vtx(gm::Point value, bool corner) noexcept;
    Vtx(gm::Point value, double projected_length);

    Vtx(const Vtx&) = default;
    Vtx(Vtx&&) noexcept = default;

    Vtx& operator=(const Vtx&) = default;
    Vtx& operator=(Vtx&&) = default;

    double projected_length() const;
    bool external() const noexcept;
    VtxBoundaryCode code() const noexcept;
    const gm::Point& value() const noexcept;
    const Adjacent& adjacent() const;

    Vtx& set_value(gm::Point new_value) noexcept;
    void set_adjacent(size_t id);
    void erase_adjacent(size_t id = Mesh::Vertices::npos);

    rapidjson::Value& serialize(rapidjson::Value& result,
                                rapidjson::Value::AllocatorType& alloc) const;

private:
    gm::Point value_;
    VtxBoundaryCode code_;
    double projected_length_;
    Adjacent adjacent_;
};

class QMSH_EXPORT VtxPtr {
    friend class ConstVtxPtr;

public:
    VtxPtr() noexcept;
    VtxPtr(Mesh::Vertices& parent, size_t id) noexcept;
    VtxPtr(const VtxPtr&) = default;
    VtxPtr(VtxPtr&&) noexcept = default;
    VtxPtr& operator=(const VtxPtr&) = default;
    VtxPtr& operator=(VtxPtr&&) noexcept = default;

    Mesh::Vertices::reference operator*() const noexcept;
    Mesh::Vertices::pointer operator->() const noexcept;
    const Mesh::Vertices& parent() const noexcept;
    size_t id() const noexcept;
    bool belongs_to(const Mesh& mesh) const noexcept;

    friend bool operator==(const VtxPtr& lhs, const VtxPtr& rhs);
    friend bool operator!=(const VtxPtr& lhs, const VtxPtr& rhs);

private:
    Mesh::Vertices* parent_;
    size_t value_;
};

class QMSH_EXPORT ConstVtxPtr {
public:
    ConstVtxPtr() noexcept;
    ConstVtxPtr(const Mesh::Vertices& parent, size_t id) noexcept;
    ConstVtxPtr(const VtxPtr& obj) noexcept;
    ConstVtxPtr(VtxPtr&& obj) noexcept;

    ConstVtxPtr(const ConstVtxPtr&) = default;
    ConstVtxPtr(ConstVtxPtr&&) noexcept = default;
    ConstVtxPtr& operator=(const ConstVtxPtr&) = default;
    ConstVtxPtr& operator=(ConstVtxPtr&&) noexcept = default;

    Mesh::Vertices::const_reference operator*() const noexcept;
    Mesh::Vertices::const_pointer operator->() const noexcept;
    const Mesh::Vertices& parent() const noexcept;
    size_t id() const noexcept;
    bool belongs_to(const Mesh& mesh) const noexcept;

    friend bool operator==(const ConstVtxPtr& lhs, const ConstVtxPtr& rhs);
    friend bool operator!=(const ConstVtxPtr& lhs, const ConstVtxPtr& rhs);

private:
    const Mesh::Vertices* parent_;
    size_t value_;
};

} // namespace qmsh

namespace std {

template <>
struct hash<qmsh::VtxPtr> {
    bool operator()(const qmsh::VtxPtr& obj) const
    {
        return (144451 << 2) ^ (ph(&obj.parent()) << 1) ^ sh(obj.id());
    }

private:
    hash<const qmsh::Mesh::Vertices*> ph;
    hash<size_t> sh;
};

template <>
struct hash<qmsh::ConstVtxPtr> {
    bool operator()(const qmsh::ConstVtxPtr& obj) const
    {
        return (144451 << 2) ^ (ph(&obj.parent()) << 1) ^ sh(obj.id());
    }

private:
    hash<const qmsh::Mesh::Vertices*> ph;
    hash<size_t> sh;
};

} // namespace std

#endif // QUADMESH_INCLUDE_QMSH_MESH_HPP_
