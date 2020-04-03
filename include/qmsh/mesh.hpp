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
    using Super::Super;

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
    using Super::Super;

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
    ElementPtr element_ptr(size_t id) const noexcept;

    Vertices& vertices();
    const Vertices& vertices() const;

    LookupRange element_lookup(size_t id) const;

    bool vertex_inserted(VtxPtr ptr) const noexcept;
    VtxPtr insert_vertex(Vtx vertex);
    VtxPtr insert_vertex(gm::Point value, bool external = false);
    VtxPtr replace_vertex(VtxPtr from, VtxPtr to);

    void set_adjacent(size_t lhs, size_t rhs);

    ElementPtr insert_element(ElementPtr elem);

    rapidjson::Value& serialize(rapidjson::Value& result,
                                rapidjson::Value::AllocatorType& alloc) const;

private:
    Vertices vertices_;
    std::vector<Element> elements_;
    LookupMap element_lookup_;
};

class QMSH_EXPORT Vtx {
public:
    using Adjacent = std::vector<size_t>;

    explicit Vtx(gm::Point value, bool external = false) noexcept;

    bool external() const noexcept;
    const gm::Point& value() const noexcept;
    const Adjacent& adjacent() const;

    Vtx& set_value(gm::Point new_value) noexcept;
    void set_adjacent(size_t id);
    void erase_adjacent(size_t id = Mesh::Vertices::npos);

    rapidjson::Value& serialize(rapidjson::Value& result,
                                rapidjson::Value::AllocatorType& alloc) const;

private:
    gm::Point value_;
    bool external_;
    Adjacent adjacent_;
};

class QMSH_EXPORT VtxPtr {
public:
    VtxPtr() noexcept;
    VtxPtr(Mesh::Vertices* parent, size_t id) noexcept;
    Mesh::Vertices::reference operator*() const noexcept;
    Mesh::Vertices::pointer operator->() const noexcept;
    Mesh::Vertices* parent() const noexcept;
    size_t id() const noexcept;

private:
    Mesh::Vertices* parent_;
    size_t value_;
};

class QMSH_EXPORT ConstVtxPtr {
public:
    ConstVtxPtr() noexcept;
    ConstVtxPtr(const Mesh::Vertices* parent, size_t id) noexcept;
    Mesh::Vertices::const_reference operator*() const noexcept;
    Mesh::Vertices::const_pointer operator->() const noexcept;
    const Mesh::Vertices* parent() const noexcept;
    size_t id() const noexcept;

private:
    const Mesh::Vertices* parent_;
    size_t value_;
};

} // namespace qmsh

#endif // QUADMESH_INCLUDE_QMSH_MESH_HPP_
