#ifndef QUADMESH_SRC_GEN_SURFACE_MESH_HPP_
#define QUADMESH_SRC_GEN_SURFACE_MESH_HPP_

#include <unordered_map>
#include <vector>

#include <gm/point.hpp>
#include <qmsh/holed_vector.hpp>

namespace qmsh {

class SurfaceMesh {
public:
    class Vtx;
    class VtxPtr;
    class ConstVtxPtr;

    using Id = size_t;
    using Vertices = HoledVector<Vtx>;
    using LookupMap = std::unordered_multimap<Id, Id>;
    using LookupRange
        = std::pair<LookupMap::const_iterator, LookupMap::const_iterator>;

    VtxPtr operator[](Id id);
    ConstVtxPtr operator[](Id id) const;

    VtxPtr insert_external(gm::Point value, double size, bool corner);
    VtxPtr insert_internal(gm::Point value, double size);
    VtxPtr replace_vertex(VtxPtr from, VtxPtr to);

    static void set_adjacent(VtxPtr lhs, VtxPtr rhs);

    LookupRange element_by_vertex(Id id) const;
    LookupRange element_by_vertex(const VtxPtr& ptr) const;
    LookupRange element_by_vertex(const ConstVtxPtr& ptr) const;

private:
};

class SurfaceMesh::Vtx {
    friend class SurfaceMesh;

public:
    using Adjacent = std::vector<SurfaceMesh::Id>;
    enum class ExternalCode : int {
        INTERNAL = 0,
        EDGE = 1,
        CORNER = 2,
    };

    Vtx() noexcept;

    const gm::Point& value() const noexcept;
    double size() const noexcept;
    bool boundary() const noexcept;
    bool external() const noexcept;
    ExternalCode external_code() const noexcept;
    Adjacent& adjacent() noexcept;
    const Adjacent& adjacent() const noexcept;

    Vtx& set_adjacent(SurfaceMesh::Id index);
    Vtx& erase_adjacent(SurfaceMesh::Id index);

    Vtx& set_boundary(bool value) noexcept;
    Vtx& set_value(const gm::Point& value) noexcept;

private:
    Vtx(gm::Point value, double size, ExternalCode code);

    gm::Point value_;
    double size_;
    bool boundary_;
    ExternalCode code_;
    Adjacent adjacent_;
};

class SurfaceMesh::VtxPtr {
public:
    VtxPtr();
    VtxPtr(const VtxPtr&) = default;
    VtxPtr(VtxPtr&&) noexcept = default;
    VtxPtr& operator=(const VtxPtr&) = default;
    VtxPtr& operator=(VtxPtr&&) noexcept = default;

    SurfaceMesh::Vertices::reference operator*() const noexcept;
    SurfaceMesh::Vertices::pointer operator->() const noexcept;
    const SurfaceMesh::Vertices& parent() const noexcept;
    SurfaceMesh::Id id() const noexcept;

    operator bool() const noexcept
    {
        return ptr_ && id_ != SurfaceMesh::Vertices::npos;
    }

    friend bool operator==(const VtxPtr& lhs, const VtxPtr& rhs);
    friend bool operator!=(const VtxPtr& lhs, const VtxPtr& rhs);

private:
    VtxPtr(SurfaceMesh::Vertices& parent, SurfaceMesh::Id id) noexcept;

    SurfaceMesh::Vertices* ptr_;
    SurfaceMesh::Id id_;
};

class SurfaceMesh::ConstVtxPtr {
public:
    ConstVtxPtr();
    ConstVtxPtr(const VtxPtr&);
    ConstVtxPtr(VtxPtr&&) noexcept;

    ConstVtxPtr(const ConstVtxPtr&) = default;
    ConstVtxPtr(ConstVtxPtr&&) noexcept = default;
    ConstVtxPtr& operator=(const ConstVtxPtr&) = default;
    ConstVtxPtr& operator=(ConstVtxPtr&&) noexcept = default;

    SurfaceMesh::Vertices::reference operator*() const noexcept;
    SurfaceMesh::Vertices::pointer operator->() const noexcept;
    const SurfaceMesh::Vertices& parent() const noexcept;
    SurfaceMesh::Id id() const noexcept;

    operator bool() const noexcept
    {
        return ptr_ && id_ != SurfaceMesh::Vertices::npos;
    }

    friend bool operator==(const ConstVtxPtr& lhs, const ConstVtxPtr& rhs);
    friend bool operator!=(const ConstVtxPtr& lhs, const ConstVtxPtr& rhs);

private:
    ConstVtxPtr(const SurfaceMesh::Vertices& parent,
                SurfaceMesh::Id id) noexcept;

    const SurfaceMesh::Vertices* ptr_;
    SurfaceMesh::Id id_;
};

} // namespace qmsh

#endif // QUADMESH_SRC_GEN_SURFACE_MESH_HPP_