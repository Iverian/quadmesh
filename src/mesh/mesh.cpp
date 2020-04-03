#include <qmsh/mesh.hpp>

#include <cmms/cyclic_iterator.hpp>
#include <cmms/debug.hpp>
#include <qmsh/serialize.hpp>

#include <rapidjson/document.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/writer.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace qmsh {

VtxPtr Mesh::operator[](size_t id)
{
    return VtxPtr(&vertices_, id);
}

ConstVtxPtr Mesh::operator[](size_t id) const
{
    return ConstVtxPtr(&vertices_, id);
}

Element& Mesh::element(size_t id) noexcept
{
    return elements_[id];
}

const Element& Mesh::element(size_t id) const noexcept
{
    return elements_[id];
}

ElementPtr Mesh::element_ptr(size_t id) const noexcept
{
    auto& elem = elements_[id];

    ElementPtr result;
    std::transform(std::begin(elem), std::end(elem), std::begin(result),
                   [this](auto& x) { return VtxPtr(&vertices_, x); });
    return result;
}

Mesh::Vertices& Mesh::vertices()
{
    return vertices_;
}

const Mesh::Vertices& Mesh::vertices() const
{
    return vertices_;
}

Mesh::LookupRange Mesh::element_lookup(size_t id) const
{
    return element_lookup_.equal_range(id);
}

bool Mesh::vertex_inserted(VtxPtr ptr) const noexcept
{
    return ptr.parent() == &vertices_;
}

VtxPtr Mesh::insert_vertex(Vtx vertex)
{
    return VtxPtr(&vertices_, vertices_.insert(std::move(vertex)));
}

VtxPtr Mesh::insert_vertex(gm::Point value, bool external)
{
    return VtxPtr(&vertices_, vertices_.insert(Vtx(value, external)));
}

VtxPtr Mesh::replace_vertex(VtxPtr from, VtxPtr to)
{
    for (auto& i : from->adjacent()) {
        set_adjacent(to.id(), i);
        vertices_[i].erase_adjacent(from.id());
    }
    vertices_.erase(from.id());

    auto elements = element_lookup(from.id());
    for (auto elem = elements.first; elem != elements.second; ++elem) {
        for (auto& i : elements_[elem->second]) {
            if (i == from.id()) {
                i = to.id();
            }
        }
    }
    element_lookup_.erase(from.id());

    return to;
}

void Mesh::set_adjacent(size_t lhs, size_t rhs)
{
    vertices_.at(lhs).set_adjacent(rhs);
    vertices_.at(rhs).set_adjacent(lhs);
}

ElementPtr Mesh::insert_element(ElementPtr elem)
{
    for (auto& i : elem) {
        if (i.parent() != &vertices_) {
            i = insert_vertex(*i);
        }
    }
    for (size_t i = 0; i < elem_vtx; ++i) {
        set_adjacent(elem[i].id(), elem[(i + 1) % elem_vtx].id());
    }

    Element new_elem;
    std::transform(std::begin(elem), std::end(elem), std::begin(new_elem),
                   [](auto& x) { return x.id(); });

    auto elem_id = elements_.size();
    elements_.emplace_back(std::move(new_elem));
    for (auto& i : new_elem) {
        element_lookup_.emplace(i, new_elem);
    }

    return elem;
}

rapidjson::Value& Mesh::serialize(rapidjson::Value& result,
                                  rapidjson::Value::AllocatorType& alloc) const
{
    result.SetObject();

    rapidjson::Value vtxs(rapidjson::kArrayType);
    rapidjson::Value elems(rapidjson::kArrayType);

    for (auto& i : vertices_) {
        rapidjson::Value base, value;
        base.SetObject();
        base.AddMember("id", i.first, alloc);
        base.AddMember("value", i.second.serialize(value, alloc), alloc);
        vtxs.PushBack(base, alloc);
    }
    for (auto& i : elements_) {
        rapidjson::Value v;
        elems.PushBack(i.serialize(v, alloc), alloc);
    }

    result.AddMember("vertices", vtxs, alloc);
    result.AddMember("elements", elems, alloc);

    return result;
}

Vtx::Vtx(gm::Point value, bool external) noexcept
    : value_(value)
    , external_(external)
{
}

bool Vtx::external() const noexcept
{
    return external_;
}

const gm::Point& Vtx::value() const noexcept
{
    return value_;
}

const Vtx::Adjacent& Vtx::adjacent() const
{
    return adjacent_;
}

Vtx& Vtx::set_value(gm::Point new_value) noexcept
{
    value_ = new_value;
    return *this;
}

void Vtx::set_adjacent(size_t id)
{
    for (auto& i : adjacent_) {
        if (i == id) {
            return;
        } else if (i == Mesh::Vertices::npos) {
            i = id;
            return;
        }
    }
    adjacent_.push_back(id);
}

void Vtx::erase_adjacent(size_t id)
{
    if (id == Mesh::Vertices::npos) {
        adjacent_.clear();
    } else {
        for (auto& i : adjacent_) {
            if (i == id) {
                i = Mesh::Vertices::npos;
                break;
            }
        }
    }
}

rapidjson::Value& Vtx::serialize(rapidjson::Value& result,
                                 rapidjson::Value::AllocatorType& alloc) const
{
    result.SetObject();

    rapidjson::Value adjacent(rapidjson::kArrayType);
    rapidjson::Value value(rapidjson::kArrayType);

    for (auto& i : adjacent_) {
        if (i == Mesh::Vertices::npos) {
            continue;
        }
        adjacent.PushBack(i, alloc);
    }

    for (auto& i : value_) {
        value.PushBack(i, alloc);
    }

    result.AddMember("external", external_, alloc);
    result.AddMember("adjacent", adjacent, alloc);
    result.AddMember("value", value, alloc);

    return result;
}

VtxPtr::VtxPtr() noexcept
    : parent_(nullptr)
    , value_(Mesh::Vertices::npos)
{
}

VtxPtr::VtxPtr(Mesh::Vertices* parent, size_t id) noexcept
    : parent_(parent)
    , value_(id)
{
}

Mesh::Vertices::reference VtxPtr::operator*() const noexcept
{
    return (*parent_)[value_];
}

Mesh::Vertices::pointer VtxPtr::operator->() const noexcept
{
    return &(*parent_)[value_];
}

Mesh::Vertices* VtxPtr::parent() const noexcept
{
    return parent_;
}

size_t VtxPtr::id() const noexcept
{
    return value_;
}

ConstVtxPtr::ConstVtxPtr() noexcept
    : parent_(nullptr)
    , value_(Mesh::Vertices::npos)
{
}

ConstVtxPtr::ConstVtxPtr(const Mesh::Vertices* parent, size_t id) noexcept
    : parent_(parent)
    , value_(id)
{
}

Mesh::Vertices::const_reference ConstVtxPtr::operator*() const noexcept
{
    return (*parent_)[value_];
}

Mesh::Vertices::const_pointer ConstVtxPtr::operator->() const noexcept
{
    return &(*parent_)[value_];
}

const Mesh::Vertices* ConstVtxPtr::parent() const noexcept
{
    return parent_;
}

size_t ConstVtxPtr::id() const noexcept
{
    return value_;
}

} // namespace qmsh
