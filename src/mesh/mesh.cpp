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

Mesh::Mesh()
    : vertices_()
    , elems_()
{
}

Mesh::VtxPtr Mesh::add_vertex(Vtx vtx)
{
    vtx.set_id(vertices_.size());
    vertices_.emplace_back(std::move(vtx));
    return &vertices_.back();
}

Mesh::VtxPtr Mesh::add_vertex(gm::Point vtx)
{
    vertices_.push_back(Vtx(vertices_.size(), vtx));
    return &vertices_.back();
}

Mesh::VtxPtr Mesh::replace_vertex(Mesh::VtxPtr from, Mesh::VtxPtr to)
{
    for (auto& i : from->adjacent_) {
        set_adjacent(to, i);
        i->adjacent_.erase(from);
    }
    from->adjacent_.clear();
    for (auto& elem : elems_) {
        for (auto& vtx : elem) {
            if (vtx == from->id_) {
                vtx = to->id_;
            }
        }
    }
    for (auto& edge : edges_) {
        for (auto& vtx : edge) {
            if (vtx == from->id_) {
                vtx = to->id_;
            }
        }
    }
    return to;
}

void Mesh::remove_obsolete_vertices()
{
    for (auto i = std::begin(vertices_); i != std::end(vertices_);) {
        if (!i->adj_count()) {
            i = vertices_.erase(i);
        } else {
            ++i;
        }
    }
}

Mesh::VtxView Mesh::get_view(gm::Point val) noexcept
{
    return VtxView(val, *this);
}

Mesh::VtxView Mesh::get_view(VtxPtr ptr) noexcept
{
    return VtxView(ptr, *this);
}

Mesh::EdgePtr Mesh::add_edge(EdgePtr edge)
{
    for (auto& i : edge) {
        if (!i->is_inserted()) {
            i = add_vertex(std::move(*i));
        }
    }
    set_adjacent(edge[0], edge[1]);
    edges_.push_back({{edge[0]->id(), edge[1]->id()}});

    return edge;
}

Mesh::ElemPtr Mesh::add_element(ElemPtr elem)
{
    for (auto& i : elem) {
        if (!i->is_inserted()) {
            i = add_vertex(std::move(*i));
        }
    }
    add_edge({elem[0], elem[1]});
    add_edge({elem[1], elem[2]});
    add_edge({elem[2], elem[3]});
    add_edge({elem[3], elem[0]});
    elems_.push_back(
        {{elem[0]->id(), elem[1]->id(), elem[2]->id(), elem[3]->id()}});

    return elem;
}

const Mesh::VertexContainer& Mesh::vtx_view() const noexcept
{
    return vertices_;
}

const Mesh::ElementContainer& Mesh::elem_view() const noexcept
{
    return elems_;
}

const Mesh::EdgeElementContainer& Mesh::edge_view() const noexcept
{
    return edges_;
}

void Mesh::set_adjacent(VtxPtr lhs, VtxPtr rhs)
{
    check_if(
        lhs->is_inserted() && rhs->is_inserted(),
        "insert vertices into existing mesh before setting them as adjacent");

    lhs->adjacent_.insert(rhs);
    rhs->adjacent_.insert(lhs);
}

std::array<Mesh::EdgePtr, elem_vtx> Mesh::edges(ElemPtr ptr)
{
    return {{{(ptr)[0], (ptr)[1]},
             {(ptr)[1], (ptr)[2]},
             {(ptr)[2], (ptr)[3]},
             {(ptr)[3], (ptr)[0]}}};
}

rapidjson::Value& Mesh::serialize(rapidjson::Value& result,
                                  rapidjson::Value::AllocatorType& alloc) const
{
    result.SetObject();

    rapidjson::Value vtxs(rapidjson::kArrayType);
    rapidjson::Value elems(rapidjson::kArrayType);
    rapidjson::Value edges(rapidjson::kArrayType);

    for (auto& i : vertices_) {
        rapidjson::Value v;
        vtxs.PushBack(i.serialize(v, alloc), alloc);
    }
    for (auto& i : elems_) {
        rapidjson::Value v;
        elems.PushBack(i.serialize(v, alloc), alloc);
    }
    for (auto& i : edges_) {
        rapidjson::Value v;
        edges.PushBack(i.serialize(v, alloc), alloc);
    }
    result.AddMember("vertices", vtxs, alloc);
    result.AddMember("elements", elems, alloc);
    result.AddMember("edges", edges, alloc);

    return result;
}

Mesh::Vtx::Vtx(gm::Point vertex) noexcept
    : id_(Mesh::npos)
    , vertex_(vertex)
{
}

Mesh::Vtx::Vtx(VtxId id, gm::Point vertex) noexcept
    : id_(id)
    , vertex_(vertex)
{
}

void Mesh::Vtx::set_id(VtxId new_id)
{
    check_ifd(id_ == Mesh::npos && new_id != Mesh::npos,
              "unable to change valid id or set id to invalid");

    id_ = new_id;
}

bool Mesh::Vtx::is_inserted() const noexcept
{
    return id_ != Mesh::npos;
}

Mesh::VtxId Mesh::Vtx::id() const noexcept
{
    return id_;
}

const gm::Point& Mesh::Vtx::value() const noexcept
{
    return vertex_;
}

Mesh::Vtx& Mesh::Vtx::set_value(gm::Point new_value) noexcept
{
    vertex_ = new_value;
    return *this;
}

size_t Mesh::Vtx::adj_count() const noexcept
{
    return adjacent_.size();
}

Mesh::Vtx::AdjacentContainer::const_iterator Mesh::Vtx::begin() const
{
    return std::begin(adjacent_);
}

Mesh::Vtx::AdjacentContainer::const_iterator Mesh::Vtx::end() const
{
    return std::end(adjacent_);
}

Mesh::Vtx::AdjacentContainer::const_iterator Mesh::Vtx::cbegin() const
{
    return std::cbegin(adjacent_);
}

Mesh::Vtx::AdjacentContainer::const_iterator Mesh::Vtx::cend() const
{
    return std::cend(adjacent_);
}

rapidjson::Value&
Mesh::Vtx::serialize(rapidjson::Value& result,
                     rapidjson::Value::AllocatorType& alloc) const
{
    result.SetObject();

    rapidjson::Value adjacent(rapidjson::kArrayType);
    rapidjson::Value value(rapidjson::kArrayType);

    for (auto& i : adjacent_) {
        adjacent.PushBack(i->id(), alloc);
    }
    for (auto& i : vertex_) {
        value.PushBack(i, alloc);
    }
    result.AddMember("id", id_, alloc);
    result.AddMember("adjacent", adjacent, alloc);
    result.AddMember("value", value, alloc);

    return result;
}

gm::Point& Mesh::VtxView::get() noexcept
{
    return val_;
}

const gm::Point& Mesh::VtxView::get() const noexcept
{
    return val_;
}

gm::Point& Mesh::VtxView::operator*() noexcept
{
    return val_;
}

const gm::Point& Mesh::VtxView::operator*() const noexcept
{
    return val_;
}

gm::Point* Mesh::VtxView::operator->() noexcept
{
    return &val_;
}

const gm::Point* Mesh::VtxView::operator->() const noexcept
{
    return &val_;
}

Mesh::VtxPtr Mesh::VtxView::update()
{
    if (ptr_) {
        ptr_->set_value(val_);
    } else {
        ptr_ = parent_->add_vertex(val_);
    }

    return ptr_;
}

Mesh::VtxView::operator bool() const noexcept
{
    return bool(ptr_);
}

Mesh::VtxView::VtxView(gm::Point val, Mesh& parent) noexcept
    : val_(val)
    , ptr_(nullptr)
    , parent_(&parent)
{
}

Mesh::VtxView::VtxView(Mesh::VtxPtr ptr, Mesh& parent) noexcept
    : val_(ptr->value())
    , ptr_(ptr)
    , parent_(&parent)
{
}

} // namespace qmsh
