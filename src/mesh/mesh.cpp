#include <cstddef>
#include <qmsh/mesh.hpp>

#include <cmms/cyclic_iterator.hpp>
#include <cmms/debug.hpp>
#include <qmsh/serialize.hpp>

#include <rapidjson/document.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/writer.h>

#include <algorithm>
#include <type_traits>
#include <utility>
#include <vector>

namespace qmsh {

template <class T, class = std::enable_if_t<std::is_signed_v<T>>>
inline T bmod(T a, T b) noexcept
{
    auto c = a % b;
    return (c < T()) ? (b + c) : c;
}

VtxPtr Mesh::operator[](size_t id)
{
    return VtxPtr(vertices_, id);
}

ConstVtxPtr Mesh::operator[](size_t id) const
{
    return ConstVtxPtr(vertices_, id);
}

Element& Mesh::element(size_t id) noexcept
{
    return elements_[id];
}

const Element& Mesh::element(size_t id) const noexcept
{
    return elements_[id];
}

ConstElementPtr Mesh::element_ptr(size_t id) const noexcept
{
    auto& elem = elements_[id];

    ConstElementPtr result;
    std::transform(std::begin(elem), std::end(elem), std::begin(result),
                   [this](auto& x) { return ConstVtxPtr(vertices_, x); });
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

std::array<ConstElementPtr, edge_vtx> Mesh::element_by_edge(EdgePtr ptr) const
{
    std::array<ConstElementPtr, edge_vtx> result;
    size_t pos = 0;

    auto [a0, a1] = element_lookup(ptr[0].id());
    auto [b0, b1] = element_lookup(ptr[1].id());
    for (auto i = a0; i != a1; ++i) {
        for (auto j = b0; j != b1; ++j) {
            if (i->second == j->second) {
                result[pos++] = element_ptr(i->second);
                break;
            }
        }
    }

    return result;
}

VtxPtr Mesh::insert_vertex(Vtx vertex)
{
    return VtxPtr(vertices_, vertices_.emplace(std::move(vertex)));
}

VtxPtr Mesh::insert_external_vertex(gm::Point value, bool corner)
{
    return VtxPtr(vertices_, vertices_.emplace(std::move(value), corner));
}

VtxPtr Mesh::insert_internal_vertex(gm::Point value, double projected_length)
{
    return VtxPtr(vertices_,
                  vertices_.emplace(std::move(value), projected_length));
}

EdgePtr Mesh::insert_edge(EdgePtr edge)
{
    for (auto& i : edge) {
        if (!i.belongs_to(*this)) {
            i = insert_vertex(std::move(*i));
        }
    }
    set_adjacent(edge[0], edge[1]);
    edges_.push_back({{edge[0].id(), edge[1].id()}});

    return edge;
}

VtxPtr Mesh::replace_vertex(VtxPtr from, VtxPtr to)
{
    check_if(from.id() != to.id(), "unable to replace vertex with itself");
    check_if(!from->external() && !to->external(),
             "unable to replace from / to external vertex");
    check_if(&from.parent() == &vertices_ && &to.parent() == &vertices_,
             "invalid vertex pointers");

    for (auto& i : from->adjacent()) {
        auto& v = vertices_[i];
        v.erase_adjacent(from.id());

        to->set_adjacent(i);
        v.set_adjacent(to.id());
    }
    vertices_.erase(from.id());

    auto elements = element_lookup(from.id());
    for (auto elem = elements.first; elem != elements.second; ++elem) {
        element_lookup_.emplace(to.id(), elem->second);
        for (auto& i : elements_[elem->second]) {
            if (i == from.id()) {
                i = to.id();
            }
        }
    }
    element_lookup_.erase(from.id());
    for (auto& i : edges_) {
        for (auto& j : i) {
            if (j == from.id()) {
                j = to.id();
            }
        }
    }

    return to;
}

void Mesh::set_adjacent(VtxPtr lhs, VtxPtr rhs)
{
    check_if(&lhs.parent() == &rhs.parent(),
             "vertices belong to different meshes");
    lhs->set_adjacent(rhs.id());
    rhs->set_adjacent(lhs.id());
}

ElementPtr Mesh::insert_element(ElementPtr elem)
{
    for (auto& i : elem) {
        if (!i.belongs_to(*this)) {
            i = insert_vertex(*i);
        }
    }
    for (size_t i = 0; i < elem_vtx; ++i) {
        insert_edge(EdgePtr({elem[i], elem[(i + 1) % elem_vtx]}));
    }

    Element new_elem;
    std::transform(std::begin(elem), std::end(elem), std::begin(new_elem),
                   [](auto& x) { return x.id(); });

    elements_.emplace_back(std::move(new_elem));
    auto elem_id = elements_.size() - 1;
    for (auto& i : elements_.back()) {
        element_lookup_.emplace(i, elem_id);
    }

    return elem;
}

std::vector<ConstVtxTriplet>
Mesh::element_triplets_by_vertex(ConstVtxPtr vtx) const
{
    std::vector<ConstVtxTriplet> result;
    result.reserve(2);
    for (auto& i : elements_) {
        if (std::any_of(std::begin(i), std::end(i),
                        [&vtx](auto& x) { return x == vtx.id(); })) {
            ConstVtxTriplet elem;
            size_t pos = 0;
            for (auto& j : i) {
                if (j == vtx.id()) {
                    continue;
                }
                elem[pos++] = (*this)[j];
            }

            result.emplace_back(std::move(elem));
        }
    }
    return result;
}

std::vector<ConstVtxTriplet>
Mesh::element_triplets_by_vertex(ConstVtxPtr vtx, ConstVtxPtr left) const
{
    std::vector<ConstVtxTriplet> result;
    std::vector<size_t> element_id;

    element_id.reserve(2);
    result.reserve(2);

    auto [first, last] = element_lookup(vtx.id());
    for (auto i = first; i != last; ++i) {
        element_id.emplace_back(i->second);
    }

    auto init_id = left.id();
    auto size = element_id.size();
    for (size_t i = 0; i < size; ++i) {
        for (auto& j : element_id) {
            auto& e = elements_[j];

            auto l = e.has_item(init_id);
            if (l != Element::npos) {
                auto v = e.has_item(vtx.id());
                if ((v + 1) % elem_vtx == l) {
                    result.emplace_back(ConstVtxTriplet(
                        {(*this)[e[l]], (*this)[e[(l + 1) % elem_vtx]],
                         (*this)[e[(l + 2) % elem_vtx]]}));
                } else if ((l + 1) % elem_vtx == v) {
                    result.emplace_back(
                        ConstVtxTriplet({(*this)[e[(v + 3) % elem_vtx]],
                                         (*this)[e[(v + 2) % elem_vtx]],
                                         (*this)[e[(v + 1) % elem_vtx]]}));
                } else {
                    throw_fmt("left and target vertices are not adjacent");
                }

                std::swap(j, element_id.back());
                init_id = result.back().back().id();
                break;
            }
        }
        element_id.pop_back();
    }
    check_if(
        result.size() == size,
        "unable to find elements containing both target and left vertices");

    return result;
}

rapidjson::Value& Mesh::serialize(rapidjson::Value& result,
                                  rapidjson::Value::AllocatorType& alloc) const
{
    result.SetObject();

    rapidjson::Value vtxs(rapidjson::kArrayType);
    rapidjson::Value elems(rapidjson::kArrayType);
    rapidjson::Value edges(rapidjson::kArrayType);

    for (auto& i : vertices_.view()) {
        rapidjson::Value base, value, adjacent;
        base.SetObject();
        base.AddMember("id", i.second, alloc);
        base.AddMember("code", int(i.first.code()), alloc);

        value.SetArray();
        for (auto& j : i.first.value()) {
            value.PushBack(j, alloc);
        }
        base.AddMember("value", value, alloc);

        adjacent.SetArray();
        for (auto& j : i.first.adjacent()) {
            adjacent.PushBack(j, alloc);
        }
        base.AddMember("adjacent", adjacent, alloc);

        vtxs.PushBack(base, alloc);
    }
    for (auto& i : elements_) {
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

std::ostream& Mesh::to_gmsh(std::ostream& os) const
{
    fmt::print(os,
               "$MeshFormat\n4.1 0 {0}\n$EndMeshFormat\n"
               "$Nodes\n1 {1} {2} {3}\n2 1 0 {1}\n",
               sizeof(size_t), vertices_.filled_size(),
               vertices_.front_index() + 1, vertices_.back_index() + 1);
    for (auto& i : vertices_.view()) {
        fmt::print(os, "{}\n", i.second + 1);
    }
    for (auto& i : vertices_.view()) {
        auto& p = i.first.value();
        fmt::print(os, "{:.5f} {:.5f} {:.5f}\n", p[0], p[1], p[2]);
    }
    fmt::print(os, "$EndNodes\n$Elements\n1 {0} 1 {0}\n2 1 3 {0}\n",
               elements_.size());
    size_t k = 1;
    for (auto& i : elements_) {
        fmt::print(os, "{} {} {} {} {}\n", k, i[0] + 1, i[1] + 1, i[2] + 1,
                   i[3] + 1);
        ++k;
    }
    fmt::print(os, "$EndElements\n");

    return os;
}

Vtx::Vtx() noexcept
    : value_()
    , code_(VtxBoundaryCode::INTERNAL)
    , projected_length_(0)
    , adjacent_()
{
}

Vtx::Vtx(gm::Point value, bool corner) noexcept
    : value_(value)
    , code_(corner ? VtxBoundaryCode::CORNER : VtxBoundaryCode::EDGE)
    , projected_length_(0)
    , adjacent_()
{
    adjacent_.reserve(elem_vtx);
}

Vtx::Vtx(gm::Point value, double projected_length)
    : value_(value)
    , code_(VtxBoundaryCode::INTERNAL)
    , projected_length_(projected_length)
    , adjacent_()
{
    check_if(!gm::cmp::zero(projected_length_),
             "projected length can not be zero");
    adjacent_.reserve(elem_vtx);
}

double Vtx::projected_length() const
{
    check_if(!external(),
             "unable to retrieve projected length of external node");
    return projected_length_;
}

bool Vtx::external() const noexcept
{
    return code_ != VtxBoundaryCode::INTERNAL;
}

VtxBoundaryCode Vtx::code() const noexcept
{
    return code_;
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
                std::swap(i, adjacent_.back());
                break;
            }
        }
        if (adjacent_.back() == Mesh::Vertices::npos) {
            adjacent_.pop_back();
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

    result.AddMember("code", int(code_), alloc);
    result.AddMember("adjacent", adjacent, alloc);
    result.AddMember("value", value, alloc);

    return result;
}

VtxPtr::VtxPtr() noexcept
    : parent_(nullptr)
    , value_(Mesh::Vertices::npos)
{
    parent_ = nullptr;
}

VtxPtr::VtxPtr(Mesh::Vertices& parent, size_t id) noexcept
    : parent_(&parent)
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

const Mesh::Vertices& VtxPtr::parent() const noexcept
{
    return *parent_;
}

size_t VtxPtr::id() const noexcept
{
    return value_;
}

bool VtxPtr::belongs_to(const Mesh& mesh) const noexcept
{
    return parent_ == &mesh.vertices();
}

bool operator==(const VtxPtr& lhs, const VtxPtr& rhs)
{
    return lhs.parent_ == rhs.parent_ && lhs.value_ == rhs.value_;
}

bool operator!=(const VtxPtr& lhs, const VtxPtr& rhs)
{
    return !(lhs == rhs);
}

ConstVtxPtr::ConstVtxPtr() noexcept
    : parent_(nullptr)
    , value_(Mesh::Vertices::npos)
{
}

ConstVtxPtr::ConstVtxPtr(const Mesh::Vertices& parent, size_t id) noexcept
    : parent_(&parent)
    , value_(id)
{
}

ConstVtxPtr::ConstVtxPtr(const VtxPtr& obj) noexcept
    : parent_(obj.parent_)
    , value_(obj.value_)
{
}

ConstVtxPtr::ConstVtxPtr(VtxPtr&& obj) noexcept
    : parent_(obj.parent_)
    , value_(obj.value_)
{
    obj.parent_ = nullptr;
    obj.value_ = 0;
}

Mesh::Vertices::const_reference ConstVtxPtr::operator*() const noexcept
{
    return (*parent_)[value_];
}

Mesh::Vertices::const_pointer ConstVtxPtr::operator->() const noexcept
{
    return &(*parent_)[value_];
}

const Mesh::Vertices& ConstVtxPtr::parent() const noexcept
{
    return *parent_;
}

size_t ConstVtxPtr::id() const noexcept
{
    return value_;
}

bool ConstVtxPtr::belongs_to(const Mesh& mesh) const noexcept
{
    return parent_ == &mesh.vertices();
}

bool operator==(const ConstVtxPtr& lhs, const ConstVtxPtr& rhs)
{
    return lhs.parent_ == rhs.parent_ && lhs.value_ == rhs.value_;
}

bool operator!=(const ConstVtxPtr& lhs, const ConstVtxPtr& rhs)
{
    return !(lhs == rhs);
}

} // namespace qmsh
