#define _USE_MATH_DEFINES

#include "quad_surface_mesh_builder.h"
#include "quad_mesh_builder.h"

#include <util/debug.h>

#include <cmath>
#include <stdexcept>

using namespace std;

QuadSurfaceMeshBuilder& QuadSurfaceMeshBuilder::get()
{
    for (auto& i : bounds_) {
        if (i.size() > 6) {
            auto last = begin(i);
            while (true) {
                auto [first, full_traverse] = choose_row(i, last);
                last = generate_row(i, first);
                if (full_traverse) {
                    break;
                }
            }
        }
    }

    flush();
    return *this;
}

pair<QuadSurfaceMeshBuilder::PavingIter, bool>
QuadSurfaceMeshBuilder::choose_row(PavingBoundary& b, PavingIter first)
{
    PavingIter result;
    bool full_traverse = false;

    size_t pos = 0;
    auto min = first;
    vector<PavingIter> corners_and_reversals;
    auto c = cycle(b).set_iter(first);

    do {
        set_iangle(c, b.type);
        if (c->iangle < min->iangle) {
            min = c.iter();
        }

        switch (resolve_ambiguity(set_node_type(*c)).type) {
        case NodeType::END:
            if (pos == 0) {
                result = c.iter();
            }
            ++pos;
            break;
        case NodeType::CORNER:
        case NodeType::REVERSAL:
            corners_and_reversals.push_back(c.iter());
            break;
        default:
            break;
        }

        ++c;
        if (!full_traverse && c.iter() == begin(b)) {
            full_traverse = true;
        }
    } while (c.iter() != first && pos < 2);

    switch (pos) {
    case 0:
        if (!corners_and_reversals.empty()) {
            min = *min_element(begin(corners_and_reversals),
                               end(corners_and_reversals),
                               [](const auto& lhs, const auto& rhs) {
                                   return lhs->iangle < rhs->iangle;
                               });
        }
        result = min;
        break;
    case 1:
        break;
    default:
        break;
    }

    return make_pair(result, full_traverse);
}

QuadSurfaceMeshBuilder::PavingIter
QuadSurfaceMeshBuilder::generate_row(PavingBoundary& b, PavingIter first)
{
    PavingIter result;
    bool end_flag = false;

    for (auto c = next(cycle(b).set_iter(first)); c.iter() != first; ++c) {
        auto iprev = prev(c);
        auto iprev2 = prev(c, 2);

        switch (c->type) {
        case NodeType::END: {
            auto inext = next(c);
            set_local_adjacent(*iprev2, *inext);
            b.erase(iprev.iter());
            b.erase(c.iter());

            result = inext.iter();
            end_flag = true;
            break;
        }
        case NodeType::SIDE: {
            auto n = at_bisect(c);
            set_local_adjacent(n, {*c, *iprev2});
            *iprev = n;
            break;
        }
        case NodeType::CORNER: {
            auto n = at_trisect(c);
            set_local_adjacent(n[0], {*iprev2, *c, n[1]});
            set_local_adjacent(n[2], {n[1], *c});
            *iprev = n[0];
            b.insert(c.iter(), {n[1], n[2]});
            break;
        }
        case NodeType::REVERSAL: {
            auto n = at_pentasect(c);
            set_local_adjacent(n[0], {*iprev2, *c});
            set_local_adjacent(n[1], {n[0], n[2]});
            set_local_adjacent(n[2], {n[3], *c});
            set_local_adjacent(n[4], {n[3], *c});
            *iprev = n[0];
            b.insert(c.iter(), {n[1], n[2], n[3], n[4]});
            break;
        }
        case NodeType::NIL:
        case NodeType::END_SIDE:
        case NodeType::SIDE_CORNER:
        case NodeType::CORNER_REVERSAL: {
            THROW_(runtime_error, "unresolved ambiguity");
            break;
        }
        }
        if (end_flag) {
            break;
        }
    }

    return result;
}

QuadSurfaceMeshBuilder::PavingBoundaryNode
QuadSurfaceMeshBuilder::at_bisect(const PavingCycle& it) const
{
    auto [p, c, n, ax] = get_triple(it);
    auto angle = it->iangle / 2;
    auto u = ax.rotate_z(angle, Vec(c, n)).normalize();
    log_->info("node: {}, ax: {}, u: {}, norm(u): {}", it->vindex, ax, u,
               u.norm());
    CHECK_(!u.isnan(), "u is nan ( p: {}, c: {}, n: {} )", p, c, n);

    auto d = (dist(p, c) + dist(c, n)) / (2 * sin(angle));

    return append_b(c + u * d);
}

array<QuadSurfaceMeshBuilder::PavingBoundaryNode, 3>
QuadSurfaceMeshBuilder::at_trisect(const PavingCycle& it) const
{
    auto [p, c, n, ax] = get_triple(it);

    auto angle = it->iangle / 3;
    auto cn = unit({c, n});
    auto u2 = ax.rotate_z(angle, cn);
    auto u0 = ax.rotate_z(angle, u2);
    auto u1 = unit(u0 + u2);

    CHECK_(!u0.isnan(), "u0 is nan ( p: {}, c: {}, n: {} )", p, c, n);
    CHECK_(!u1.isnan(), "u1 is nan ( p: {}, c: {}, n: {} )", p, c, n);
    CHECK_(!u2.isnan(), "u2 is nan ( p: {}, c: {}, n: {} )", p, c, n);

    auto d = (dist(p, c) + dist(c, n)) / (2 * sin(angle));

    return {append_b(c + d * u0), append_b(c + sqrt(2) * d * u1),
            append_b(c + d * u2)};
}

array<QuadSurfaceMeshBuilder::PavingBoundaryNode, 5>
QuadSurfaceMeshBuilder::at_pentasect(const PavingCycle& it) const
{
    auto [p, c, n, ax] = get_triple(it);
    auto cp = unit({c, p}), cn = unit({c, n});

    auto u2 = ax.rotate_z(it->iangle / 2, cn);
    auto u0 = unit(cp + u2);
    auto u4 = unit(cn + u2);
    auto u1 = unit(u0 + u2);
    auto u3 = unit(u2 + u4);

    CHECK_(!u0.isnan(), "u0 is nan ( p: {}, c: {}, n: {} )", p, c, n);
    CHECK_(!u1.isnan(), "u1 is nan ( p: {}, c: {}, n: {} )", p, c, n);
    CHECK_(!u2.isnan(), "u2 is nan ( p: {}, c: {}, n: {} )", p, c, n);
    CHECK_(!u3.isnan(), "u3 is nan ( p: {}, c: {}, n: {} )", p, c, n);
    CHECK_(!u4.isnan(), "u4 is nan ( p: {}, c: {}, n: {} )", p, c, n);

    auto d = (dist(p, c) + dist(c, n)) / (2 * sin(it->iangle / 4));

    return {append_b(c + d * u0), append_b(c + sqrt(2) * d * u1),
            append_b(c + d * u2), append_b(c + d * u3), append_b(c + d * u4)};
}

double QuadSurfaceMeshBuilder::set_iangle(const PavingCycle& i,
                                          BoundaryType type)
{
    auto [p, c, n, ax] = get_triple(i);
    auto cp = Vec(c, p), cn = Vec(c, n);
    auto snorm = normal_at(c);

    auto result = angle(cp, cn);
    if (int(type) * dot(cross(cp, cn), snorm) < 0) {
        result = 2 * M_PI - result;
    }
    CHECK_(!isnan(result),
           "internal angle is nan ( p: {}, c: {}, n: {}, cp: "
           "{}, cn: {} )",
           p, c, n, cp, cn);
    i->iangle = result;
    return result;
}

#define POW_M1(n) (((n) % 2) ? (-1) : (1))

QuadSurfaceMeshBuilder::PavingBoundaryNode&
QuadSurfaceMeshBuilder::set_node_type(PavingBoundaryNode& node)
{
    node.type = NodeType::REVERSAL;
    auto& angtol = conf().get_angle_tolerances();
    for (size_t i = 0; i < angtol.size(); ++i) {
        auto limit = (i + 1) * M_PI_2 + POW_M1(i) * angtol[i];
        if (node.iangle <= limit) {
            node.type = NodeType(i);
            break;
        }
    }
    return node;
}

#undef POW_M1

QuadSurfaceMeshBuilder::PavingBoundaryNode&
QuadSurfaceMeshBuilder::resolve_ambiguity(PavingBoundaryNode& node)
{
    auto adjcnt = node.adjcnt + size_t(node.external);
    switch (node.type) {
    case NodeType::END_SIDE: {
        node.type = (adjcnt >= 4) ? NodeType::END : NodeType::SIDE;
        break;
    }
    case NodeType::SIDE_CORNER: {
        node.type = (adjcnt >= 3) ? NodeType::SIDE : NodeType::CORNER;
        break;
    }
    case NodeType::CORNER_REVERSAL: {
        node.type = (adjcnt >= 2) ? NodeType::CORNER : NodeType::REVERSAL;
        break;
    }
    case NodeType::NIL:
    case NodeType::END:
    case NodeType::SIDE:
    case NodeType::CORNER:
    case NodeType::REVERSAL:
        break;
    }
    return node;
}

void QuadSurfaceMeshBuilder::set_local_adjacent(PavingBoundaryNode& lhs,
                                                PavingBoundaryNode& rhs)
{
    local_.emplace_back(lhs.vindex, rhs.vindex);
    ++lhs.adjcnt;
    ++rhs.adjcnt;
}

void QuadSurfaceMeshBuilder::set_local_adjacent(
    PavingBoundaryNode& lhs,
    const std::initializer_list<std::reference_wrapper<PavingBoundaryNode>>&
        ilist)
{
    for (auto& rhs : ilist) {
        set_local_adjacent(lhs, rhs);
    }
}

void QuadSurfaceMeshBuilder::flush()
{
    for (auto& p : local_) {
        set_adjacent(at(p.first), at(p.second));
    }
}

QuadSurfaceMeshBuilder::PavingCycle
QuadSurfaceMeshBuilder::cycle(PavingBoundary& b) const
{
    return PavingCycle(begin(b), end(b));
}

tuple<Point, Point, Point, Axis>
QuadSurfaceMeshBuilder::get_triple(const PavingCycle& i) const
{
    auto p = at(prev(i)->vindex)->v();
    auto c = at(i->vindex)->v();
    auto n = at(next(i)->vindex)->v();
    auto s = tangent_at(c);

    return {s.gproject(p), c, s.gproject(n), s.ax()};
}

Plane QuadSurfaceMeshBuilder::tangent_at(const Point& p) const
{
    return surf()->tangent(surf()->project(p));
}

Vec QuadSurfaceMeshBuilder::normal_at(const Point& p) const
{
    return surf()->normal(surf()->project(p));
}

QuadMesh::NodePtr QuadSurfaceMeshBuilder::at(size_t index) const
{
    return (*mesh())[index];
}

QuadMesh::NodePtr
QuadSurfaceMeshBuilder::at(const PavingBoundaryNode& node) const
{
    return at(node.vindex);
}

shared_ptr<const AbstractSurface> QuadSurfaceMeshBuilder::surf() const
{
    return surf_;
}

const QuadmeshConfig& QuadSurfaceMeshBuilder::conf() const
{
    return parent_->conf();
}

shared_ptr<QuadMesh> QuadSurfaceMeshBuilder::mesh() const
{
    return parent_->mesh();
}

QuadMesh::NodePtr QuadSurfaceMeshBuilder::append(const Point& p) const
{
    auto sp = surf()->project(p);
    auto result = mesh()->append(surf()->f(sp));
    log_->info("append node: {} with p: {}, sp: {}", *result, p, sp);

    return result;
}

QuadSurfaceMeshBuilder::PavingBoundaryNode
QuadSurfaceMeshBuilder::append_b(const Point& p) const
{
    return PavingBoundaryNode(append(p), false);
}

QuadSurfaceMeshBuilder::QuadSurfaceMeshBuilder(
    QuadMeshBuilder& parent, std::shared_ptr<spdlog::logger> log)
    : parent_(&parent)
    , surf_(nullptr)
    , bounds_()
    , log_(log)
    , local_()
{
}

QuadSurfaceMeshBuilder&
QuadSurfaceMeshBuilder::set_surface(shared_ptr<const AbstractSurface> surface)
{
    surf_ = move(surface);
    return *this;
}

QuadSurfaceMeshBuilder&
QuadSurfaceMeshBuilder::set_bounds(vector<vector<QuadMesh::NodePtr>>&& bounds)
{
    bounds_.resize(bounds.size());
    auto i = begin(bounds_);
    auto j = begin(bounds);
    init_boundary(*i++, *j++, BoundaryType::OUTER);
    for (; i != end(bounds_); ++i, ++j) {
        init_boundary(*i, *j, BoundaryType::INNER);
    }
    return *this;
}

void QuadSurfaceMeshBuilder::init_boundary(
    PavingBoundary& b, const vector<QuadMesh::NodePtr>& nodes,
    BoundaryType type)
{
    b.type = type;
    b.resize(nodes.size());
    auto j = begin(b);
    for (auto i = begin(nodes); i != end(nodes); ++i, ++j) {
        *j = PavingBoundaryNode(*i, 2, true);
    }
}

QuadSurfaceMeshBuilder::PavingBoundaryNode::PavingBoundaryNode()
    : vindex(0)
    , iangle(0)
    , type(NodeType::NIL)
    , external(false)
{
}

QuadSurfaceMeshBuilder::PavingBoundaryNode::PavingBoundaryNode(
    QuadMesh::NodePtr ptr, size_t p_adjcnt, bool p_external)
    : vindex(ptr->vindex())
    , iangle(0)
    , type(NodeType::NIL)
    , external(p_external)
    , adjcnt(p_adjcnt)
{
}

QuadSurfaceMeshBuilder::PavingBoundaryNode&
QuadSurfaceMeshBuilder::PavingBoundaryNode::set_iangle(double a)
{
    iangle = a;
    return *this;
}

QuadSurfaceMeshBuilder::PavingBoundaryNode&
QuadSurfaceMeshBuilder::PavingBoundaryNode::set_type(NodeType t)
{
    type = t;
    return *this;
}
