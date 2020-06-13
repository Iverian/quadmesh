#include <gm/compare.hpp>
#include <gm/point.hpp>
#define _USE_MATH_DEFINES

#include "minimizer.hpp"
#include "surface_mesh_builder.hpp"
#include "util.hpp"

#include <cmms/cycler.hpp>
#include <cmms/debug.hpp>
#include <cmms/itertools.hpp>
#include <cmms/range_print.hpp>
#include <fmt/ostream.h>
#include <gen/generation_front.hpp>
#include <gm/abstract_surface.hpp>
#include <gm/axis.hpp>
#include <gm/mat.hpp>
#include <gm/plane.hpp>
#include <qmsh/inout.hpp>
#include <qmsh/mesh.hpp>
#include <rapidjson/document.h>
#include <rapidjson/rapidjson.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

#ifndef NDEBUG
static int _count = 0;
#define debug_mesh()                                                          \
    do {                                                                      \
        constexpr static auto path = "out";                                   \
        if (!_count) {                                                        \
            fs::remove_all(path);                                             \
            fs::create_directory(path);                                       \
        }                                                                     \
        qmsh::json_export(                                                    \
            {mesh_},                                                          \
            fmt::format("{}/{:05d}-{:03d}.json", path, _count++, __LINE__));  \
    } while (0)
#else
#define debug_mesh()
#endif

namespace qmsh {

SurfaceMeshBuilder::SurfaceMeshBuilder(
    cmms::LoggerRef log, const gm::AbstractSurface& s, bool same_sence,
    Mesh& mesh, const Config& conf,
    std::vector<std::vector<VtxPtr>> boundaries)
    : s_(s)
    , mesh_(mesh)
    , conf_(conf)
    , buf_()
    , adj_()
    , fronts_()
    , log_(log)
{
    fronts_.emplace_back(FrontType::OUTER, adj_, same_sence,
                         boundaries.front());
    std::transform(std::next(std::begin(boundaries)), std::end(boundaries),
                   std::back_inserter(fronts_), [this, same_sence](auto& i) {
                       return GenerationFront(FrontType::INNER, adj_,
                                              same_sence, i);
                   });
}

void SurfaceMeshBuilder::get()
{
    while (!fronts_.empty()) {
        for (auto it = std::begin(fronts_); it != std::end(fronts_);) {
            auto to_erase = false;
            auto last = std::begin(*it);

            while (true) {
                debug_mesh();
                check_if(it->size() % 2 == 0,
                         "front contains odd amount of vertices / {}",
                         it->size());

                eval_iangle(*it);
                if (place_seams(*it)) {
                    break;
                }
                if (closure_check(*it)) {
                    to_erase = true;
                    break;
                }
                it->resolve_row(adj_);

                auto [first, full_traverse] = it->choose_row(last, adj_);
                if (last = build_row(first, *it);
                    full_traverse || last == std::end(*it)) {
                    break;
                }
            }

            if (to_erase) {
                it = fronts_.erase(it);
            } else {
                ++it;
            }
        }
    }
    debug_mesh();
}

bool SurfaceMeshBuilder::place_seams(GenerationFront& front)
{
    static constexpr double eps[] = {M_PI / 4, M_PI / 3};

    auto result = false;
    auto c = cmms::make_cycler(front);
    auto i = std::begin(front);
    while (true) {
        auto seams_inserted = false;
        do {
            auto [first, last] = mesh_.element_lookup(i->global().id());
            if (i->iangle() < eps[std::distance(first, last) < 5]) {
                GenerationFront::VtxCache cache;

                auto p = c.prev(i);
                auto n = c.next(i);
                if (p->global()->external() || n->global()->external()) {
                    continue;
                }

                p->global()->set_value(s_.gproject(
                    (p->global()->value() + n->global()->value()) / 2));
                mesh_.replace_vertex(n->global(), p->global());

                auto removed_vtx = i->global();
                for (auto last = c.next(n); i != last;) {
                    auto ii = c.next(i);
                    front.erase(i);
                    i = ii;
                }
                smooth_internal_node(removed_vtx, front, cache);

                i = p;
                c = cmms::make_cycler(front);

                i->set_iangle(triple(c, i).first.iangle(front.type()),
                              conf_.angtol());
                result = seams_inserted = true;
                log_->debug("inserted seam");
                debug_mesh();
                if (front.size() <= edge_vtx) {
                    break;
                }
            }
        } while (i = c.next(i), !c.is_first(i));
        if (front.size() <= edge_vtx || !seams_inserted) {
            break;
        }
    }
    return result;
}

bool SurfaceMeshBuilder::closure_check(GenerationFront& front)
{
    auto s = front.size();
    auto result = false;

    switch (s) {
    case 0: {
        result = true;
        break;
    }
    case 1:
    case 3: {
        // FIXME: понять, что тут надо делать
        throw_fmt("unable to close front with single vertex");
        break;
    }
    case 2: {
        // TODO: add seams
        result = true;
        break;
    }
    case 4: {
        auto it = std::begin(front);
        ElementPtr tmp {{it->global(), std::next(it, 1)->global(),
                         std::next(it, 2)->global(),
                         std::next(it, 3)->global()}};
        tmp = mesh_.insert_element(tmp);
        result = true;
        break;
    }
    case 6: {
        result = true;
        six_vertices_closure(front);
        break;
    }
    default: {
        break;
    }
    }

    if (result) {
        front.clear();
    }
    return result;
}

void SurfaceMeshBuilder::smooth_boundary_nodes(FrontIter first, FrontIter last,
                                               GenerationFront& front,
                                               int depth) const
{
    GenerationFront::VtxCache internal_node_cache;
    auto c = cmms::make_cycler(front);

    for (auto i = first; i != last; i = c.next(i)) {
        smooth_boundary_node(c, i);
    }

    if (depth > 0) {
        for (auto i = first; i != last; i = c.next(i)) {
            if (!i->global()->external()) {
                for (auto& adj : i->global()->adjacent()) {
                    smooth_internal_node(mesh_[adj], front,
                                         internal_node_cache, depth);
                }
            }
        }
        for (auto i = first; i != last; i = c.next(i)) {
            smooth_boundary_node(c, i);
        }
    }
}

gm::Vec angular_smooth_delta(gm::Point vtx, double vtx_projected_length,
                             std::array<gm::Point, 3> surround, gm::Vec norm)
{
    auto v0 = gm::Vec(surround[1], surround[0]);
    auto v1 = gm::Vec(surround[1], vtx);
    auto v2 = gm::Vec(surround[1], surround[2]);

    auto c = gm::bisect({v0, v2, v1});
    auto line = gm::Line(c, surround[1]);
    auto [u, _] = line_solve(
        line, gm::Line(gm::Vec(surround[0], surround[2]), surround[0]));
    auto lq = gm::dist(line.f(u), surround[1]);
    c *= (vtx_projected_length > lq) ? ((vtx_projected_length + lq) / 2)
                                     : vtx_projected_length;
    return c - v1;
}

void SurfaceMeshBuilder::smooth_boundary_node(const FrontCycler& cycle,
                                              FrontIter it) const
{
    gm::Point delta;
    auto& vtx = it->global();
    auto& p0 = vtx->value();
    if (vtx->external() || vtx->adjacent().size() < 3) {
        return;
    }

    // FIXME: найти алгоритм сглаживания получше
    gm::Point p1;
    for (auto& i : vtx->adjacent()) {
        p1 += mesh_[i]->value();
    }
    p1 /= vtx->adjacent().size();

    if (vtx->adjacent().size() == 3) {
        auto triplets
            = mesh_.element_triplets_by_vertex(vtx, cycle.prev(it)->global());

        auto& pl = triplets[0][0]->value();
        auto& pc = triplets[0][2]->value();
        auto& pr = triplets[1][2]->value();

        auto ld = vtx->projected_length();
        auto la = gm::dist(pc, p1);

        auto d0 = p1 - p0;
        auto d1 = pc - p0 + (d0 + p0 - pc) * (ld / la);
        auto d2 = angular_smooth_delta(p0, ld, {pl, pc, pr},
                                       s_.normal(s_.project(pc)));
        delta = (d1 + d2) / 2;
    } else {
        delta = p1 - p0;
    }

    if (!gm::cmp::zero(delta)) {
        vtx->set_value(s_.gproject(p0 + delta));
        debug_mesh();
    }
}

void SurfaceMeshBuilder::smooth_internal_node(
    VtxPtr vptr, const GenerationFront& front,
    GenerationFront::VtxCache& smoothed_nodes, int depth) const
{
    if (depth <= 0 || vptr->external() || front.is_vertex_in_front(vptr)
        || smoothed_nodes.find(vptr) != std::end(smoothed_nodes)) {
        return;
    }

    auto delta = gm::Vec();
    auto norms = 0.;
    for (auto& adj : vptr->adjacent()) {
        auto v = mesh_[adj];
        auto c = gm::Vec(vptr->value(), v->value());
        if (v->external()) {
            std::array<gm::Point, 2> va;

            auto& g = v->value();
            auto n = s_.normal(s_.project(g));

            auto elems = mesh_.element_by_edge(EdgePtr({vptr, v}));
            for (auto i = 0; i < edge_vtx; ++i) {
                auto i0 = elems[i].has_item(vptr);
                auto i1 = elems[i].has_item(v);
                check_if(i0 != Element::npos && i1 != Element::npos,
                         "invalid lookup");

                if ((i0 + 1) % elem_vtx == i1) {
                    va[i] = elems[i][(i0 + elem_vtx - 1) % elem_vtx]->value();
                } else {
                    va[i] = elems[i][(i0 + 1) % elem_vtx]->value();
                }
            }

            if (!gm::cmp::zero(
                    gm::angle(n, gm::cross({g, va[0]}, {g, va[1]})))) {
                std::swap(va[0], va[1]);
            }

            auto d0 = angular_smooth_delta(
                vptr->value(), vptr->projected_length(), {va[0], g, va[1]}, n);
            c -= d0;
        }
        auto nc = gm::norm(c);
        delta += nc * c;
        norms += nc;
    }

    delta /= norms;
    if (!gm::cmp::zero(delta)) {
        vptr->set_value(s_.gproject(vptr->value() + delta));
        debug_mesh();
    }

    smoothed_nodes.insert(vptr);
    for (auto& adj : vptr->adjacent()) {
        smooth_internal_node(mesh_[adj], front, smoothed_nodes, depth - 1);
    }
}

void SurfaceMeshBuilder::six_vertices_closure(GenerationFront& front)
{
    std::vector<ptrdiff_t> iv;
    std::vector<FrontIter> ends;
    std::vector<VtxPtr> vtx;

    auto t = front.classify_primitive(true, adj_, &iv);
    auto c = cmms::make_cycler(front);
    auto first = std::begin(front);
    auto i = first;

    ends.reserve(iv.size());
    vtx.reserve(6);
    if (!iv.empty()) {
        do {
            if (i->type() == VtxType::END) {
                ends.emplace_back(i);
            }
        } while (i = c.next(i), !c.is_first(i));
        if (ends.size() > 1) {
            auto miv = *std::max_element(std::begin(iv), std::end(iv));
            while (c.distance(ends[0], ends[1]) != miv) {
                cmms::cycle_forward(ends);
                cmms::cycle_forward(iv);
            }
        }
        i = ends.front();
    } else {
        auto max = std::numeric_limits<double>::min();
        auto j = first;
        do {
            if (auto ia = j->iangle(); max < ia) {
                max = ia;
                i = j;
            }
        } while (j = c.next(j), !c.is_first(j));
    }
    do {
        vtx.emplace_back(i->global());
    } while (i = c.next(i), i != ends.front());

    switch (t) {
    case PrimitiveType::CIRCLE: {
        mesh_.insert_element({{vtx[0], vtx[1], vtx[2], vtx[3]}});
        mesh_.insert_element({{vtx[3], vtx[4], vtx[5], vtx[0]}});
        break;
    }
    case PrimitiveType::SEMI_CIRCLE: {
        if (iv[0] == 3 && iv[1] == 3) {
            mesh_.insert_element({{vtx[0], vtx[1], vtx[4], vtx[5]}});
            mesh_.insert_element({{vtx[1], vtx[2], vtx[3], vtx[4]}});
        } else if (iv[0] == 4 && iv[2] == 2) {
            mesh_.insert_element({{vtx[0], vtx[1], vtx[2], vtx[5]}});
            mesh_.insert_element({{vtx[2], vtx[3], vtx[4], vtx[5]}});
        } else if (iv[0] == 5 && iv[1] == 1) {
            mesh_.insert_element({{vtx[0], vtx[1], vtx[2], vtx[5]}});
            mesh_.insert_element({{vtx[2], vtx[3], vtx[4], vtx[5]}});
        } else {
            throw_fmt("unexpected intervals");
        }
        break;
    }
    case PrimitiveType::TRIANGLE: {
        if ((iv[0] == 2 && iv[1] == 2 && iv[2] == 2)
            || (iv[0] == 4 && iv[1] == 1 && iv[2] == 1)) {
            auto p = (vtx[1]->value() + vtx[3]->value() + vtx[5]->value()) / 3;
            vtx.emplace_back(mesh_.insert_internal_vertex(s_.gproject(p), 1));

            mesh_.insert_element({{vtx[0], vtx[1], vtx[6], vtx[5]}});
            mesh_.insert_element({{vtx[1], vtx[2], vtx[3], vtx[6]}});
            mesh_.insert_element({{vtx[3], vtx[4], vtx[5], vtx[6]}});
        } else if (iv[0] == 3 && iv[1] == 2 && iv[2] == 1) {
            mesh_.insert_element({{vtx[0], vtx[1], vtx[4], vtx[5]}});
            mesh_.insert_element({{vtx[1], vtx[2], vtx[3], vtx[4]}});
        } else if (iv[0] == 3 && iv[1] == 1 && iv[2] == 2) {
            mesh_.insert_element({{vtx[0], vtx[5], vtx[2], vtx[1]}});
            mesh_.insert_element({{vtx[2], vtx[5], vtx[4], vtx[3]}});
        } else {
            throw_fmt("unexpected intervals");
        }
        break;
    }
    case PrimitiveType::RECTANGLE: {
        if (iv[0] == 2 && iv[1] == 1 && iv[2] == 2 && iv[3] == 1) {
            mesh_.insert_element({{vtx[0], vtx[1], vtx[4], vtx[5]}});
            mesh_.insert_element({{vtx[1], vtx[2], vtx[3], vtx[4]}});
        } else if (iv[0] == 3 && iv[1] == 1 && iv[2] == 1 && iv[3] == 1) {
            auto p = (vtx[2]->value() + vtx[5]->value()) / 2;
            auto q = (vtx[1]->value() + vtx[4]->value()) / 2;
            vtx.emplace_back(mesh_.insert_internal_vertex(s_.gproject(p), 1));
            vtx.emplace_back(mesh_.insert_internal_vertex(s_.gproject(q), 1));

            mesh_.insert_element({{vtx[0], vtx[1], vtx[6], vtx[5]}});
            mesh_.insert_element({{vtx[1], vtx[2], vtx[7], vtx[6]}});
            mesh_.insert_element({{vtx[2], vtx[3], vtx[4], vtx[7]}});
            mesh_.insert_element({{vtx[4], vtx[5], vtx[6], vtx[7]}});
        } else if (iv[0] == 2 && iv[1] == 2 && iv[2] == 1 && iv[3] == 1) {
            auto p = (vtx[1]->value() + vtx[3]->value() + vtx[5]->value()) / 3;
            vtx.emplace_back(mesh_.insert_internal_vertex(s_.gproject(p), 1));

            mesh_.insert_element({{vtx[0], vtx[1], vtx[6], vtx[5]}});
            mesh_.insert_element({{vtx[1], vtx[2], vtx[3], vtx[6]}});
            mesh_.insert_element({{vtx[3], vtx[4], vtx[5], vtx[6]}});
        } else {
            throw_fmt("unexpected intervals");
        }
        break;
    }
    case PrimitiveType::NIL: {
        throw_fmt("undefined primitive type");
        break;
    }
    }

    {
        GenerationFront::VtxCache cache;
        for (auto& j : vtx) {
            smooth_internal_node(j, front, cache);
        }
    }
}

std::pair<Triple, gm::Plane>
SurfaceMeshBuilder::triple(const FrontCycle& it) const
{
    return triple(std::prev(it).iter(), it.iter(), std::next(it).iter());
}

std::pair<Triple, gm::Plane>
SurfaceMeshBuilder::triple(const FrontCycler& c, const FrontIter& i) const
{
    return triple(c.prev(i), i, c.next(i));
}

std::pair<Triple, gm::Plane>
SurfaceMeshBuilder::triple(FrontIter i, FrontIter j, FrontIter k) const
{
    return triple(i->value(), j->value(), k->value());
}

std::pair<Triple, gm::Plane>
SurfaceMeshBuilder::triple(gm::Point i, gm::Point j, gm::Point k) const
{
    Triple result;
    auto t = gm::Plane(s_.tangent(s_.project(j)));
    result.p[0] = t.gproject(i);
    result.p[1] = j;
    result.p[2] = t.gproject(k);
    result.norm = t.ax().z();

    return {result, t};
}

void SurfaceMeshBuilder::eval_iangle(GenerationFront& front)
{
    auto it = front.cycle();
    do {
        it->set_iangle(triple(it).first.iangle(front.type()), conf_.angtol());
    } while (++it, it.iter() != it.first());
}

void SurfaceMeshBuilder::eval_and_resolve(GenerationFront& front)
{
    auto it = front.cycle();
    do {
        it->set_iangle(triple(it).first.iangle(front.type()), conf_.angtol())
            .resolve(adj_.adjcount(it->global()));
    } while (++it, it.iter() != it.first());
}

FrontIter SurfaceMeshBuilder::build_row(FrontIter first,
                                        GenerationFront& front)
{
    auto t = front.type();
    AddElementResult last_state {true, false, true};

    auto c = cmms::make_cycler(front);
    ElementPtr tmp;

    buf_.clear();

    // Случай, когда на фронте нет концов: создаем на ровном месте
    // четырехугольник и выходим для перерасчета типов узлов
    if (first->type() == VtxType::SIDE) {
        auto ii = first;
        auto ir = c.next(first);
        auto [tri, pi] = triple(c, ii);
        auto [trr, pr] = triple(c, ir);
        auto vi = project_bisect(t, tri)[0];
        auto vr = project_bisect(t, trr)[0];

        tmp = {{ii->global(), ir->global(), vr, vi}};
        if ((last_state = add_element(front, ir, ii, tmp, pr))) {
            front.insert(ir, tmp[2]);
            front.insert(ir, tmp[3]);
            first = ir;
        } else if (!last_state.iterators_valid) {
            first = std::end(front);
        }
        return first;
    }

    auto ii = c.next(first);
    for (; ii != first; ii = c.next(ii)) {
        auto ir = c.next(ii);
        auto ip = c.prev(ii);
        auto iq = c.prev(ip);
        auto [tr, p] = triple(c, ii);

        switch (ii->type()) {
        case VtxType::SIDE: {
            auto v = project_bisect(t, tr);

            tmp = {{iq->global(), ip->global(), ii->global(), v[0]}};
            if (!(last_state = add_element(front, ii, iq, tmp, p))) {
                break;
            }

            v[0] = tmp[3];
            front.replace(ip, v[0]);
            break;
        }
        case VtxType::CORNER: {
            auto v = project_trisect(t, tr);

            tmp = {{iq->global(), ip->global(), ii->global(), v[0]}};
            if (!(last_state = add_element(front, ii, iq, tmp, p))) {
                break;
            }
            v[0] = tmp[3];
            front.replace(ip, v[0]);

            tmp = {{ii->global(), v[2], v[1], v[0]}};
            if (!(last_state = add_element(front, ii, ip, tmp, p))) {
                break;
            }
            v[1] = tmp[2];
            v[2] = tmp[1];
            front.insert(ii, v[1]);
            front.insert(ii, v[2]);

            break;
        }
        case VtxType::REVERSAL: {
            auto v = project_pentasect(t, tr);

            tmp = {{iq->global(), ip->global(), ii->global(), v[0]}};
            if (!(last_state = add_element(front, ii, iq, tmp, p))) {
                break;
            }
            v[0] = tmp[3];
            front.replace(ip, v[0]);

            tmp = {{ii->global(), v[2], v[1], v[0]}};
            if (!(last_state = add_element(front, ii, ip, tmp, p))) {
                break;
            }
            v[1] = tmp[2];
            v[2] = tmp[1];
            front.insert(ii, v[1]);
            front.insert(ii, v[2]);

            auto i2p = c.prev(ii);
            tmp = {{ii->global(), v[4], v[3], v[2]}};
            if (!(last_state = add_element(front, ii, i2p, tmp, p))) {
                break;
            }
            v[3] = tmp[2];
            v[4] = tmp[1];
            front.insert(ii, v[3]);
            front.insert(ii, v[4]);

            break;
        }
        case VtxType::END: {
            tmp = {{iq->global(), ip->global(), ii->global(), ir->global()}};
            if (!(last_state = add_element(front, ii, iq, tmp, p))) {
                break;
            }

            if (first == ip) {
                auto nip = front.erase(ip);
                first = (nip == std::end(front)) ? std::prev(nip) : nip;
            } else {
                front.erase(ip);
            }
            front.erase(ii);
            ii = ir;

            last_state.face_inserted = false;
            break;
        }
        case VtxType::NIL:
        case VtxType::END_SIDE:
        case VtxType::SIDE_CORNER:
        case VtxType::CORNER_REVERSAL: {
            throw_fmt("unresolved ambiguity (type: {})", ii->type());
            break;
        }
        }
        buf_.clear();
        // это нельзя написать в условии цикла т.к. там инкрементируется
        // итератор, но при last_state.iterators_valid=false все итераторы
        // невалидны
        if (!last_state) {
            if (!last_state.iterators_valid) {
                ii = std::end(front);
            }
            if (front.size()) {
                smooth_boundary_nodes(std::next(std::begin(front)),
                                      std::begin(front), front);
            }
            break;
        }
    }

    return ii;
}

SurfaceMeshBuilder::AddElementResult
SurfaceMeshBuilder::add_element(GenerationFront& front, FrontIter cur,
                                FrontIter prev2, ElementPtr& tmp,
                                const gm::Plane& tan)
{
    AddElementResult result {false, false, true};

    if (!tmp[0].belongs_to(mesh_) || !tmp[1].belongs_to(mesh_)
        || !tmp[2].belongs_to(mesh_) || !tmp[3].belongs_to(mesh_)) {
        for (auto it = std::begin(fronts_); it != std::end(fronts_); ++it) {
            auto& f = *it;
            auto is_same = (&front == &f);

            auto c = cmms::make_cycler(f);
            auto i = std::begin(f);
            decltype(i) j;
            do {
                j = c.next(i);
                EdgePtr v {{i->global(), j->global()}};
                for (size_t p = 0; p < elem_vtx; ++p) {
                    auto q = (p + 1) % elem_vtx;

                    auto& vp = tmp[p];
                    auto& vq = tmp[q];
                    if ((vp.belongs_to(mesh_) && vq.belongs_to(mesh_))
                        || (v[0] == vp || v[0] == vq)
                        || (v[1] == vp || v[1] == vq)) {
                        continue;
                    }

                    if (auto r = edge_intersection({{vp, vq}}, v); r) {
                        auto info = r.value();

                        switch (info.pcase) {
                        case ProximityCase::END_TO_END:
                        case ProximityCase::END_TO_MID: {
                            if (is_same) {
                                auto a = c.next(cur);
                                auto b = c.prev(cur);

                                if (a->type() == VtxType::END
                                    || b->type() == VtxType::END) {

                                    log_->debug("inserted seam at row build");

                                    auto x
                                        = (a->type() == VtxType::END) ? a : b;
                                    auto pos = (a->type() == VtxType::END)
                                        ? c.next(a)
                                        : c.prev(b);
                                    if (cur->global()->external()
                                        || pos->global()->external()) {
                                        goto label_insert;
                                    }

                                    cur->global()->set_value(
                                        s_.gproject((cur->global()->value()
                                                     + pos->global()->value())
                                                    / 2));
                                    mesh_.replace_vertex(pos->global(),
                                                         cur->global());
                                    log_->debug("replacing vertex {} with {}",
                                                pos->global().id(),
                                                cur->global().id());

                                    front.erase(pos);
                                    front.erase(x);

                                    result.iterators_valid = false;
                                    goto label_result;
                                } else {
                                    goto label_intersection;
                                }
                            }
                        }
                        case ProximityCase::MID_TO_MID: {
                        label_intersection:
                            append_to_mesh(tmp);
                            debug_mesh();

                            if (v[0]->external() || v[1]->external()) {
                                throw_fmt("mid to mid intersection with "
                                          "external edge");
                            }

                            auto a = s_.gproject(
                                (vq->value() + i->global()->value()) / 2);
                            auto b = s_.gproject(
                                (vp->value() + j->global()->value()) / 2);

                            i->global()->set_value(std::move(a));
                            j->global()->set_value(std::move(b));
                            mesh_.replace_vertex(vp, i->global());
                            mesh_.replace_vertex(vq, j->global());
                            vp = i->global();
                            vq = j->global();

                            if (is_same) {
                                auto [na, nb]
                                    = split_front(front, cur, j, prev2);
                                front.clear();
                                if (na.size() != 1) {
                                    fronts_.emplace_back(std::move(na));
                                }
                                if (nb.size() != 1) {
                                    fronts_.emplace_back(std::move(nb));
                                }
                                log_->debug("split front");
                            } else {
                                merge_fronts(front, cur, f, i);
                                fronts_.erase(it);
                                log_->debug("merge fronts");
                            }

                            result.iterators_valid = false;
                            goto label_result;

                            break;
                        }
                        }
                    }
                }
            } while (i = j, !c.is_first(i));
        }
    }
label_insert:
    append_to_mesh(tmp);
    result.face_inserted = true;

label_result:
    if (result) {
        log_->debug("inserted element / {}", tmp);
    } else {
        log_->debug("element not inserted");
    }

    debug_mesh();
    return result;
}

void SurfaceMeshBuilder::append_to_mesh(ElementPtr& tmp)
{
    tmp = mesh_.insert_element(tmp);
    for (auto i = 0; i < elem_vtx; ++i) {
        adj_.set_adjacent(tmp[i], tmp[(i + 1) % elem_vtx]);
    }
}

std::optional<DistResult>
SurfaceMeshBuilder::edge_intersection(const EdgePtr& a, const EdgePtr& b)
{
    static constexpr auto f_dist = 1.0;
    static constexpr auto f_angle = 0.25;
    static constexpr auto f_case = 0.1;

    auto la = gm::Line(gm::Vec(a[0]->value(), a[1]->value()), a[0]->value());
    auto lb = gm::Line(gm::Vec(b[0]->value(), b[1]->value()), b[0]->value());
    auto r = unary_segment_dist(la, lb);

    auto uv = gm::cos(la.dir(), lb.dir());
    auto w = (gm::dist(a[0]->value(), a[1]->value())
              + gm::dist(b[0]->value(), b[1]->value()))
        / 2;
    auto s = (sqrt(6) - 2) * w;
    auto c = f_dist * r.dist / s + f_angle * (1 - uv) + f_case * int(r.pcase);

    return (c < 1) ? std::optional<DistResult>(r) : std::nullopt;
}

VtxPtr SurfaceMeshBuilder::tmp(gm::Point vertex, double projected_length)
{
    return buf_(vertex, projected_length);
}

std::array<VtxPtr, 1> SurfaceMeshBuilder::project_bisect(FrontType t,
                                                         const Triple& tr)
{
    auto angle = tr.iangle(t) / 2;

    auto d = (dist(tr[0], tr[1]) + dist(tr[1], tr[2])) / (2 * sin(angle));
    auto r = s_.gproject(tr[1]
                         + d
                             * gm::dot(gm::Mat::rotate(angle, tr.norm),
                                       gm::unit({tr[1], tr[2]})));

    log_->debug("at bisect / {}", tr);

    return {tmp(r, gm::dist(tr[1], r))};
}

std::array<VtxPtr, 3> SurfaceMeshBuilder::project_trisect(FrontType t,
                                                          const Triple& tr)
{
    auto angle = tr.iangle(t) / 3;
    auto rmat = gm::Mat::rotate(angle, tr.norm);
    auto d = (dist(tr[0], tr[1]) + dist(tr[1], tr[2])) / (2 * sin(angle));

    auto u2 = gm::dot(rmat, gm::unit({tr[1], tr[2]}));
    auto u0 = gm::dot(rmat, u2);
    auto u1 = unit(u0 + u2);

    log_->debug("at trisect / {}", tr);

    std::array<gm::Point, 3> r {{s_.gproject(tr[1] + d * u0),
                                 s_.gproject(tr[1] + sqrt(2) * d * u1),
                                 s_.gproject(tr[1] + d * u2)}};

    return {tmp(r[0], gm::dist(tr[1], r[0])), tmp(r[1], gm::dist(tr[1], r[1])),
            tmp(r[2], gm::dist(tr[1], r[2]))};
}

std::array<VtxPtr, 5> SurfaceMeshBuilder::project_pentasect(FrontType t,
                                                            const Triple& tr)
{
    auto angle = tr.iangle(t);
    auto rmat = gm::Mat::rotate(angle, tr.norm);
    auto u4 = gm::dot(rmat, gm::unit({tr[1], tr[2]}));
    auto u2 = gm::dot(rmat, u4);
    auto u0 = gm::dot(rmat, u2);
    auto u1 = gm::unit(u0 + u2);
    auto u3 = gm::unit(u2 + u4);

    auto d
        = (gm::dist(tr[0], tr[1]) + gm::dist(tr[1], tr[2])) / (2 * sin(angle));

    log_->debug("at pentasect / {}", tr);

    std::array<gm::Point, 5> r {
        {s_.gproject(tr[1] + d * u0), s_.gproject(tr[1] + sqrt(2) * d * u1),
         s_.gproject(tr[1] + d * u2), s_.gproject(tr[1] + d * u3),
         s_.gproject(tr[1] + d * u4)}};

    return {tmp(r[0], gm::dist(tr[1], r[0])), tmp(r[1], gm::dist(tr[1], r[1])),
            tmp(r[2], gm::dist(tr[1], r[2])), tmp(r[3], gm::dist(tr[1], r[3])),
            tmp(r[4], gm::dist(tr[1], r[4]))};
}

[[nodiscard]] std::pair<GenerationFront, GenerationFront>
SurfaceMeshBuilder::split_front(GenerationFront& front, FrontIter first,
                                FrontIter last, FrontIter last2)
{
    auto c = cmms::make_cycler(front);
    GenerationFront a(front.type(), adj_), b(front.type(), adj_);

    auto aend = std::end(a);
    auto it = first;
    for (; it != last; it = c.next(it)) {
        a.insert(aend, it->global());
    }
    auto bend = std::end(b);
    for (; it != last2; it = c.next(it)) {
        b.insert(bend, it->global());
    }

    return std::make_pair(a, b);
}

void SurfaceMeshBuilder::merge_fronts(GenerationFront& a, FrontIter apos,
                                      GenerationFront& b, FrontIter bpos)
{
    auto c = cmms::make_cycler(b);

    auto it = bpos;
    do {
        a.insert(apos, it->global());
    } while (it = c.prev(it), it != bpos);
    b.clear();
}

TempVertexBuffer::TempVertexBuffer()
    : buf_()
{
    buf_.reserve(init_size);
}

VtxPtr TempVertexBuffer::operator()(gm::Point vertex, double projected_length)
{
    return VtxPtr(buf_, buf_.emplace(vertex, projected_length));
}

void TempVertexBuffer::clear() noexcept
{
    buf_.clear();
}

} // namespace qmsh
