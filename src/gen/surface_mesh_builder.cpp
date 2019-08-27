#define _USE_MATH_DEFINES

#include "surface_mesh_builder.hpp"
#include "minimizer.hpp"
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

namespace fs = std::filesystem;

#ifndef NDEBUG
static int _count = 0;
#define debug_mesh()                                                          \
    do {                                                                      \
        constexpr static auto path = "debug";                                 \
        if (!_count) {                                                        \
            fs::remove_all(path);                                             \
            fs::create_directory(path);                                       \
        }                                                                     \
        qmsh::json_export({mesh_}, fmt::format("debug/{}.json", _count++));   \
    } while (0)
#else
#define debug_mesh()
#endif

namespace qmsh {

SurfaceMeshBuilder::SurfaceMeshBuilder(
    cmms::LoggerRef log, const gm::AbstractSurface& s, bool same_sence,
    Mesh& mesh, const Config& conf,
    std::vector<std::vector<Mesh::VtxPtr>> boundaries)
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

                eval_iangle(*it);
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
    case 1: {
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
        Mesh::ElemPtr tmp {{it->global(), std::next(it, 1)->global(),
                            std::next(it, 2)->global(),
                            std::next(it, 3)->global()}};
        tmp = mesh_.add_element(tmp);
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

void SurfaceMeshBuilder::six_vertices_closure(GenerationFront& front)
{
    std::vector<ptrdiff_t> iv;
    std::vector<FrontIter> ends;
    std::vector<Mesh::VtxPtr> vtx;

    auto t = front.classify_primitive(true, adj_, &iv);
    auto c = cmms::make_cycler(front);
    auto first = std::begin(front);
    auto last = std::end(front);
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
            while (std::distance(ends[0], ends[1]) != miv) {
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
        mesh_.add_element({vtx[0], vtx[1], vtx[2], vtx[3]});
        mesh_.add_element({vtx[3], vtx[4], vtx[5], vtx[0]});
        break;
    }
    case PrimitiveType::SEMI_CIRCLE: {
        if (iv[0] == 3 && iv[1] == 3) {
            mesh_.add_element({vtx[0], vtx[1], vtx[4], vtx[5]});
            mesh_.add_element({vtx[1], vtx[2], vtx[3], vtx[4]});
        } else if (iv[0] == 4 && iv[2] == 2) {
            mesh_.add_element({vtx[0], vtx[1], vtx[2], vtx[5]});
            mesh_.add_element({vtx[2], vtx[3], vtx[4], vtx[5]});
        } else if (iv[0] == 5 && iv[1] == 1) {
            mesh_.add_element({vtx[0], vtx[1], vtx[2], vtx[5]});
            mesh_.add_element({vtx[2], vtx[3], vtx[4], vtx[5]});
        } else {
            throw_fmt("unexpected intervals");
        }
        break;
    }
    case PrimitiveType::TRIANGLE: {
        auto p = (vtx[1]->value() + vtx[3]->value() + vtx[5]->value()) / 3;
        vtx.emplace_back(mesh_.add_vertex(s_.gproject(p)));

        if ((iv[0] == 2 && iv[1] == 2 && iv[2] == 2)
            || (iv[0] == 4 && iv[1] == 1 && iv[2] == 1)) {
            mesh_.add_element({vtx[0], vtx[1], vtx[6], vtx[5]});
            mesh_.add_element({vtx[1], vtx[2], vtx[3], vtx[6]});
            mesh_.add_element({vtx[3], vtx[4], vtx[5], vtx[6]});
        } else if (iv[0] == 3 && iv[1] == 2 && iv[2] == 1) {
            mesh_.add_element({vtx[0], vtx[1], vtx[4], vtx[5]});
            mesh_.add_element({vtx[1], vtx[2], vtx[3], vtx[4]});
        } else {
            throw_fmt("unexpected intervals");
        }
        break;
    }
    case PrimitiveType::RECTANGLE: {
        if (iv[0] == 2 && iv[1] == 1 && iv[2] == 2 && iv[3] == 1) {
            mesh_.add_element({vtx[0], vtx[1], vtx[4], vtx[5]});
            mesh_.add_element({vtx[1], vtx[2], vtx[3], vtx[4]});
        } else if (iv[0] == 3 && iv[1] == 1 && iv[2] == 1 && iv[3] == 1) {
            auto p = (vtx[2]->value() + vtx[5]->value()) / 2;
            auto q = (vtx[1]->value() + vtx[4]->value()) / 2;
            vtx.emplace_back(mesh_.add_vertex(s_.gproject(p)));
            vtx.emplace_back(mesh_.add_vertex(s_.gproject(q)));

            mesh_.add_element({vtx[0], vtx[1], vtx[6], vtx[5]});
            mesh_.add_element({vtx[1], vtx[2], vtx[7], vtx[6]});
            mesh_.add_element({vtx[2], vtx[3], vtx[4], vtx[7]});
            mesh_.add_element({vtx[4], vtx[5], vtx[6], vtx[7]});
        } else if (iv[0] == 2 && iv[1] == 2 && iv[2] == 1 && iv[3] == 1) {
            auto p = (vtx[1]->value() + vtx[3]->value() + vtx[5]->value()) / 3;
            vtx.emplace_back(mesh_.add_vertex(s_.gproject(p)));

            mesh_.add_element({vtx[0], vtx[1], vtx[6], vtx[5]});
            mesh_.add_element({vtx[1], vtx[2], vtx[3], vtx[6]});
            mesh_.add_element({vtx[3], vtx[4], vtx[5], vtx[6]});
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
}

std::pair<Triple, gm::Plane>
SurfaceMeshBuilder::triple(const FrontCycle& it) const
{
    Triple result;
    auto t = gm::Plane(s_.tangent(s_.project(it->value())));
    result.p[0] = t.gproject(std::prev(it)->value());
    result.p[1] = it->value();
    result.p[2] = t.gproject(std::next(it)->value());
    result.norm = t.ax().z();

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
    Triple result;
    auto t = gm::Plane(s_.tangent(s_.project(j->value())));
    result.p[0] = t.gproject(i->value());
    result.p[1] = j->value();
    result.p[2] = t.gproject(k->value());
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
    auto is_end = false;
    auto iterators_valid = false;

    auto c = cmms::make_cycler(front);
    Mesh::ElemPtr tmp;

    buf_.clear();

    // all side projection
    if (first->type() == VtxType::SIDE) {
        auto ii = first;
        auto ir = c.next(first);
        auto [tri, pi] = triple(c, ii);
        auto [trr, pr] = triple(c, ir);
        auto vi = project_bisect(t, tri)[0];
        auto vr = project_bisect(t, trr)[0];

        tmp = {{ii->global(), ir->global(), vr, vi}};
        // TODO: почекать какую плоскость тут лучше использовать
        if (add_element(front, ir, ii, tmp, is_end, pr)) {
            front.insert(ir, tmp[2], false);
            front.insert(ir, tmp[3], false);
            first = ir;
        }
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
            if (!add_element(front, ii, iq, tmp, is_end, p)) {
                break;
            }
            v[0] = tmp[3];
            *ip = GenerationFront::Vtx(v[0], false);

            break;
        }
        case VtxType::CORNER: {
            auto v = project_trisect(t, tr);

            tmp = {{iq->global(), ip->global(), ii->global(), v[0]}};
            if (!add_element(front, ii, iq, tmp, is_end, p)) {
                break;
            }
            v[0] = tmp[3];
            *ip = GenerationFront::Vtx(v[0], false);
            if (is_end) {
                break;
            }

            tmp = {{ii->global(), v[2], v[1], v[0]}};
            if (!add_element(front, ii, ip, tmp, is_end, p)) {
                break;
            }
            v[1] = tmp[2];
            v[2] = tmp[1];
            front.insert(ii, v[1], false);
            front.insert(ii, v[2], false);

            break;
        }
        case VtxType::REVERSAL: {
            auto v = project_pentasect(t, tr);

            tmp = {{iq->global(), ip->global(), ii->global(), v[0]}};
            if (!add_element(front, ii, iq, tmp, is_end, p)) {
                break;
            }
            v[0] = tmp[3];
            *ip = GenerationFront::Vtx(v[0], false);
            if (is_end) {
                break;
            }

            tmp = {{ii->global(), v[2], v[1], v[0]}};
            if (!add_element(front, ii, ip, tmp, is_end, p)) {
                break;
            }
            v[1] = tmp[2];
            v[2] = tmp[1];
            front.insert(ii, v[1], false);
            front.insert(ii, v[2], false);
            if (is_end) {
                break;
            }

            tmp = {{ii->global(), v[4], v[3], v[2]}};
            if (!add_element(front, ii, c.prev(ii), tmp, is_end, p)) {
                break;
            }
            v[3] = tmp[2];
            v[4] = tmp[1];
            front.insert(ii, v[3], false);
            front.insert(ii, v[4], false);

            break;
        }
        case VtxType::END: {
            tmp = {{iq->global(), ip->global(), ii->global(), ir->global()}};
            if (!add_element(front, ii, iq, tmp, is_end, p)) {
                break;
            }
            front.erase(ip);
            front.erase(ii);
            ii = ir;
            iterators_valid = true;
            is_end = true;

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
        // итератор, но при is_end=true все итераторы невалидны
        if (is_end) {
            if (!iterators_valid) {
                ii = std::end(front);
            }
            break;
        }
    }

    return ii;
}

bool SurfaceMeshBuilder::add_element(GenerationFront& front, FrontIter cur,
                                     FrontIter prev2, Mesh::ElemPtr& tmp,
                                     bool& is_end, const gm::Plane& p)
{
    auto face_inserted = false;

    if (is_end) {
        goto result;
    }
    if (!is_convex(p, tmp)) {
        is_end = true;
        face_inserted = false;
        log_->warn("element is not convex");

        // FIXME: убрать
        append_to_mesh(tmp);
        debug_mesh();
        exit(1);

        goto result;
    }
    if (!tmp[0]->is_inserted() || !tmp[1]->is_inserted()
        || !tmp[2]->is_inserted() || !tmp[3]->is_inserted()) {
        for (auto it = std::begin(fronts_); it != std::end(fronts_); ++it) {
            auto& f = *it;
            auto is_same = (&front == &f);

            auto c = cmms::make_cycler(f);
            auto i = std::begin(f);
            decltype(i) j;
            do {
                j = c.next(i);
                Mesh::EdgePtr v = {{i->global(), j->global()}};
                for (size_t p = 0; p < elem_vtx; ++p) {
                    auto q = (p + 1) % elem_vtx;

                    auto& vp = tmp[p];
                    auto& vq = tmp[q];
                    if ((vp->is_inserted() && vq->is_inserted())
                        || (v[0] == vp || v[0] == vq)
                        || (v[1] == vp || v[1] == vq)) {
                        continue;
                    }

                    if (auto r = edge_intersection({{vp, vq}}, v); r) {
                        if (i->external() || j->external()) {
                            is_end = true;
                            face_inserted = false;
                            goto result;
                        }

                        auto a = s_.gproject(
                            (vq->value() + i->global()->value()) / 2);
                        auto b = s_.gproject(
                            (vp->value() + j->global()->value()) / 2);

                        i->global()->set_value(std::move(a));
                        j->global()->set_value(std::move(b));
                        vp = i->global();
                        vq = j->global();

                        if (is_same) {
                            auto [na, nb] = split_front(front, cur, j, prev2);
                            front = std::move(na);
                            fronts_.emplace_back(std::move(nb));
                            log_->debug("split front");
                        } else {
                            merge_fronts(front, cur, f, i);
                            fronts_.erase(it);
                            log_->debug("merge fronts");
                        }

                        is_end = true;
                        face_inserted = false;
                        goto result;
                    }
                }
            } while (i = j, !c.is_first(i));
        }
    }
    append_to_mesh(tmp);
    face_inserted = true;

result:
    if (face_inserted) {
        log_->debug("inserted element / {}", tmp);
    } else {
        log_->debug("element not inserted / {}", tmp);
    }

    debug_mesh();
    return face_inserted;
}

void SurfaceMeshBuilder::append_to_mesh(Mesh::ElemPtr& tmp)
{
    tmp = mesh_.add_element(tmp);
    for (auto i = 0; i < elem_vtx; ++i) {
        adj_.set_adjacent(tmp[i], tmp[(i + 1) % elem_vtx]);
    }
}

std::optional<std::pair<gm::Point, gm::Point>>
SurfaceMeshBuilder::edge_intersection(const Mesh::EdgePtr& a,
                                      const Mesh::EdgePtr& b)
{
    static constexpr auto f_dist = 1.0;
    static constexpr auto f_angle = 0.25;
    static constexpr auto f_case = 0.1;

    std::optional<std::pair<gm::Point, gm::Point>> result = std::nullopt;
    auto la = gm::Line(gm::Vec(a[0]->value(), a[1]->value()), a[0]->value());
    auto lb = gm::Line(gm::Vec(b[0]->value(), b[1]->value()), b[0]->value());
    auto [d, pa, pb, pcase] = unary_segment_dist(la, lb);

    auto uv = gm::cos(la.dir(), lb.dir());
    auto w = (gm::dist(a[0]->value(), a[1]->value())
              + gm::dist(b[0]->value(), b[1]->value()))
        / 2;
    auto s = (sqrt(6) - 2) * w;
    auto c = f_dist * d / s + f_angle * (1 - uv) + f_case * int(pcase);

    if (c < 1) {
        result = {la.f(pa), lb.f(pb)};
    }
    return result;
}

Mesh::VtxPtr SurfaceMeshBuilder::tmp(gm::Point vertex)
{
    return buf_(s_.gproject(vertex));
}

std::array<Mesh::VtxPtr, 1>
SurfaceMeshBuilder::project_bisect(FrontType t, const Triple& tr)
{
    auto angle = tr.iangle(t) / 2;

    auto d = (dist(tr[0], tr[1]) + dist(tr[1], tr[2])) / (2 * sin(angle));
    auto r = tr[1]
        + d
            * gm::dot(gm::Mat::rotate(angle, tr.norm),
                      gm::unit({tr[1], tr[2]}));

    log_->debug("at bisect / {}", tr);
    return {tmp(r)};
}

std::array<Mesh::VtxPtr, 3>
SurfaceMeshBuilder::project_trisect(FrontType t, const Triple& tr)
{
    auto angle = tr.iangle(t) / 3;
    auto rmat = gm::Mat::rotate(angle, tr.norm);
    auto d = (dist(tr[0], tr[1]) + dist(tr[1], tr[2])) / (2 * sin(angle));

    auto u2 = gm::dot(rmat, gm::unit({tr[1], tr[2]}));
    auto u0 = gm::dot(rmat, u2);
    auto u1 = unit(u0 + u2);

    log_->debug("at trisect / {}", tr);
    return {tmp(tr[1] + d * u0), tmp(tr[1] + sqrt(2) * d * u1),
            tmp(tr[1] + d * u2)};
}

std::array<Mesh::VtxPtr, 5>
SurfaceMeshBuilder::project_pentasect(FrontType t, const Triple& tr)
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
    return {tmp(tr[1] + d * u0), tmp(tr[1] + sqrt(2) * d * u1),
            tmp(tr[1] + d * u2), tmp(tr[1] + d * u3), tmp(tr[1] + d * u4)};
}

[[nodiscard]] std::pair<GenerationFront, GenerationFront>
SurfaceMeshBuilder::split_front(GenerationFront& front, FrontIter first,




                                FrontIter last, FrontIter last2){
    auto c = cmms::make_cycler(front);
    GenerationFront a(front.type(), adj_), b(front.type(), adj_);

    auto aend = std::end(a);
    auto it = first;
    for (; it != last; it = c.next(it)) {
        a.insert(aend, it->global(), it->external());
    }
    auto bend = std::end(b);
    for (; it != last2; it = c.next(it)) {
        b.insert(bend, it->global(), it->external());
    }

    return std::make_pair(a, b);
}

void SurfaceMeshBuilder::merge_fronts(GenerationFront& a, FrontIter apos,
                                      GenerationFront& b, FrontIter bpos)
{
    auto c = cmms::make_cycler(b);

    auto it = bpos;
    do {
        a.insert(apos, it->global(), it->external());
    } while (it = c.prev(it), it != bpos);
    b.clear();
}

TempVertexBuffer::TempVertexBuffer()
    : buf_()
    , pos_(0)
{
    buf_.reserve(init_size);
}

Mesh::VtxPtr TempVertexBuffer::operator()(gm::Point vertex)
{
    buf_.emplace_back(vertex);
    return &buf_.back();
}

void TempVertexBuffer::clear() noexcept
{
    buf_.clear();
    pos_ = 0;
}

} // namespace qmsh
