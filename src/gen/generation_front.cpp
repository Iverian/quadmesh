#define _USE_MATH_DEFINES

#include "generation_front.hpp"

#include <cmms/cartesian.hpp>
#include <cmms/itertools.hpp>
#include <gm/compare.hpp>
#include <gm/plane.hpp>
#include <qmsh/config.hpp>
#include <util/debug.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>

namespace qmsh {

GenerationFront::GenerationFront(FrontType type, LocalAdjacent& adj,
                                 bool same_sense,
                                 const std::vector<Mesh::VtxPtr>& vtxs)
    : type_(type)
    , vtxs_()
{
    if (!vtxs.empty()) {
        auto first = std::begin(vtxs);
        auto last = std::end(vtxs);

        adj.set_adjacent(first, last);
        if (same_sense) {
            std::transform(first, last, std::back_inserter(vtxs_),
                           [](auto& x) { return Vtx(x, true); });
        } else {
            std::transform(first, last, std::front_inserter(vtxs_),
                           [](auto& x) { return Vtx(x, true); });
        }
    }
}

FrontType GenerationFront::type() const noexcept
{
    return type_;
}

GenerationFront::Cycle GenerationFront::cycle()
{
    return ::CyclicIterator(std::begin(vtxs_), std::end(vtxs_));
}

GenerationFront& GenerationFront::insert(Iter pos, Mesh::VtxPtr ptr,
                                         bool external)
{
    vtxs_.insert(pos, Vtx(ptr, external));
    return *this;
}

GenerationFront& GenerationFront::erase(Iter pos)
{
    vtxs_.erase(pos);
    return *this;
}

GenerationFront& GenerationFront::clear() noexcept
{
    vtxs_.clear();
    return *this;
}

bool GenerationFront::empty() const noexcept
{
    return vtxs_.empty();
}

size_t GenerationFront::size() const noexcept
{
    return vtxs_.size();
}

GenerationFront::VtxContainer::iterator GenerationFront::begin()
{
    return std::begin(vtxs_);
}

GenerationFront::VtxContainer::iterator GenerationFront::end()
{
    return std::end(vtxs_);
}

GenerationFront::VtxContainer::const_iterator GenerationFront::begin() const
{
    return std::begin(vtxs_);
}

GenerationFront::VtxContainer::const_iterator GenerationFront::end() const
{
    return std::end(vtxs_);
}

GenerationFront::VtxContainer::const_iterator GenerationFront::cbegin() const
{
    return std::cbegin(vtxs_);
}

GenerationFront::VtxContainer::const_iterator GenerationFront::cend() const
{
    return std::end(vtxs_);
}

void GenerationFront::resolve_row(const LocalAdjacent& adj)
{
    auto p = PrimitiveType::NIL;

    if (type_ != FrontType::OUTER
        || (p = classify_primitive(false, adj)) == PrimitiveType::NIL) {
        for (auto& i : vtxs_) {
            auto c = adj.adjcount(i.global());
            i.resolve(c);
        }
    }
}

std::pair<GenerationFront::Iter, bool>
GenerationFront::choose_row(GenerationFront::Iter first,
                            const LocalAdjacent& adj)
{
    static constexpr auto max_pos = size_t(2);

    Iter result;
    bool full_traverse = false;
    size_t pos = 0;
    auto min = first;
    std::vector<Iter> corners_and_reversals;

    auto it = cycle().set_iter(first);
    do {
        if (it->iangle() < min->iangle()) {
            min = it.iter();
        }

        switch (it->type()) {
        case VtxType::END: {
            if (pos == 0) {
                result = it.iter();
            }
            ++pos;
            break;
        }
        case VtxType::CORNER:
        case VtxType::REVERSAL: {
            corners_and_reversals.push_back(it.iter());
            break;
        }
        default:
            break;
        }

        ++it;
        if (!full_traverse && it.iter() == it.first()) {
            full_traverse = true;
        }
    } while (it.iter() != it.first() && pos != max_pos);

    if (pos == 0) {
        if (!corners_and_reversals.empty()) {
            min = *std::min_element(std::begin(corners_and_reversals),
                                    std::end(corners_and_reversals),
                                    [](const auto& lhs, const auto& rhs) {
                                        return lhs->iangle() < rhs->iangle();
                                    });
        }
        result = min;
    }

    return {result, full_traverse};
}

std::optional<std::vector<std::vector<VtxType>>>
GenerationFront::primitive_variants(bool force, const LocalAdjacent& adj)
{
    std::optional<std::vector<std::vector<VtxType>>> result;
    std::vector<std::vector<VtxType>> variants;
    variants.reserve(size());

    for (auto& i : vtxs_) {
        auto t = i.type();
        std::vector<VtxType> j;

        switch (t) {
        case VtxType::END:
        case VtxType::SIDE:
            j.emplace_back(t);
            break;
        case VtxType::CORNER:
        case VtxType::REVERSAL:
            if (!force) {
                goto end;
            } else {
                j.emplace_back(VtxType::SIDE);
            }
            break;
        case VtxType::END_SIDE:
            j.emplace_back(VtxType::END);
            j.emplace_back(VtxType::SIDE);
            break;
        case VtxType::SIDE_CORNER:
            j.emplace_back(VtxType::SIDE);
            break;
        case VtxType::CORNER_REVERSAL:
            if (!force) {
                goto end;
            } else {
                j.emplace_back(VtxType::SIDE);
            }
            break;
        case VtxType::NIL:
            throw_fmt("nil type not supported");
            break;
        }
        variants.emplace_back(j);
    }
    result = cmms::cartesian(variants);

end:
    return result;
}

PrimitiveType
GenerationFront::classify_primitive(bool force, const LocalAdjacent& adj,
                                    std::vector<ptrdiff_t>* intervals)
{
    auto result = PrimitiveType::NIL;
    if (auto opt = primitive_variants(force, adj); opt) {
        std::pair minq{std::numeric_limits<ptrdiff_t>::max(),
                       std::numeric_limits<double>::max()};

        auto& prod = opt.value();
        auto& best_primitive = prod.front();

        for (auto& i : prod) {
            std::vector<ptrdiff_t> m;
            auto t = primitive_viability(i, adj, &m);

            if (t == PrimitiveType::NIL) {
                continue;
            }
            auto curq = primitive_quality(i, adj);
            if (curq < minq) {
                result = t;
                minq = curq;

                if (intervals != nullptr) {
                    intervals->clear();
                    intervals->resize(m.size());
                    std::copy(std::begin(m), std::end(m),
                              std::begin(*intervals));
                }
            }
        }
        if (result != PrimitiveType::NIL) {
            auto p = std::begin(best_primitive);
            auto pend = std::end(best_primitive);
            auto v = std::begin(vtxs_);
            auto vend = std::end(vtxs_);

            for (; p != pend && v != vend; ++p, ++v) {
                v->resolve(adj.adjcount(v->global()), *p, force);
            }
        }
    }

    return result;
}

PrimitiveType
GenerationFront::primitive_viability(const std::vector<VtxType>& primitive,
                                     const LocalAdjacent& adj,
                                     std::vector<ptrdiff_t>* intervals)
{
    auto result = PrimitiveType::NIL;
    std::vector<ptrdiff_t> end_indices;
    end_indices.reserve(elem_vtx);
    auto ps = primitive.size();

    for (size_t i = 0; i < ps; ++i) {
        if (primitive[i] == VtxType::END) {
            end_indices.emplace_back(i);
        }
    }
    if (auto s = end_indices.size(); s <= elem_vtx) {
        std::vector<ptrdiff_t> m(s);
        if (s != 0) {
            for (size_t i = 0; i < s - 1; ++i) {
                m[i] = end_indices[i + 1] - end_indices[i];
            }
            m[s - 1] = (ps + end_indices[0]) - end_indices[s - 1];
        }
        if (intervals != nullptr) {
            intervals->clear();
            intervals->resize(m.size());
            std::copy(std::begin(m), std::end(m), std::begin(*intervals));
        }

        switch (s) {
        case 0:
        case 1: {
            result = PrimitiveType::CIRCLE;
            break;
        }
        case 2: {
            ptrdiff_t n[6];
            n[1] = (3 * m[0] - m[1] >= 8) ? ((m[0] + m[1]) / 8)
                                          : ((m[0] - 2) / 2);
            n[2] = n[1];
            n[5] = (m[1] - 2 * n[1]) / 2;
            n[0] = (m[0] + m[1] - 4 * n[2] - 2 * n[6]) / 2;
            n[3] = (m[0] - m[1] + 2 * n[5]) / 2;
            n[4] = m[1] - 2 * n[1] - n[3];
            // TODO: разобраться, какое здесь должно быть условие
            if (ps <= 6 || true) {
                result = PrimitiveType::SEMI_CIRCLE;
            }
            break;
        }
        case 3: {
            if (ps <= 6
                || ((m[0] + m[1] >= m[2] + 2) && (m[0] + m[2] >= m[1] + 2)
                    && (m[1] + m[2] >= m[0] + 2))) {
                result = PrimitiveType::TRIANGLE;
            }
            break;
        }
        case 4: {
            if (ps <= 6
                || ((m[0] + m[1] - m[2] - m[3] >= 2)
                    && (m[0] + m[3] - m[1] - m[2] >= 2))) {
                result = PrimitiveType::RECTANGLE;
            }
            break;
        }
        default:
            break;
        }
    }

end:
    return result;
}

std::pair<ptrdiff_t, double>
GenerationFront::primitive_quality(const std::vector<VtxType>& primitive,
                                   const LocalAdjacent& adj)
{
    auto [p, pend] = cmms::iterpair(primitive);
    auto [v, vend] = cmms::iterpair(vtxs_);
    auto s = primitive.size();
    ptrdiff_t irregular_count = 0;
    double angle_deviation = 0;

    for (; p != pend && v != vend; ++p, ++v) {
        irregular_count += std::abs(
            ptrdiff_t(adj.adjcount(v->global()) + insert_number(v->type()))
            - elem_vtx);
        angle_deviation += std::abs(v->iangle() - perfect_iangle(*p));
    }

    return std::make_pair(irregular_count, angle_deviation / s);
}

Mesh::EdgePtr edge(const FrontCycle& it)
{
    return {{it->global(), std::next(it)->global()}};
}

Mesh::EdgePtr edge(const FrontCycler& c, const FrontIter& it)
{
    return {{it->global(), c.next(it)->global()}};
}

GenerationFront::Vtx::Vtx(Mesh::VtxPtr global, bool external)
    : global_(std::move(global))
    , external_(external)
    , iangle_(0)
    , type_(VtxType::NIL)
{
    check_if(global_->is_inserted(),
             "trying to insert temporary vertex into generation front");
}

gm::Point GenerationFront::Vtx::value() const noexcept
{
    return global_->value();
}

bool GenerationFront::Vtx::external() const noexcept
{
    return external_;
}
double GenerationFront::Vtx::iangle() const noexcept
{
    return iangle_;
}

VtxType GenerationFront::Vtx::type() const noexcept
{
    return type_;
}

bool GenerationFront::Vtx::is_ambiguous() const noexcept
{
    return (type_ == VtxType::END_SIDE || type_ == VtxType::SIDE_CORNER
            || type_ == VtxType::CORNER_REVERSAL);
}

Mesh::VtxPtr GenerationFront::Vtx::global() const noexcept
{
    return global_;
}

GenerationFront::Vtx&
GenerationFront::Vtx::set_iangle(double new_angle,
                                 const Config::AngTol& at) noexcept
{
    iangle_ = new_angle;
    base_type(at);

    return *this;
}

GenerationFront::Vtx&
GenerationFront::Vtx::resolve(size_t adjcnt, VtxType new_type, bool force)
{
    switch (type_) {
    case VtxType::END_SIDE: {
        if (new_type == VtxType::NIL) {
            type_ = (adjcnt >= 4) ? VtxType::END : VtxType::SIDE;
        } else if (force || new_type == VtxType::END
                   || new_type == VtxType::SIDE) {
            type_ = new_type;
        } else {
            throw_fmt("incorrect vertex type pair (old = {}, new = {})", type_,
                      new_type);
        }
        break;
    }
    case VtxType::SIDE_CORNER: {
        if (new_type == VtxType::NIL) {
            type_ = (adjcnt >= 3) ? VtxType::SIDE : VtxType::CORNER;
        } else if (force || new_type == VtxType::SIDE
                   || new_type == VtxType::CORNER) {
            type_ = new_type;
        } else {
            throw_fmt("incorrect vertex type pair (old = {}, new = {})", type_,
                      new_type);
        }
        break;
    }
    case VtxType::CORNER_REVERSAL: {
        if (new_type == VtxType::NIL) {
            type_ = (adjcnt >= 2) ? VtxType::CORNER : VtxType::REVERSAL;
        } else if (force || new_type == VtxType::CORNER
                   || new_type == VtxType::REVERSAL) {
            type_ = new_type;
        } else {
            throw_fmt("incorrect vertex type pair (old = {}, new = {})", type_,
                      new_type);
        }
        break;
    }
    case VtxType::NIL:
    case VtxType::END:
    case VtxType::SIDE:
    case VtxType::CORNER:
    case VtxType::REVERSAL:
        break;
    }

    return *this;
}

#define pow_n1(x) (((x) % 2) ? (-1) : (1))

void GenerationFront::Vtx::base_type(const Config::AngTol& at) noexcept
{
    type_ = VtxType::REVERSAL;
    for (size_t i = 0; i < at.size(); ++i) {
        auto limit = (i + 1) * M_PI_2 + pow_n1(i) * at[i];
        if (gm::cmp::le(iangle_, limit)) {
            type_ = VtxType(i + 1);
            break;
        }
    }
}

#undef pow_n1

std::ostream& operator<<(std::ostream& os, FrontType obj)
{
    static constexpr const char* values[] = {"INNER", "OUTER"};

    return os << values[size_t(obj)];
}

std::ostream& operator<<(std::ostream& os, VtxType obj)
{
    static constexpr const char* values[]
        = {"NIL",         "END",    "END_SIDE",        "SIDE",
           "SIDE_CORNER", "CORNER", "CORNER_REVERSAL", "REVERSAL"};

    return os << values[size_t(obj)];
}

rapidjson::Value&
GenerationFront::Vtx::serialize(rapidjson::Value& result,
                                rapidjson::Value::AllocatorType& alloc) const
{
    rapidjson::Value glob;
    auto s = cmms::to_string(type_);

    result.SetObject();
    result.AddMember("global", global_->serialize(glob, alloc), alloc);
    result.AddMember("external", external_, alloc);
    result.AddMember("internal_angle", iangle_, alloc);
    result.AddMember("type", rapidjson::StringRef(s.c_str(), s.length()),
                     alloc);

    return result;
}

ptrdiff_t insert_number(VtxType t)
{
    ptrdiff_t result;

    switch (t) {
    case VtxType::END:
        result = 0;
        break;
    case VtxType::SIDE:
        result = 1;
        break;
    case VtxType::CORNER:
        result = 3;
        break;
    case VtxType::REVERSAL:
        result = 5;
        break;
    default:
        throw_fmt("type {} is not supported", t);
        break;
    }

    return result;
}

double perfect_iangle(VtxType t)
{
    double result;

    switch (t) {
    case VtxType::END:
        result = M_PI_2;
        break;
    case VtxType::SIDE:
        result = M_PI;
        break;
    case VtxType::CORNER:
        result = M_PI_2 * 3;
        break;
    case VtxType::REVERSAL:
        result = M_PI * 2;
        break;
    default:
        throw_fmt("type {} is not supported", t);
        break;
    }

    return result;
}

} // namespace qmsh