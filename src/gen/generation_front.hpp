#ifndef QUADMESH_SRC_GEN_GENERATION_FRONT_HPP_
#define QUADMESH_SRC_GEN_GENERATION_FRONT_HPP_

#include "local_adjacent.hpp"

#include <cmms/cycler.hpp>
#include <cmms/to_string.hpp>
#include <gm/abstract_surface.hpp>
#include <gm/point.hpp>
#include <qmsh/config.hpp>
#include <qmsh/mesh.hpp>
#include <util/cyclic_iterator.hpp>

#include <list>
#include <ostream>
#include <string>
#include <utility>

namespace qmsh {

enum class FrontType : int { INNER, OUTER };

enum class VtxType : int {
    NIL,
    END,
    END_SIDE,
    SIDE,
    SIDE_CORNER,
    CORNER,
    CORNER_REVERSAL,
    REVERSAL
};

enum class PrimitiveType : int {
    NIL,
    CIRCLE,
    SEMI_CIRCLE,
    TRIANGLE,
    RECTANGLE
};

[[nodiscard]] ptrdiff_t insert_number(VtxType t);
[[nodiscard]] double perfect_iangle(VtxType t);

std::ostream& operator<<(std::ostream& os, FrontType obj);
std::ostream& operator<<(std::ostream& os, VtxType obj);

class GenerationFront {
public:
    class Vtx;

    using VtxContainer = std::list<Vtx>;
    using Iter = VtxContainer::iterator;
    using Cycle = CyclicIterator<Iter>;
    using Row = std::pair<Iter, Iter>;

    using iterator = VtxContainer::iterator;
    using const_iterator = VtxContainer::const_iterator;
    using pointer = VtxContainer::pointer;
    using const_pointer = VtxContainer::const_pointer;
    using reference = VtxContainer::reference;
    using const_reference = VtxContainer::const_reference;

    GenerationFront(FrontType type, LocalAdjacent& adj, bool same_sense = true,
                    const std::vector<Mesh::VtxPtr>& vtxs = {});

    FrontType type() const noexcept;
    Cycle cycle();

    GenerationFront& insert(Iter pos, Mesh::VtxPtr ptr, bool external = false);
    GenerationFront& erase(Iter pos);
    GenerationFront& clear() noexcept;
    bool empty() const noexcept;
    size_t size() const noexcept;

    VtxContainer::iterator begin();
    VtxContainer::iterator end();
    VtxContainer::const_iterator begin() const;
    VtxContainer::const_iterator end() const;
    VtxContainer::const_iterator cbegin() const;
    VtxContainer::const_iterator cend() const;

    void resolve_row(const LocalAdjacent& adj);
    std::pair<Iter, bool> choose_row(Iter first, const LocalAdjacent& adj);

    std::optional<std::vector<std::vector<VtxType>>>
    primitive_variants(bool force, const LocalAdjacent& adj);
    PrimitiveType classify_primitive(bool force, const LocalAdjacent& adj,
                                     std::vector<ptrdiff_t>* intervals
                                     = nullptr);
    PrimitiveType primitive_viability(const std::vector<VtxType>& primitive,
                                      const LocalAdjacent& adj,
                                      std::vector<ptrdiff_t>* intervals
                                      = nullptr);
    std::pair<ptrdiff_t, double>
    primitive_quality(const std::vector<VtxType>& primitive,
                      const LocalAdjacent& adj);

private:
    FrontType type_;
    VtxContainer vtxs_;
};

using FrontIter = GenerationFront::VtxContainer::iterator;
using FrontCycle = GenerationFront::Cycle;
using FrontCycler = cmms::Cycler<FrontIter>;
using FrontRow = GenerationFront::Row;

// [[nodiscard]] std::pair<GenerationFront, GenerationFront>
// split_front(GenerationFront& front, FrontIter first, FrontIter last,
//             FrontIter last2);
// void merge_fronts(GenerationFront& a, FrontIter apos, GenerationFront& b,
//                   FrontIter bpos);

Mesh::EdgePtr edge(const FrontCycle& it);
Mesh::EdgePtr edge(const FrontCycler& c, const FrontIter& it);

class GenerationFront::Vtx {
public:
    explicit Vtx(Mesh::VtxPtr global, bool external = false);

    gm::Point value() const noexcept;
    bool external() const noexcept;
    double iangle() const noexcept;
    VtxType type() const noexcept;
    bool is_ambiguous() const noexcept;
    Mesh::VtxPtr global() const noexcept;

    Vtx& set_iangle(double new_angle, const Config::AngTol& at) noexcept;
    Vtx& resolve(size_t local_adjcount, VtxType new_type = VtxType::NIL,
                 bool force = false);

    rapidjson::Value& serialize(rapidjson::Value& result,
                                rapidjson::Value::AllocatorType& alloc) const;

private:
    void base_type(const Config::AngTol& at) noexcept;

    Mesh::VtxPtr global_;
    bool external_;
    double iangle_;
    VtxType type_;
};

} // namespace qmsh

#endif // QUADMESH_SRC_GEN_GENERATION_FRONT_HPP_