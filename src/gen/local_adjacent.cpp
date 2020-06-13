#include "local_adjacent.hpp"

#include <algorithm>
#include <iterator>

namespace qmsh {

void LocalAdjacent::set_adjacent(VtxPtr lhs, VtxPtr rhs)
{
    set_adjacent(d_[lhs], rhs.id());
    set_adjacent(d_[rhs], lhs.id());
    Mesh::set_adjacent(lhs, rhs);
}

void LocalAdjacent::remove_adjacent(VtxPtr vtx, size_t id)
{
    auto& v = d_[vtx];
    auto i = std::find(std::begin(v), std::end(v), id);
    if (i != std::end(v)) {
        std::swap(*i, v.back());
        v.pop_back();
    }
}

void LocalAdjacent::set_adjacent(std::vector<size_t>& v, size_t obj)
{
    auto flag = std::all_of(std::begin(v), std::end(v),
                            [&obj](auto& i) { return i != obj; });
    if (flag) {
        v.push_back(obj);
    }
}

size_t LocalAdjacent::adjcount(VtxPtr obj) const
{
    auto i = d_.find(obj);
    return (i == std::end(d_)) ? 0 : i->second.size();
}

} // namespace qmsh
