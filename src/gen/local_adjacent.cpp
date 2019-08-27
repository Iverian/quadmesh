#include "local_adjacent.hpp"

namespace qmsh {

void LocalAdjacent::set_adjacent(Mesh::VtxPtr lhs, Mesh::VtxPtr rhs)
{
    set_adjacent(d_[lhs], rhs->id());
    set_adjacent(d_[rhs], lhs->id());
    Mesh::set_adjacent(lhs, rhs);
}

void LocalAdjacent::set_adjacent(std::vector<Mesh::VtxId>& v, Mesh::VtxId obj)
{
    auto flag = std::all_of(std::begin(v), std::end(v),
                            [&obj](auto& i) { return i != obj; });
    if (flag) {
        v.push_back(obj);
    }
}

size_t LocalAdjacent::adjcount(Mesh::VtxPtr obj) const
{
    auto i = d_.find(obj);
    return (i == std::end(d_)) ? 0 : i->second.size();
}

} // namespace qmsh
