#ifndef QUADMESH_SRC_GEN_LOCAL_ADJACENT_HPP_
#define QUADMESH_SRC_GEN_LOCAL_ADJACENT_HPP_

#include <cmms/cycler.hpp>
#include <qmsh/mesh.hpp>

#include <map>
#include <vector>

namespace qmsh {
class LocalAdjacent {
public:
    using Data = std::unordered_map<VtxPtr, std::vector<size_t>>;

    void set_adjacent(VtxPtr lhs, VtxPtr rhs);
    void remove_adjacent(VtxPtr vtx, size_t id);
    size_t adjcount(VtxPtr obj) const;

    template <class BidirIt>
    void set_adjacent(BidirIt first, BidirIt last)
    {
        auto c = cmms::Cycler(first, last);
        auto prv = first;
        auto cur = c.next(first);
        if (prv != cur) {
            do {
                set_adjacent(*prv, *cur);
                prv = cur;
                cur = c.next(cur);
            } while (cur != first);
        }
    }

private:
    void set_adjacent(std::vector<size_t>& v, size_t obj);

    Data d_;
};
}

#endif // QUADMESH_SRC_GEN_LOCAL_ADJACENT_HPP_
