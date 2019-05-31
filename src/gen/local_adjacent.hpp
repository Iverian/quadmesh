#ifndef QUADMESH_SRC_GEN_LOCAL_ADJACENT_HPP_
#define QUADMESH_SRC_GEN_LOCAL_ADJACENT_HPP_

#include <cmms/cycler.hpp>
#include <qmsh/mesh.hpp>

#include <map>
#include <vector>

namespace qmsh {

class LocalAdjacent {
public:
    using Data = std::map<Mesh::VtxPtr, std::vector<Mesh::VtxId>>;

    void set_adjacent(Mesh::VtxPtr lhs, Mesh::VtxPtr rhs);
    size_t adjcount(Mesh::VtxPtr obj) const;

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
    void set_adjacent(std::vector<Mesh::VtxId>& v, Mesh::VtxId obj);

    Data d_;
};
}

#endif // QUADMESH_SRC_GEN_LOCAL_ADJACENT_HPP_
