#include <gm/compare.hpp>
#include <gm/plane.hpp>
#include <quadmesh/quad_mesh.h>
#include <util/debug.h>
#include <util/itertools.h>

#include <fmt/ostream.h>

#include <iterator>
#include <map>

using namespace std;

ostream& operator<<(ostream& os, const QuadMesh::NodePtr& obj);

QuadMesh::NodesType::iterator QuadMesh::begin()
{
    return ::begin(nodes_);
}

QuadMesh::NodesType::const_iterator QuadMesh::begin() const
{
    return ::begin(nodes_);
}

QuadMesh::NodesType::iterator QuadMesh::end()
{
    return ::end(nodes_);
}

QuadMesh::NodesType::const_iterator QuadMesh::end() const
{
    return ::end(nodes_);
}

QuadMesh::NodesType::reference QuadMesh::operator[](size_t i)
{
    return nodes_[i];
}

QuadMesh::NodesType::const_reference QuadMesh::operator[](size_t i) const
{
    return nodes_[i];
}

size_t QuadMesh::size() const
{
    return nodes_.size();
}

QuadMesh::NodesType::reference QuadMesh::append(const gm::Point& v)
{
    auto vindex = nodes_.size();
    auto ptr = shared_ptr<Node>(new Node(shared_from_this(), v, vindex));
    nodes_.push_back(ptr);
    return nodes_.back();
}

QuadMesh::EdgeSoup QuadMesh::get_edge_soup() const
{
    EdgeSoup result;

    set<array<size_t, 2>> unique;
    for (auto& i : nodes_) {
        for (auto& j : i->adjacent_) {
            array<size_t, 2> obj = {i->index_, j};
            unique.insert(obj);
        }
    }
    result.resize(unique.size());
    transform(::begin(unique), ::end(unique), ::begin(result),
              [&](const auto& i) {
                  return array<gm::Point, 2> {nodes_[i[0]]->vertex_,
                                              nodes_[i[1]]->vertex_};
              });

    return result;
}

QuadMesh::VertexSoup QuadMesh::get_vertex_soup() const
{
    VertexSoup result;
    result.resize(nodes_.size());
    transform(::begin(nodes_), ::end(nodes_), ::begin(result),
              [&](const auto& i) { return i->vertex_; });
    return result;
}

void QuadMesh::remove(size_t index)
{
    remove(nodes_[index]);
}

void QuadMesh::remove(NodePtr node)
{
    for (auto& i : node->adjacent_) {
        nodes_[i]->adjacent_.erase(node->index_);
    }
    node->adjacent_.clear();
}

size_t QuadMesh::drop_obsolete()
{
    size_t result = 0;

    sort(::begin(nodes_), ::end(nodes_),
         [&](const auto& lhs, const auto& rhs) {
             auto a = lhs->adjacent().size();
             auto b = rhs->adjacent().size();
             return a > b;
         });

    auto i = nodes_.size() - 1;
    for (; i != size_t(-1); --i) {
        if (!nodes_[i]->adjacent().empty()) {
            break;
        }
    }
    if (i == size_t(-1)) {
        result = nodes_.size();
        nodes_.clear();
    } else {
        result = nodes_.size() - (i + 1);
        nodes_.erase(::begin(nodes_) + i + 1, ::end(nodes_));
    }

    fix_indices();

    return result;
}

void QuadMesh::fix_indices()
{
    map<size_t, size_t> indices;
    auto n = nodes_.size();
    for (size_t i = 0; i < n; ++i) {
        indices.insert({nodes_[i]->index_, i});
        nodes_[i]->index_ = i;
    }
    for (auto& i : nodes_) {
        set<size_t> new_adjacent;
        for (auto& j : i->adjacent_) {
            if (auto k = indices.find(j); k != ::end(indices)) {
                new_adjacent.insert(k->second);
            }
        }
        i->adjacent_ = new_adjacent;
    }
}

QuadMesh::Node::Node()
    : parent_()
    , index_(0)
    , vertex_()
    , adjacent_()
{
}

QuadMesh::Node::Node(shared_ptr<QuadMesh> parent, const gm::Point& v,
                     size_t vindex)
    : parent_(parent)
    , index_(vindex)
    , vertex_(v)
    , adjacent_()
{
}

size_t QuadMesh::Node::vindex() const
{
    return index_;
}

const gm::Point& QuadMesh::Node::v() const
{
    return vertex_;
}

shared_ptr<QuadMesh> QuadMesh::Node::parent() const
{
    return parent_.lock();
}

QuadMesh::NodesType QuadMesh::Node::adjacent_view() const
{
    NodesType result;
    auto p = parent();
    transform(::begin(adjacent_), ::end(adjacent_), ::begin(result),
              [&p](const auto& i) { return p->nodes_[i]; });
    return result;
}

const QuadMesh::Node::AdjacentType& QuadMesh::Node::adjacent() const
{
    return adjacent_;
}

size_t QuadMesh::Node::count() const
{
    return adjacent_.size();
}

void set_adjacent(QuadMesh::Node& lhs, QuadMesh::Node& rhs)
{
    lhs.adjacent_.insert(rhs.index_);
    rhs.adjacent_.insert(lhs.index_);
}

void set_adjacent(shared_ptr<QuadMesh::Node> lhs,
                  shared_ptr<QuadMesh::Node> rhs)
{
    set_adjacent(*lhs, *rhs);
}

void set_adjacent(shared_ptr<QuadMesh::Node> lhs,
                  const initializer_list<shared_ptr<QuadMesh::Node>>& rhs)
{
    for (auto& i : rhs) {
        set_adjacent(*lhs, *i);
    }
}

bool is_adjacent(const QuadMesh::Node& lhs, const QuadMesh::Node& rhs)
{
    return (lhs.adjacent_.find(rhs.index_) != ::end(lhs.adjacent_))
        && (rhs.adjacent_.find(lhs.index_) != ::end(rhs.adjacent_));
}

bool is_adjacent(shared_ptr<QuadMesh::Node> lhs,
                 shared_ptr<QuadMesh::Node> rhs)
{
    return is_adjacent(*lhs, *rhs);
}

bool operator==(const QuadMesh::Node& lhs, const QuadMesh::Node& rhs)
{
    auto plhs = lhs.parent();
    auto prhs = rhs.parent();

    return (plhs == prhs) && (lhs.index_ == rhs.index_);
}

bool operator!=(const QuadMesh::Node& lhs, const QuadMesh::Node& rhs)
{
    return !(lhs == rhs);
}

ostream& operator<<(ostream& os, const QuadMesh::Node& obj)
{
    fmt::print(os, "{{ \"index\": {}, \"vertex\": {}, \"adjacent\": {} }}",
               obj.index_, obj.vertex_,
               RangePrint(begin(obj.adjacent_), end(obj.adjacent_)));
    return os;
}

ostream& operator<<(ostream& os, const QuadMesh::NodePtr& obj)
{
    return os << (*obj);
}

ostream& operator<<(ostream& os, const QuadMesh& obj)
{
    return os << RangePrint(begin(obj.nodes_), end(obj.nodes_));
}