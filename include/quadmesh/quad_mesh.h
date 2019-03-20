#ifndef QUADMESH_INCLUDE_QUADMESH_QUAD_MESH_H_
#define QUADMESH_INCLUDE_QUADMESH_QUAD_MESH_H_

#include <gm/abstract_surface.hpp>
#include <gm/axis.hpp>
#include <gm/point.hpp>

#include <memory>
#include <ostream>
#include <set>
#include <vector>

class QuadMesh;

class QuadMesh : public std::enable_shared_from_this<QuadMesh> {
public:
    struct Node;

    using NodePtr = std::shared_ptr<Node>;
    using NodesType = std::vector<NodePtr>;
    using EdgeSoup = std::vector<std::array<gm::Point, 2>>;
    using VertexSoup = std::vector<gm::Point>;

    NodesType::iterator begin();
    NodesType::const_iterator begin() const;
    NodesType::iterator end();
    NodesType::const_iterator end() const;

    NodesType::reference operator[](size_t i);
    NodesType::const_reference operator[](size_t i) const;
    size_t size() const;

    NodesType::reference append(const gm::Point& v);

    EdgeSoup get_edge_soup() const;
    VertexSoup get_vertex_soup() const;

    void remove(size_t index);
    void remove(NodePtr node);

    size_t drop_obsolete();
    void fix_indices();

    friend std::ostream& operator<<(std::ostream& os, const QuadMesh& obj);

private:
    NodesType nodes_;
};

struct QuadMesh::Node {
    friend class QuadMesh;

    using AdjacentType = std::set<size_t>;

    Node();
    Node(std::shared_ptr<QuadMesh> parent, const gm::Point& v, size_t vindex);

    const gm::Point& v() const;
    size_t vindex() const;
    std::shared_ptr<QuadMesh> parent() const;
    NodesType adjacent_view() const;
    const AdjacentType& adjacent() const;
    size_t count() const;

    friend void set_adjacent(Node& lhs, Node& rhs);

    friend void set_adjacent(std::shared_ptr<Node> lhs,
                             std::shared_ptr<Node> rhs);
    friend void
    set_adjacent(std::shared_ptr<Node> lhs,
                 const std::initializer_list<std::shared_ptr<Node>>& rhs);

    friend bool is_adjacent(const Node& lhs, const Node& rhs);
    friend bool is_adjacent(std::shared_ptr<Node> lhs,
                            std::shared_ptr<Node> rhs);

    friend bool operator==(const Node& lhs, const Node& rhs);
    friend bool operator!=(const Node& lhs, const Node& rhs);

    friend std::ostream& operator<<(std::ostream& os,
                                    const QuadMesh::Node& obj);

private:
    std::weak_ptr<QuadMesh> parent_;
    size_t index_;
    gm::Point vertex_;
    AdjacentType adjacent_;
};

#endif // QUADMESH_INCLUDE_QUADMESH_QUAD_MESH_H_