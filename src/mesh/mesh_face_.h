// #ifndef QUADMESH_SRC_MESH_MESH_FACE_H_
// #define QUADMESH_SRC_MESH_MESH_FACE_H_

// #include <geom_model/abstract_surface.h>
// #include <quadmesh/quad_surface_mesh.h>
// #include <quadmesh/quadmesh_config.h>
// #include <util/itertools.h>

// #include <list>
// #include <optional>
// #include <vector>

// #include <spdlog/async.h>

// class MeshFace {
// public:
//     enum class BoundaryType : int { OUTER = -1, INNER = 1 };
//     enum class NodeType : int {
//         NIL = -1,
//         END,
//         END_SIDE,
//         SIDE,
//         SIDE_CORNER,
//         CORNER,
//         CORNER_REVERSAL,
//         REVERSAL
//     };
//     struct PavingBoundaryNode {
//         PavingBoundaryNode();
//         PavingBoundaryNode(const ParametricPoint& p);
//         PavingBoundaryNode& push_quad(QuadSurfaceMeshNode& q);

//         ParametricPoint point;
//         double internal_angle;
//         NodeType type;
//         std::array<QuadSurfaceMeshNode*, 4> quad;
//     };
//     struct PavingBoundary {
//         using NodesType = std::list<PavingBoundaryNode>;

//         BoundaryType type;
//         NodesType nodes;
//     };
//     using PavingIter = PavingBoundary::NodesType::iterator;
//     using PavingCycle = Cycle<PavingIter>;
//     using PavingRow = std::array<PavingIter, 2>;

//     MeshFace(const AbstractSurface& surface,
//              const std::vector<std::vector<ParametricPoint>>& boundaries,
//              const QuadmeshConfig& config);

//     const QuadSurfaceMesh& get();

// private:
//     const AbstractSurface& s() const;

//     void make_initial_boundary(
//         const std::vector<std::vector<ParametricPoint>>& init);
//     void make_loop(PavingBoundary& b, std::vector<ParametricPoint> loop,
//                    BoundaryType type);
//     void get_angles_for_boundary(PavingBoundary& b);

//     double get_internal_angle(const PavingCycle& it, BoundaryType type)
//     const; NodeType get_node_type(const PavingBoundaryNode& node) const;
//     NodeType resolve_ambiguity(NodeType type) const;

//     PavingRow choose_row(PavingBoundary& b);
//     std::optional<PavingRow> select_primitive(PavingBoundary& b);
//     void generate_variants(PavingIter cur, PavingIter end,
//                            std::vector<std::vector<NodeType>>& result);
//     void generate_row(PavingBoundary& b, PavingRow limits);

//     std::vector<Point> discretize(const OrientedEdge& edge);
//     PavingCycle get_cycle(PavingBoundary& b);
//     std::tuple<Point, Point, Point> get_triple(const PavingCycle& it) const;

//     PavingBoundaryNode at_bisect(const PavingCycle& it);
//     std::array<PavingBoundaryNode, 3> at_trisect(const PavingCycle& it);
//     std::array<PavingBoundaryNode, 5> at_pentasect(const PavingCycle& it);

//     QuadSurfaceMeshNode& append_quad(PavingBoundaryNode& p0,
//                                      PavingBoundaryNode& p1,
//                                      PavingBoundaryNode& p2,
//                                      PavingBoundaryNode& p3);
//     QuadSurfaceMeshNode&
//     set_adjacent(QuadSurfaceMeshNode& q,
//                  std::initializer_list<PavingBoundaryNode> nodes);
//     void insert_nodes(PavingBoundary& b, PavingIter pos,
//                       std::initializer_list<PavingBoundaryNode> nodes);

//     std::shared_ptr<const AbstractSurface> surface_;
//     std::list<PavingBoundary> boundaries_;
//     QuadmeshConfig config_;
//     std::shared_ptr<spdlog::logger> logger_;

//     QuadSurfaceMesh result_;
// };

// std::ostream& operator<<(std::ostream& os, const
// MeshFace::PavingBoundaryNode& p);

// #endif // QUADMESH_SRC_MESH_MESH_FACE_H_