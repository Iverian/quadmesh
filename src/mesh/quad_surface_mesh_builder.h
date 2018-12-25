#ifndef QUADMESH_SRC_QUAD_SURFACE_MESH_BUILDER_H_
#define QUADMESH_SRC_QUAD_SURFACE_MESH_BUILDER_H_

#include <geom_model/plane.h>
#include <quadmesh/quad_mesh.h>
#include <util/itertools.h>

#include <spdlog/logger.h>

#include <map>
#include <memory>
#include <optional>
#include <vector>

class QuadmeshConfig;
class QuadMeshBuilder;

class QuadSurfaceMeshBuilder {
public:
    friend class QuadMeshBuilder;

    enum class BoundaryType : int { OUTER = -1, INNER = 1 };
    enum class NodeType : int {
        NIL = -1,
        END,
        END_SIDE,
        SIDE,
        SIDE_CORNER,
        CORNER,
        CORNER_REVERSAL,
        REVERSAL
    };

    struct PavingBoundaryNode {

        PavingBoundaryNode();
        explicit PavingBoundaryNode(QuadMesh::NodePtr ptr, size_t p_adjcnt = 0,
                                    bool p_external = false);

        PavingBoundaryNode(const PavingBoundaryNode&) = default;
        PavingBoundaryNode(PavingBoundaryNode&&) noexcept = default;
        PavingBoundaryNode& operator=(const PavingBoundaryNode&) = default;
        PavingBoundaryNode& operator=(PavingBoundaryNode&&) noexcept = default;

        PavingBoundaryNode& set_iangle(double a);
        PavingBoundaryNode& set_type(NodeType t);

        size_t vindex;
        double iangle;
        NodeType type;
        bool external;
        size_t adjcnt;
    };

    struct PavingBoundary : std::list<PavingBoundaryNode> {
        BoundaryType type;
    };

    using PavingIter = PavingBoundary::iterator;
    using PavingCycle = Cycle<PavingIter>;
    using PavingRow = std::array<PavingIter, 2>;

    QuadSurfaceMeshBuilder(QuadMeshBuilder& parent,
                           std::shared_ptr<spdlog::logger> log);

    QuadSurfaceMeshBuilder&
    set_surface(std::shared_ptr<const AbstractSurface> surface,
                bool same_sence);
    QuadSurfaceMeshBuilder&
    set_bounds(std::vector<std::vector<QuadMesh::NodePtr>>&& bounds);

    QuadSurfaceMeshBuilder& get();

    std::shared_ptr<const AbstractSurface> surf() const;
    const QuadmeshConfig& conf() const;
    std::shared_ptr<QuadMesh> mesh() const;
    QuadMesh::NodePtr append(const Point& p) const;
    PavingBoundaryNode append_b(const Point& p) const;

    void init_boundary(PavingBoundary& b,
                       const std::vector<QuadMesh::NodePtr>& nodes,
                       BoundaryType type);

    double set_iangle(const PavingCycle& i, BoundaryType type);
    PavingBoundaryNode& set_node_type(PavingBoundaryNode& node);
    PavingBoundaryNode& resolve_ambiguity(PavingBoundaryNode& node);

    PavingCycle cycle(PavingBoundary& b) const;
    std::tuple<Point, Point, Point, Vec>
    get_triple(const PavingCycle& i) const;
    Plane tangent_at(const Point& p) const;
    Vec normal_at(const Point& p) const;
    QuadMesh::NodePtr at(size_t index) const;
    QuadMesh::NodePtr at(const PavingBoundaryNode& node) const;

    std::pair<PavingIter, bool> choose_row(PavingBoundary& b,
                                           PavingIter first);
    std::optional<PavingIter> primitive_classification(PavingBoundary& b);

    PavingIter generate_row(PavingBoundary& b, PavingIter first);

    PavingBoundaryNode at_bisect(const PavingCycle& i) const;
    std::array<PavingBoundaryNode, 3> at_trisect(const PavingCycle& i) const;
    std::array<PavingBoundaryNode, 5> at_pentasect(const PavingCycle& i) const;

    void set_local_adjacent(PavingBoundaryNode& lhs, PavingBoundaryNode& rhs);
    void
    set_local_adjacent(PavingBoundaryNode& lhs,
                       const std::initializer_list<
                           std::reference_wrapper<PavingBoundaryNode>>& ilist);
    void flush();

private:
    QuadMeshBuilder* parent_;
    std::shared_ptr<const AbstractSurface> surf_;
    bool same_sence_;

    std::list<PavingBoundary> bounds_;
    std::shared_ptr<spdlog::logger> log_;

    std::vector<std::pair<size_t, size_t>> local_;
};

#endif // QUADMESH_SRC_QUAD_SURFACE_MESH_BUILDER_H_