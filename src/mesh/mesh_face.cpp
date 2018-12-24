// #define _USE_MATH_DEFINES

// #include "mesh_face.h"

// #include <geom_model/plane.h>
// #include <geom_model/vec.h>
// #include <quadmesh/quadmesh.h>

// #include <util/debug.h>
// #include <util/itertools.h>
// #include <util/math.h>

// #include <fmt/ostream.h>
// #include <spdlog/spdlog.h>

// #include <cmath>

// using namespace std;

// MeshFace::MeshFace(const AbstractSurface& surface,
//                    const std::vector<std::vector<ParametricPoint>>&
//                    boundaries, const QuadmeshConfig& config)
//     : surface_(surface.shared_from_this())
//     , boundaries_()
//     , config_(config)
//     , logger_(spdlog::get("logger"))
//     , result_(surface_)
// {
// }

// const QuadSurfaceMesh& MeshFace::get()
// {
//     return result_;
// }

// MeshFace::PavingRow MeshFace::choose_row(PavingBoundary& b)
// {
//     PavingRow result;

//     if (auto p = select_primitive(b); p.has_value()) {
//         result = p.value();
//     } else {
//         size_t pos = 0;
//         PavingIter min = begin(b.nodes);
//         vector<PavingIter> corners_and_reversals;

//         for (auto i = begin(b.nodes); pos < result.size() && i !=
//         end(b.nodes);
//              ++i) {
//             if (i->internal_angle < min->internal_angle) {
//                 min = i;
//             }
//             switch (i->type = resolve_ambiguity(get_node_type(*i))) {
//             case NodeType::END:
//                 result[pos++] = i;
//                 break;
//             case NodeType::CORNER:
//             case NodeType::REVERSAL:
//                 corners_and_reversals.push_back(i);
//                 break;
//             default:
//                 break;
//             }
//         }
//         switch (pos) {
//         case 0:
//             if (!corners_and_reversals.empty()) {
//                 min = *min_element(
//                     begin(corners_and_reversals),
//                     end(corners_and_reversals),
//                     [](const auto& lhs, const auto& rhs) {
//                         return lhs->internal_angle < rhs->internal_angle;
//                     });
//             }
//             result = {min, min};
//             break;
//         case 1:
//             result[1] = result[0];
//             break;
//         default:
//             break;
//         }
//     }
//     return result;
// }

// optional<MeshFace::PavingRow> MeshFace::select_primitive(PavingBoundary& b)
// {
//     optional<PavingRow> result = nullopt;
//     if (b.type == BoundaryType::OUTER) {
//         for (auto i = begin(b.nodes); i != end(b.nodes); ++i) {
//         }
//     }
//     return result;
// }

// void MeshFace::generate_variants(PavingIter cur, PavingIter end,
//                                  vector<vector<NodeType>>& result)
// {
//     if (cur != end) {
//         auto type = get_node_type(*cur);
//         switch (type) {
//         case NodeType::END:
//         case NodeType::SIDE:
//         case NodeType::CORNER:
//         case NodeType::REVERSAL:
//             for (auto& i : result) {
//                 i.push_back(type);
//             }
//             break;
//         case NodeType::END_SIDE:
//         case NodeType::SIDE_CORNER:
//         case NodeType::CORNER_REVERSAL:
//             break;
//         case NodeType::NIL:
//             break;
//         }
//     }
// }

// void MeshFace::generate_row(PavingBoundary& b, PavingRow limits)
// {
//     auto flag = true;
//     auto c = ++get_cycle(b).set_iter(limits[0]);
//     while (flag && c.iter() != limits[1]) {
//         auto& ncur = *c;
//         auto& nprev = *prev(c);
//         auto& nprev2 = *prev(c, 2);

//         switch (ncur.type) {
//         case NodeType::END: {
//             flag = false;
//             break;
//         }
//         case NodeType::SIDE: {
//             auto p = at_bisect(c);
//             if (nprev.type == NodeType::SIDE) {
//                 auto pp = at_bisect(prev(c));
//                 auto& q = append_quad(nprev, pp, p, ncur);
//                 set_adjacent(q, {nprev});
//                 logger_->info("s \"first\": {} \"second\": {}", nprev, pp);
//                 logger_->info("s \"first\": {} \"second\": {}", ncur, p);
//                 nprev = pp;
//                 ncur = p;
//             } else {
//                 auto& q = append_quad(nprev2, nprev, ncur, p);
//                 // TODO: только ли из этой ноды добавлять?
//                 set_adjacent(q, {nprev});
//                 logger_->info("s \"first\": {} \"second\": {}", nprev, p);
//                 nprev = p;
//             }
//             break;
//         }
//         case NodeType::CORNER: {
//             auto p = at_trisect(c);
//             auto& q0 = append_quad(nprev2, nprev2, ncur, p[0]);
//             auto& q1 = append_quad(p[0], ncur, p[2], p[2]);
//             set_adjacent(q0, {nprev});
//             q1.set_adjacent(q0);
//             logger_->info("s \"first\": {} \"second\": {}", nprev, p[0]);
//             nprev = p[0];
//             b.nodes.insert(c.iter(), {p[1], p[2]});
//             break;
//         }
//         case NodeType::REVERSAL: {
//             auto p = at_pentasect(c);
//             auto& q0 = append_quad(nprev2, nprev, ncur, p[0]);
//             auto& q1 = append_quad(p[0], ncur, p[2], p[1]);
//             auto& q2 = append_quad(p[2], ncur, p[4], p[3]);
//             set_adjacent(q0, {nprev});
//             q1.set_adjacent(q0);
//             q1.set_adjacent(q2);
//             q2.set_adjacent(q1);
//             logger_->info("s \"first\": {} \"second\": {}", nprev, p[0]);
//             nprev = p[0];
//             b.nodes.insert(c.iter(), {p[1], p[2], p[3], p[4]});
//             break;
//         }
//         case NodeType::NIL:
//         case NodeType::END_SIDE:
//         case NodeType::SIDE_CORNER:
//         case NodeType::CORNER_REVERSAL: {
//             logger_->critical("unresolved ambiguity");
//             // THROW_(runtime_error, "Неразрешенная не");
//             break;
//         }
//         }
//     }
// }

// // TODO проверить, что внешняя граница направлена против часовой стрелки, а
// все
// // внутренние -- по
// void MeshFace::make_initial_boundary(
//     const std::vector<std::vector<ParametricPoint>>& init)
// {
//     boundaries_.resize(init.size());
//     auto i = begin(boundaries_);
//     auto j = begin(init);

//     make_loop(*i++, *j++, BoundaryType::OUTER);
//     for (; j != end(init); ++i, ++j) {
//         make_loop(*i, *j, BoundaryType::INNER);
//     }
// }

// void MeshFace::make_loop(PavingBoundary& b, std::vector<ParametricPoint>
// loop,
//                          BoundaryType type)
// {
//     b.type = type;
//     for (auto& i : loop) {
//         b.nodes.emplace_back(i);
//     }
//     get_angles_for_boundary(b);
// }

// void MeshFace::get_angles_for_boundary(PavingBoundary& b)
// {
//     auto c = get_cycle(b);
//     do {
//         c->internal_angle = get_internal_angle(c, b.type);
//         ++c;
//     } while (c.iter() != c.first());
// }

// double MeshFace::get_internal_angle(const PavingCycle& it,
//                                     BoundaryType type) const
// {
//     auto [p, c, n] = get_triple(it);
//     auto cp = Vec(c, p), cn = Vec(c, n);
//     auto surface_norm = s().normal(it->point);

//     auto result = angle(cp, cn);
//     if (int(type) * dot(cross(cp, cn), surface_norm) < 0) {
//         result = 2 * M_PI - result;
//     }
//     return result;
// }

// #define POW_M1(n) (((n) % 2) ? (-1) : (1))

// MeshFace::NodeType
// MeshFace::get_node_type(const PavingBoundaryNode& node) const
// {
//     auto result = NodeType::REVERSAL;
//     auto& angtol = config_.get_angle_tolerances();
//     for (size_t i = 0; i < angtol.size(); ++i) {
//         auto limit = (i + 1) * M_PI_2 + POW_M1(i) * angtol[i];
//         if (node.internal_angle <= limit) {
//             result = NodeType(i);
//             break;
//         }
//     }
//     return result;
// }

// #undef POW_M1

// MeshFace::NodeType MeshFace::resolve_ambiguity(NodeType type) const
// {
//     switch (type) {
//     case NodeType::END_SIDE:
//     case NodeType::SIDE_CORNER:
//         type = NodeType::SIDE;
//         break;
//     case NodeType::CORNER_REVERSAL:
//         type = NodeType::CORNER;
//         break;
//     case NodeType::NIL:
//     case NodeType::END:
//     case NodeType::SIDE:
//     case NodeType::CORNER:
//     case NodeType::REVERSAL:
//         break;
//     }
//     return type;
// }

// vector<Point> MeshFace::discretize(const OrientedEdge& edge)
// {
//     return edge.discretize();
// }

// MeshFace::PavingCycle MeshFace::get_cycle(PavingBoundary& b)
// {
//     return Cycle(begin(b.nodes), end(b.nodes));
// }

// tuple<Point, Point, Point> MeshFace::get_triple(const PavingCycle& it) const
// {
//     auto tp = s().tangent(it->point);
//     auto prev = tp.gproject(s().f(::prev(it)->point));
//     auto next = tp.gproject(s().f(::next(it)->point));
//     auto cur = tp.ax().center();

//     return make_tuple(prev, cur, next);
// }

// #define TO_SIGN(flag) ((flag) ? (1) : (-1))

// MeshFace::PavingBoundaryNode MeshFace::at_bisect(const PavingCycle& it)
// {
//     auto [p, c, n] = get_triple(it);
//     auto u = TO_SIGN(it->internal_angle < M_PI)
//         * unit(unit({c, p}) + unit({c, n}));
//     auto d = (dist(p, c) + dist(c, n)) / (2 * sin(it->internal_angle / 2));
//     return s().project(c + u * d);
// }

// array<MeshFace::PavingBoundaryNode, 3>
// MeshFace::at_trisect(const PavingCycle& it)
// {
//     auto [p, c, n] = get_triple(it);
//     auto ax = Axis::from_abc(p, c, n);
//     auto angle = TO_SIGN(it->internal_angle < M_PI) * it->internal_angle /
//     3;

//     auto u0 = ax.rotate_z(angle, ax.x());
//     auto u2 = ax.rotate_z(angle, u0);
//     auto u1 = unit(u0 + u2);
//     auto d = (dist(p, c) + dist(c, n)) / (2 * sin(it->internal_angle / 3));

//     return {s().project(c + d * u0), s().project(c + sqrt(2) * d * u1),
//             s().project(c + d * u2)};
// }

// std::array<MeshFace::PavingBoundaryNode, 5>
// MeshFace::at_pentasect(const PavingCycle& it)
// {
//     auto [p, c, n] = get_triple(it);
//     auto ax = Axis::from_abc(p, c, n);
//     auto cp = unit({c, p}), cn = unit({c, n});

//     auto u2 = TO_SIGN(it->internal_angle < M_PI) * unit(cp + cn);
//     auto u0 = unit(cp + u2);
//     auto u4 = unit(cn + u2);
//     auto u1 = unit(u0 + u2);
//     auto u3 = unit(u2 + u4);
//     auto d = (dist(p, c) + dist(c, n)) / (2 * sin(it->internal_angle / 4));

//     return {s().project(c + d * u0), s().project(c + sqrt(2) * d * u1),
//             s().project(c + d * u2), s().project(c + d * u3),
//             s().project(c + d * u4)};
// }

// #undef TO_SIGN

// QuadSurfaceMeshNode& MeshFace::append_quad(PavingBoundaryNode& p0,
//                                            PavingBoundaryNode& p1,
//                                            PavingBoundaryNode& p2,
//                                            PavingBoundaryNode& p3)
// {
//     auto& q = result_.append({p0.point, p1.point, p2.point, p3.point});
//     logger_->info("+ \"quad\": {}", q);
//     p0.push_quad(q);
//     p1.push_quad(q);
//     p2.push_quad(q);
//     p3.push_quad(q);
//     return q;
// }

// QuadSurfaceMeshNode&
// MeshFace::set_adjacent(QuadSurfaceMeshNode& q,
//                        std::initializer_list<PavingBoundaryNode> nodes)
// {
//     for (auto& i : nodes) {
//         for (auto j : i.quad) {
//             if (j == nullptr) {
//                 break;
//             }
//             q.set_adjacent(*j);
//         }
//     }
//     return q;
// }

// void MeshFace::insert_nodes(PavingBoundary& b, PavingIter pos,
//                             std::initializer_list<PavingBoundaryNode> nodes)
// {
//     b.nodes.insert(pos, nodes);
//     logger_->info("+ \"paving_node\": {}",
//                   RangePrint(begin(nodes), end(nodes)));
// }

// const AbstractSurface& MeshFace::s() const
// {
//     return *surface_;
// }

// MeshFace::PavingBoundaryNode::PavingBoundaryNode()
//     : point()
//     , internal_angle(0)
//     , quad{nullptr, nullptr, nullptr, nullptr}
// {
// }

// MeshFace::PavingBoundaryNode::PavingBoundaryNode(const ParametricPoint& p)
//     : point(p)
//     , internal_angle(0)
//     , type(NodeType::NIL)
//     , quad{nullptr, nullptr, nullptr, nullptr}
// {
// }

// MeshFace::PavingBoundaryNode&
// MeshFace::PavingBoundaryNode::push_quad(QuadSurfaceMeshNode& q)
// {
//     for (auto& i : quad) {
//         if (i == nullptr) {
//             i = &q;
//             break;
//         }
//     }
//     return *this;
// }

// std::ostream& operator<<(std::ostream& os,
//                          const MeshFace::PavingBoundaryNode& p)
// {
//     vector<QuadSurfaceMeshNode> q;
//     for (auto& i : p.quad) {
//         if (i == nullptr) {
//             break;
//         }
//         q.push_back(*i);
//     }

//     fmt::print(os, "{\"point\": {}, \"internal_angle\": {}, \"quads\": {}}",
//                p.point, p.internal_angle, RangePrint(begin(q), end(q)));
//     return os;
// }