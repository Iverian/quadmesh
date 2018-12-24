#define _USE_MATH_DEFINES

#include <gtest/gtest.h>
#include <memory>

#include <geom_model/curves.h>
#include <geom_model/shell.h>
#include <geom_model/surfaces.h>
#include <quadmesh/quadmesh_config.h>

#include <cmath>
#include <mesh/quad_mesh_builder.h>
#include <util/debug.h>
#include <util/itertools.h>
#include <util/util.h>

using namespace std;

struct Data {
    Data()
        : s()
    {
        array<Axis, 3> ax = {Axis::from_zx({0, 0, 1}, {1, 0, 0}, {0, 0, 0}),
                             Axis::from_zx({0, 0, -1}, {0, -1, 0}, {0, 0, 0}),
                             Axis::from_zx({-1, 0, 0}, {0, -1, 0}, {0, 0, 0})};
        array<Edge, 3> e
            = {Edge(make_shared<Circle>(1, ax[1]), M_PI, 2 * M_PI),
               Edge(make_shared<Circle>(1, ax[2]), 0, M_PI),
               Edge(make_shared<Line>(Vec(0, 1, 0), Point(0, 0, 0)), -1, 1)};
        array<shared_ptr<AbstractSurface>, 3> g
            = {make_shared<Plane>(ax[1]), make_shared<Plane>(ax[2]),
               make_shared<SphericalSurface>(1, ax[0])};
        array<Face, 3> f
            = {Face(g[0], true,
                    {OrientedEdge(e[0], true), OrientedEdge(e[2], true)}),
               Face(g[1], true,
                    {OrientedEdge(e[1], true), OrientedEdge(e[2], false)}),
               Face(g[2], true,
                    {OrientedEdge(e[0], false), OrientedEdge(e[1], false)})};
        s = Shell(ax[0], {f[0], f[1], f[2]});
    }

    Shell s;
};

class TestSurfaceBuilder : public ::testing::Test {
protected:
    void SetUp()
    {
        data = make_unique<Data>();
    }
    void TearDown()
    {
        data.reset();
    }
    std::unique_ptr<Data> data;
};

TEST_F(TestSurfaceBuilder, test_1)
{
    auto builder = QuadMeshBuilder(data->s, QuadmeshConfig());
    auto mesh = builder.get().mesh();
    cout << *mesh << endl;

    ofstream of("sphere_soup.json");
    auto e = mesh->get_edge_soup();
    auto v = mesh->get_vertex_soup();

    fmt::print(of, "{{ \"vertices\": {}, \"edges\": {} }}",
               RangePrint(begin(v), end(v)), RangePrint(begin(e), end(e)));

    SUCCEED();
}

TEST_F(TestSurfaceBuilder, test_2)
{
    auto f = data->s.faces()[0];
    auto& s = f.surface();
    DEBUG_FMT_("n : {}, t: {}", s.normal({0, 0}), s.tangent({1, 1}));
}