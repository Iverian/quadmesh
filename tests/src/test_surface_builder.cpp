#define _USE_MATH_DEFINES

#include <gtest/gtest.h>

#include <fmt/ostream.h>
#include <gm/curves.hpp>
#include <gm/shell.hpp>
#include <gm/surfaces.hpp>
#include <mesh/quad_mesh_builder.h>
#include <quadmesh/quadmesh_config.h>
#include <stp/parse.hpp>
#include <util/debug.h>
#include <util/itertools.h>
#include <util/util.h>

#include <cmath>
#include <fstream>
#include <memory>

using namespace std;

TEST(TestSurfaceBuilder, test_cube)
{
    auto manifold = stp::parse("step/cube.stp");
    auto& shell = manifold[0];

    ofstream sf("out/cube_gm.json");
    sf << shell;
    sf.close();

    auto builder = QuadMeshBuilder(shell, QuadmeshConfig());
    auto mesh = builder.get().mesh();

    auto e = mesh->get_edge_soup();
    auto v = mesh->get_vertex_soup();

    ofstream of("out/cube_es.json");
    fmt::print(of, "{{ \"vertices\": {}, \"edges\": {} }}",
               RangePrint(begin(v), end(v)), RangePrint(begin(e), end(e)));
    of.close();

    SUCCEED();
}

TEST(TestSurfaceBuilder, test_parabcyl)
{
    auto manifold = stp::parse("step/parabcyl.stp");
    auto& shell = manifold[0];

    ofstream sf("out/parabcyl_gm.json");
    sf << shell;
    sf.close();

    auto builder = QuadMeshBuilder(shell, QuadmeshConfig());
    auto mesh = builder.get().mesh();

    auto e = mesh->get_edge_soup();
    auto v = mesh->get_vertex_soup();

    ofstream of("out/parabcyl_es.json");
    fmt::print(of, "{{ \"vertices\": {}, \"edges\": {} }}",
               RangePrint(begin(v), end(v)), RangePrint(begin(e), end(e)));
    of.close();

    SUCCEED();
}

TEST(TestSurfaceBuilder, test_spherecyl)
{
    auto manifold = stp::parse("step/spherecyl.stp");
    auto& shell = manifold[0];

    ofstream("out/spherecyl_gm.json") << shell;

    auto builder = QuadMeshBuilder(shell, QuadmeshConfig());
    auto mesh = builder.get().mesh();
    ofstream("out/spherecyl_ms.json") << *mesh;

    auto e = mesh->get_edge_soup();
    auto v = mesh->get_vertex_soup();
    ofstream of("out/spherecyl_es.json");
    fmt::print(of, "{{ \"vertices\": {}, \"edges\": {} }}",
               RangePrint(begin(v), end(v)), RangePrint(begin(e), end(e)));

    SUCCEED();
}

TEST(TestSurfaceBuilder, test_qcircle)
{
    auto manifold = stp::parse("step/qcircle.stp");
    auto& shell = manifold[0];

    ofstream sf("out/qcircle_gm.json");
    sf << shell;
    sf.close();

    auto builder = QuadMeshBuilder(shell, QuadmeshConfig());
    auto mesh = builder.get().mesh();

    auto e = mesh->get_edge_soup();
    auto v = mesh->get_vertex_soup();

    ofstream of("out/qcircle_es.json");
    fmt::print(of, "{{ \"vertices\": {}, \"edges\": {} }}",
               RangePrint(begin(v), end(v)), RangePrint(begin(e), end(e)));
    of.close();

    SUCCEED();
}