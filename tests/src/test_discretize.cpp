
#include <gtest/gtest.h>

#include <gm/bspline_curve.hpp>
#include <gm/edge.hpp>
#include <gm/point.hpp>
#include <qmsh/config.hpp>

#include <mesh/curve_discretize.hpp>

#include <memory>

TEST(TestDiscretize, bspline_discretize)
{
    qmsh::Config conf;
    CurveDiscretize disc(conf);
    auto c = std::make_shared<gm::BSplineCurve>(gm::BSplineCurve(
        3, {0, 0, 0, 0, 0.5, 1, 1, 1, 1},
        {gm::Point(-10, 40, 10), gm::Point(-10, 10, 10), gm::Point(0, -10, 10),
         gm::Point(10, 10, 10), gm::Point(10, 40, 10)}));
    auto e = gm::Edge(c, 0, 1);
    auto p = disc(e);
    SUCCEED();
}