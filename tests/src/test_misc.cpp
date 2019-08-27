#define _USE_MATH_DEFINES

#include <gtest/gtest.h>

#include <gm/vec.hpp>

#include <cmath>

using namespace std;

TEST(TestMisc, test_1)
{
    auto a = gm::Vec(0.108243, -0.273392, 0);
    auto b = gm::Vec(-0.108243, 0.273392, 0);
    auto ab = angle(a, b);
    ASSERT_FALSE(isnan(ab));
}
