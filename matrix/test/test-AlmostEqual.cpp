#include <AlmostEqual.hpp>
#include <gtest/gtest.h>
#include <Matrix.hpp>

TEST(AlmostEqual, double) {
    double l = 1;
    double r = 1 + 1e-10;
    ASSERT_TRUE(isAlmostEqual(l, r, 1e-9));
    ASSERT_FALSE(isAlmostEqual(l, r, 1e-10));
}

TEST(AlmostEqual, matrix) {
    Matrix<2, 2> l = {{
        {1, 1},
        {1, 1},
    }};
    Matrix<2, 2> r = {{
        {1, 1},
        {1, 1 + 1e-10},
    }};
    ASSERT_TRUE(isAlmostEqual(l, r, 1e-9));
    ASSERT_FALSE(isAlmostEqual(l, r, 1e-10));
}