#include <AlmostEqual.hpp>
#include <LeastSquares.hpp>
#include <gtest/gtest.h>

using Matrices::T;
using std::cout;
using std::endl;

TEST(LeastSquares, solveLeastSquaresSquare) {
    Matrix<double, 3, 3> A = {{
        {1, 1, 1},
        {0, 2, 5},
        {2, 5, -1},
    }};
    ColVector<double, 3> B = {{
        {6},
        {-4},
        {27},
    }};

    ColVector<double, 3> X = solveLeastSquares(A, B);

    ColVector<double, 3> X_expected = {{
        {5},
        {3},
        {-2},
    }};

    ASSERT_TRUE(isAlmostEqual(X, X_expected, 1e-12));
}