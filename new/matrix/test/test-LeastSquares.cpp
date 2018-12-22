#include <gtest/gtest.h>

#include <AlmostEqual.hpp>
#include <LeastSquares.hpp>

using Matrices::T;
using std::cout;
using std::endl;

TEST(LeastSquares, solveLeastSquaresSquare) {
    Matrix<3, 3> A = {{
        {1, 1, 1},
        {0, 2, 5},
        {2, 5, -1},
    }};
    ColVector<3> B = {{
        {6},
        {-4},
        {27},
    }};

    ColVector<3> X = solveLeastSquares(A, B);

    ColVector<3> X_expected = {{
        {5},
        {3},
        {-2},
    }};

    ASSERT_TRUE(isAlmostEqual(X, X_expected, 1e-12));
}