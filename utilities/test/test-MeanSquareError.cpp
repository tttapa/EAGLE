#include <gtest/gtest.h>

#include <MeanSquareError.hpp>
#include <vector>

TEST(squareError, squareError) {
    std::vector<double> x = {1, 2, 3};
    std::vector<double> y = {4, -5, 1};
    double expected = (1 - 4) * (1 - 4) + (2 + 5) * (2 + 5) + (3 - 1) * (3 - 1);
    double result = squareError(x.begin(), x.end(), y.begin());
    ASSERT_EQ(result, expected);
}