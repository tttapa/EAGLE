#include <DormandPrince.hpp>
#include <Matrix.hpp>
#include <gtest/gtest.h>
#include <limits>

TEST(DoPri, euler) {
    auto func = [](double t, double x) {
        return x + 0 * t;
    };  // x'(t) = x(t) → x(t) = e^t
    double t0      = 0;
    double t1      = 1;
    double epsilon = 1e-12;
    double h_start = 1e-2;
    double x_start = 1;
    size_t maxiter = 1e6;

    auto result =
        dormandPrince(func, t0, t1, epsilon, h_start, x_start, maxiter);

    std::cout.precision(std::numeric_limits<double>::max_digits10);
    std::cout << "e = " << result.second.back() << std::endl;

    ASSERT_LE(fabs((result.second.back() - M_E)), epsilon);
}

TEST(DoPri, eulerVector) {
    using Type = ColVector<double, 3>;
    auto func  = [](double t, Type x) {
        return x;
    };  // x'(t) = x(t) → x(t) = e^t
    double t0      = 0;
    double t1      = 1;
    double epsilon = 1e-12;
    double h_start = 1e-2;
    Type x_start   = {{
        {1},
        {1},
        {1},
    }};
    size_t maxiter = 1e6;

    auto result =
        dormandPrince(func, t0, t1, epsilon, h_start, x_start, maxiter);

    std::cout.precision(std::numeric_limits<double>::max_digits10);
    std::cout << "e = " << result.second.back()[0][0] << std::endl;

    double error = norm(result.second.back() - M_E * x_start);

    ASSERT_LE(error, epsilon);
}