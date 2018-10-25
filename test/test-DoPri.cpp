#include <Matrix/Matrix.hpp>
#include <ODE/DormandPrince.hpp>
#include <gtest/gtest.h>
#include <limits>

TEST(DoPri, euler) {
    auto func = [](double t, double x) {
        return x + 0 * t;
    };  // x'(t) = x(t) → x(t) = e^t
    AdaptiveODEOptions opt = {};
    opt.t_start            = 0;
    opt.t_end              = 1;
    opt.epsilon            = 1e-12;
    opt.h_start            = 1e-2;
    opt.h_min              = 1e-6;
    opt.maxiter            = 1e6;

    double x_start = 1;

    auto result = dormandPrince(func, x_start, opt);

    std::cout.precision(std::numeric_limits<double>::max_digits10);
    std::cout << "e = " << result.solution.back() << std::endl;

    ASSERT_LE(fabs((result.solution.back() - M_E)), opt.epsilon);
    ASSERT_EQ(result.resultCode, ODEResultCodes::SUCCESS);
}

TEST(DoPri, eulerVector) {
    using Type = ColVector<3>;
    auto func  = [](double t, Type x) {
        (void) t;
        return x;
    };  // x'(t) = x(t) → x(t) = e^t

    Type x_start = {{
        {1},
        {1},
        {1},
    }};

    AdaptiveODEOptions opt = {};
    opt.t_start            = 0;
    opt.t_end              = 1;
    opt.epsilon            = 1e-12;
    opt.h_start            = 1e-2;
    opt.h_min              = 1e-6;
    opt.maxiter            = 1e6;

    auto result = dormandPrince(func, x_start, opt);

    std::cout.precision(std::numeric_limits<double>::max_digits10);
    std::cout << "e = " << result.solution.back()[0][0] << std::endl;

    double error = norm(result.solution.back() - M_E * x_start);

    ASSERT_LE(error, opt.epsilon);
    ASSERT_EQ(result.resultCode, ODEResultCodes::SUCCESS);
}