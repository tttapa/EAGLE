#include <ODE/ODEEval.hpp>
#include <gtest/gtest.h>

using std::vector;

TEST(ODEEval, interpolate) {
    double t        = 4;
    double t1       = 2;
    double t2       = 5;
    double x1       = 3;
    double x2       = 9;
    double x        = interpolate(t1, t2, x1, x2, t);
    double expected = 7;
    ASSERT_EQ(x, expected);
}

TEST(ODEEval, sampleODEResult) {
    vector<double> t          = {0, 2, 4, 6};
    vector<double> x          = {0, 1, -1, -1};
    ODEResultX<double> oderes = {t, x};
    vector<double> sampled    = sampleODEResult(oderes, 0, 1, 6);
    vector<double> expected   = {0, 0.5, 1, 0, -1, -1, -1};
    ASSERT_EQ(sampled, expected);
}

TEST(ODEEval, sampleODEResultJustBeforeEnd) {
    vector<double> t          = {0, 2, 4, 6};
    vector<double> x          = {0, 1, -1, -1};
    ODEResultX<double> oderes = {t, x};
    vector<double> sampled    = sampleODEResult(oderes, 0, 1, 5.999);
    vector<double> expected   = {0, 0.5, 1, 0, -1, -1};
    ASSERT_EQ(sampled, expected);
}

TEST(ODEEval, sampleODEResultJustAfterEnd) {
    vector<double> t          = {0, 2, 4, 6};
    vector<double> x          = {0, 1, -1, -1};
    ODEResultX<double> oderes = {t, x};
    vector<double> sampled    = sampleODEResult(oderes, 0, 1, 6.001);
    vector<double> expected   = {0, 0.5, 1, 0, -1, -1, -1};
    ASSERT_EQ(sampled, expected);
}

TEST(ODEEval, sampleODEResultOutOfBounds) {
    vector<double> t          = {0, 2, 4, 6};
    vector<double> x          = {0, 1, -1, -1};
    ODEResultX<double> oderes = {t, x};
    vector<double> sampled    = sampleODEResult(oderes, 0, 1, 8);
    // samples after t = 6 are invalid
    vector<double> expected   = {0, 0.5, 1, 0, -1, -1, -1};
    ASSERT_EQ(sampled, expected);
}