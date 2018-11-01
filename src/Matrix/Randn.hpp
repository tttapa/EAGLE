#pragma once

#include "Matrix.hpp"
#include <random>

template <size_t N>
ColVector<N> randn(const double (&stddev)[N]) {
    ColVector<N> result = {};
    static std::default_random_engine rgen;
    for (size_t i = 0; i < N; ++i) {
        std::normal_distribution<double> distribution(0, stddev[i]);
        result[i][0] = distribution(rgen);
    }
    return result;
}