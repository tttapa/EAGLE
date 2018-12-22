#pragma once

#include "Matrix.hpp"
#include <random>

template <size_t N>
ColVector<N> randn(const Array<double, N> &variance) {
    ColVector<N> result = {};
    static std::default_random_engine rgen;
    for (size_t i = 0; i < N; ++i) {
        std::normal_distribution<double> distribution(0, sqrt(variance[i]));
        result[i] = {distribution(rgen)};
    }
    return result;
}

template <size_t N>
ColVector<N> randn(const ColVector<N> &variance) {
    return randn(transpose(variance)[0]);
}

template <size_t N>
RowVector<N> randn(const RowVector<N> &variance) {
    return transpose(randn(variance[0]));
}