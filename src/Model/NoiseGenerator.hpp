#pragma once

#include <Matrix/Matrix.hpp>

template <size_t N>
struct NoiseGenerator {
    virtual ColVector<N> operator()(double t, const ColVector<N> &v) = 0;
};