#pragma once

#include <Matrix.hpp>

template <size_t N>
struct NoiseGenerator {
	virtual ~NoiseGenerator() = default;
    virtual ColVector<N> operator()(double t, const ColVector<N> &v) = 0;
};
