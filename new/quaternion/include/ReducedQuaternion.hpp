#pragma once

#include "Quaternion.hpp"

using ReducedQuaternion = ColVector<3>;

// Reduced quaternion to full quaternion
template <size_t N>
inline ColVector<N + 1> red2quat(const ColVector<N> &r) {
    auto sq = [](double x) { return x * x; };
    ColVector<N + 1> qresult;
    assignBlock<1, N + 1, 0, 1>(qresult) = r;
    qresult[0] = {sqrt(1.0 - sq(r[0]) - sq(r[1]) - sq(r[2]))};
    return qresult;
}

// Quaternion to reduced quaternion
template <size_t N>
inline ColVector<N - 1> quat2red(const ColVector<N> &q) {
    return getBlock<1, N, 0, 1>(q);
}