#pragma once

#include "Quaternion.hpp"

using ReducedQuaternion = ColVector<3>;

// Reduced quaternion to full quaternion
/*
inline Quaternion red2quat(const ReducedQuaternion &r) {
    return {{
        {sqrt(1 - r[0][0] * r[0][0] - r[1][0] * r[1][0] - r[2][0] * r[2][0])},
        {r[0][0]},
        {r[1][0]},
        {r[2][0]},
    }};
}
*/

template <size_t N>
inline ColVector<N + 1> red2quat(const ColVector<N> &r) {
    ColVector<N+1> qresult;
    assignBlock<1, N+1, 0, 1>(qresult) = r;
    qresult[0][0] = sqrt(1 - r[0][0] * r[0][0] - r[1][0] * r[1][0] - r[2][0] * r[2][0]);
    return qresult;
}

// Quaternion to reduced quaternion
inline ReducedQuaternion quat2red(const Quaternion &q) {
    return getBlock<1, 4, 0, 1>(q);
}