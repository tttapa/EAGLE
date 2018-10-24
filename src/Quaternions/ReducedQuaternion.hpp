#pragma once

#include "Quaternion.hpp"

using ReducedQuaternion = ColVector<double, 3>;

// Reduced quaternion to full quaternion
inline Quaternion red2quat(const ReducedQuaternion &r) {
    return {{
        {sqrt(1 - r[0][0] * r[0][0] - r[1][0] * r[1][0] - r[2][0] * r[2][0])},
        {r[0][0]},
        {r[1][0]},
        {r[2][0]},
    }};
}

// Quaternion to reduced quaternion
inline ReducedQuaternion quat2red(const Quaternion &q) {
    return getBlock<1, 4, 0, 1>(q);
}