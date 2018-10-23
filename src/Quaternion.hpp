#pragma once

#include "Matrix.hpp"

using Quaternion = ColVector<double, 4>;

inline Quaternion quatmultiply(const Quaternion &q, const Quaternion &r) {
    return {{
        {r[0][0] * q[0][0] - r[1][0] * q[1][0] - r[2][0] * q[2][0] - r[3][0] * q[3][0]},
        {r[0][0] * q[1][0] + r[1][0] * q[0][0] - r[2][0] * q[3][0] + r[3][0] * q[2][0]},
        {r[0][0] * q[2][0] + r[1][0] * q[3][0] + r[2][0] * q[0][0] - r[3][0] * q[1][0]},
        {r[0][0] * q[3][0] - r[1][0] * q[2][0] + r[2][0] * q[1][0] + r[3][0] * q[0][0]},
    }};
}