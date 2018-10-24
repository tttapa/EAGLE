#pragma once

#include "Matrix.hpp"

using Quaternion = ColVector<double, 4>;

inline Quaternion quatmultiply(const Quaternion &q, const Quaternion &r) {
    return {{
        {r[0][0] * q[0][0] - r[1][0] * q[1][0] - r[2][0] * q[2][0] -
         r[3][0] * q[3][0]},
        {r[0][0] * q[1][0] + r[1][0] * q[0][0] - r[2][0] * q[3][0] +
         r[3][0] * q[2][0]},
        {r[0][0] * q[2][0] + r[1][0] * q[3][0] + r[2][0] * q[0][0] -
         r[3][0] * q[1][0]},
        {r[0][0] * q[3][0] - r[1][0] * q[2][0] + r[2][0] * q[1][0] +
         r[3][0] * q[0][0]},
    }};
}

inline Quaternion quatconjugate(const Quaternion &q) {
    return {{
        {q[0][0]},
        {-q[1][0]},
        {-q[2][0]},
        {-q[3][0]},
    }};
}

inline Quaternion quatDifference(const Quaternion &p, const Quaternion &q) {
    return quatmultiply(p, quatconjugate(q));
}

using EulerAngles = Array<double, 3>;

inline Quaternion eul2quat(const EulerAngles &eulerAngles) {
    double phi   = eulerAngles[2];
    double theta = eulerAngles[1];
    double psi   = eulerAngles[0];

    double a = phi / 2;
    double b = theta / 2;
    double c = psi / 2;

    return {{
        {cos(a) * cos(b) * cos(c) + sin(a) * sin(b) * sin(c)},
        {sin(a) * cos(b) * cos(c) - cos(a) * sin(b) * sin(c)},
        {cos(a) * sin(b) * cos(c) + sin(a) * cos(b) * sin(c)},
        {cos(a) * cos(b) * sin(c) - sin(a) * sin(b) * cos(c)},
    }};
}

inline EulerAngles quat2eul(const Quaternion &q) {
    double q0 = q[0][0];
    double q1 = q[1][0];
    double q2 = q[2][0];
    double q3 = q[3][0];

    double phi =
        atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    double theta = asin(2.0 * (q0 * q2 - q3 * q1));
    double psi =
        atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

    return {psi, theta, phi};
}

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