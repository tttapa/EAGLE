#pragma once

#include <Matrix/Matrix.hpp>

using Quaternion = ColVector<4>;

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

using EulerAngles = ColVector<3>;

inline Quaternion eul2quat(const EulerAngles &eulerAngles) {
    double phi   = eulerAngles[2][0];
    double theta = eulerAngles[1][0];
    double psi   = eulerAngles[0][0];

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