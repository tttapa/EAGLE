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
    const double &q0 = q[0][0];
    const double &q1 = q[1][0];
    const double &q2 = q[2][0];
    const double &q3 = q[3][0];

    const double phi =
        atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    const double theta = asin(2.0 * (q0 * q2 - q3 * q1));
    const double psi =
        atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

    return {psi, theta, phi};
}

static inline double sq(double r) { return r * r; }

template <size_t C>
inline Matrix<3, C> quatrotate(const Quaternion &q, const Matrix<3, C> &v) {
    const double &q0 = q[0][0];
    const double &q1 = q[1][0];
    const double &q2 = q[2][0];
    const double &q3 = q[3][0];
    Matrix<3, 3> M   = {{
        {
            1 - 2 * sq(q2) - 2 * sq(q3),
            2 * (q1 * q2 + q0 * q3),
            2 * (q1 * q3 - q0 * q2),
        },
        {
            2 * (q1 * q2 - q0 * q3),
            1 - 2 * sq(q1) - 2 * sq(q3),
            2 * (q2 * q3 + q0 * q1),
        },
        {
            2 * (q1 * q3 + q0 * q2),
            2 * (q2 * q3 - q0 * q1),
            1 - 2 * sq(q1) - 2 * sq(q2),
        },
    }};
    return M * v;
}