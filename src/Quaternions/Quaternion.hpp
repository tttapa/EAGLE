#pragma once

#include <Matrix/Matrix.hpp>

using Quaternion = ColVector<4>;

constexpr Quaternion quatmultiply(const Quaternion &q, const Quaternion &r) {
    return {{
        {r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3]},
        {r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2]},
        {r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1]},
        {r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0]},
    }};
}

constexpr Quaternion quatconjugate(const Quaternion &q) {
    return {{
        {q[0]},
        {-q[1]},
        {-q[2]},
        {-q[3]},
    }};
}

constexpr Quaternion quatDifference(const Quaternion &p, const Quaternion &q) {
    return quatmultiply(p, quatconjugate(q));
}

using EulerAngles = ColVector<3>;

constexpr Quaternion eul2quat(const EulerAngles &eulerAngles) {
    using std::sin;
    using std::cos;
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

constexpr EulerAngles quat2eul(const Quaternion &q) {
    const double &q0 = q[0];
    const double &q1 = q[1];
    const double &q2 = q[2];
    const double &q3 = q[3];

    const double phi =
        atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
    const double theta = asin(2.0 * (q0 * q2 - q3 * q1));
    const double psi =
        atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

    return {psi, theta, phi};
}

static constexpr double sq(double r) { return r * r; }

template <size_t C>
constexpr Matrix<3, C> quatrotate(const Quaternion &q, const Matrix<3, C> &v) {
    const double &q0 = q[0];
    const double &q1 = q[1];
    const double &q2 = q[2];
    const double &q3 = q[3];
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