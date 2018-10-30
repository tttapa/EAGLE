#pragma once

#include <Matrix/Matrix.hpp>

namespace MotorControlTransformation {
constexpr Matrix<4, 4> M     = {{
    {1, 1, 1, -1},
    {1, 1, -1, 1},
    {1, -1, 1, 1},
    {1, -1, -1, -1},
}};
constexpr Matrix<4, 4> M_inv = 0.25 * transpose(M);
};  // namespace MotorControlTransformation

inline ColVector<4>
convertControlSignalToMotorOutputs(const ColVector<4> &u_model) {
    ColVector<4> u_motors = MotorControlTransformation::M * u_model;
    return u_motors;
}

inline ColVector<4>
convertMotorOutputsToControlSignal(const ColVector<4> &u_motors) {
    ColVector<4> u_model = MotorControlTransformation::M_inv * u_motors;
    return u_model;
}

template <class T>
T clamp(T v, T lo, T hi) {
    return (v < lo) ? lo : ((hi < v) ? hi : v);
}

inline ColVector<4> clampMotorControlSignal(const ColVector<4> &u_model_raw) {
    ColVector<4> u_motors = convertControlSignalToMotorOutputs(u_model_raw);
    for (auto &row : u_motors)
        row[0] = clamp(row[0], 0.0, 1.0);
    return convertMotorOutputsToControlSignal(u_motors);
}

inline ColVector<3> clampMotorControlSignal(const ColVector<3> &u_model_raw,
                                            double u_c) {
    return getBlock<1, 4, 0, 1>(
        clampMotorControlSignal(vcat(u_c * eye<1>(), u_model_raw)));
}