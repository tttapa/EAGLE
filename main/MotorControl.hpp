#pragma once

#include <Matrix/Matrix.hpp>
#include <algorithm>
#include <vector>

namespace MotorControlTransformation {
constexpr Matrix<4, 4> M = {{
    // nx ny nz nt
    {1, 1, -1, 1},
    {1, -1, 1, 1},
    {-1, 1, 1, 1},
    {-1, -1, -1, 1},
}};

constexpr Matrix<4, 4> M_inv = 0.25 * transpose(M);
};  // namespace MotorControlTransformation

inline ColVector<4>
convertControlSignalToMotorOutputs(const ColVector<4> &u_model) {
    ColVector<4> u_motors = MotorControlTransformation::M * u_model;
    return u_motors;
}

inline std::vector<ColVector<4>>
convertControlSignalToMotorOutputs(const std::vector<ColVector<4>> &u_model) {
    std::vector<ColVector<4>> u_motors;
    u_motors.resize(u_model.size());
    std::transform(u_model.begin(), u_model.end(), u_motors.begin(),
                   [](const ColVector<4> &u_model) {
                       return convertControlSignalToMotorOutputs(u_model);
                   });
    return u_motors;
}

inline ColVector<4>
convertMotorOutputsToControlSignal(const ColVector<4> &u_motors) {
    ColVector<4> u_model = MotorControlTransformation::M_inv * u_motors;
    return u_model;
}

inline ColVector<4> clampMotorControlSignal(const ColVector<4> &u_model_raw) {
    ColVector<4> u_motors = convertControlSignalToMotorOutputs(u_model_raw);
    clamp(u_motors, zeros<4, 1>(), ones<4, 1>());
    return convertMotorOutputsToControlSignal(u_motors);
}

inline ColVector<3> clampMotorControlSignal(const ColVector<3> &u_model_raw,
                                            double u_c) {
    return getBlock<1, 4, 0, 1>(
        clampMotorControlSignal(vcat(u_c * eye<1>(), u_model_raw)));
}

inline void checkControlSignal(const ColVector<4> &u_model_raw) {
    auto u_motors = convertControlSignalToMotorOutputs(u_model_raw);
    for (double u_i : u_motors) {
        if (u_i > 1.0 || u_i < 0.0) {
            std::stringstream ss;
            ss << "Error: control signal out of bounds. u_model = "
               << u_model_raw << ", u_motors = " << u_motors;
            throw std::runtime_error(ss.str());
        }
    }
}