#pragma once

#include "LQRController.hpp"
#include "NonLinearFullDroneModel.hpp"

/** 
 * @brief   Input signal for simulations without controllers.
 */
struct TestInputFunction : public NonLinearFullDroneModel::InputFunction {
    NonLinearFullDroneModel::VecU_t operator()(double t) override {
        // ---------------------------------------------------------------------
        return {{
            {0},
            {0},
            {-0.1 * (t < 2) + 0.2 * (t < 3) - 0.1 * (t < 5)},
        }};
        // ---------------------------------------------------------------------
    }
};

/** 
 * @brief   Reference function for controller simulations.
 */
struct TestReferenceFunction
    : public ContinuousLQRController::ReferenceFunction {
    ContinuousLQRController::VecR_t operator()(double t) override {
        // ---------------------------------------------------------------------
        ContinuousLQRController::VecR_t ref = {};

        Quaternion q = qu;
        if (t >= 1 && t < 3)
            q = quatmultiply(q, qz);
        if (t >= 5 && t < 7)
            q = quatmultiply(q, qy);
        if (t >= 9 && t < 11)
            q = quatmultiply(q, qx);

        if (t >= 13 && t < 15)
            q = quatmultiply(q, quatconjugate(qz));
        if (t >= 17 && t < 19)
            q = quatmultiply(q, quatconjugate(qy));
        if (t >= 21 && t < 23)
            q = quatmultiply(q, quatconjugate(qx));

        if (t >= 25 && t < 27)
            q = quatmultiply(q, qz);
        if (t >= 26 && t < 28)
            q = quatmultiply(q, qy);
        if (t >= 27 && t < 29)
            q = quatmultiply(q, qx);
        assignBlock<0, 4, 0, 1>(ref) = q;
        return ref;
        // ---------------------------------------------------------------------
    }
    const Quaternion qz = eul2quat({M_PI / 8, 0, 0});
    const Quaternion qy = eul2quat({0, M_PI / 16, 0});
    const Quaternion qx = eul2quat({0, 0, M_PI / 16});
    const Quaternion qu = eul2quat({0, 0, 0});
};

// -------------------------------------------------------------------------- //

/**
 * @brief   Initial state of the drone for simulations
 */
constexpr NonLinearFullDroneModel::VecX_t x0 = {{
    {1},  // q0
    {0},  // q1
    {0},  // q2
    {0},  // q3 ____
    {0},  // ωx
    {0},  // ωy
    {0},  // ωz ____
    {0},  // nx
    {0},  // ny
    {0},  // nz
}};

/** [1 0 0 0 0 0 0] */
ConstantTimeFunctionT<ColVector<7>> ref0 = vcat(ones<1, 1>(), zeros<6, 1>());