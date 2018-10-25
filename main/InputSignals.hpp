#pragma once

#include "ContinuousLQRController.hpp"
#include "NonLinearFullModel.hpp"

/** 
 * @brief   Input signal for simulations without controllers.
 */
struct TestInputFunction : public NonLinearFullModel::InputFunction {
    NonLinearFullModel::VecU_t operator()(double t) override {
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
        ContinuousLQRController::VecR_t ref = {1};
        if (t > 1 && t < 5)
            assignBlock<0, 4, 0, 1>(ref) = q1;
        return ref;
        // ---------------------------------------------------------------------
    }
    const Quaternion q1 = eul2quat({M_PI / 4, 0, 0});
};

// -------------------------------------------------------------------------- //

/**
 * @brief   Initial state of the drone for simulations
 */
NonLinearFullModel::VecX_t x0 = {{
    {1},    // q0
    {0},    // q1
    {0},    // q2
    {0},    // q3 ____
    {0.1},  // ωx
    {0},    // ωy
    {0},    // ωz ____
    {0},    // nx
    {0},    // ny
    {0},    // nz
}};