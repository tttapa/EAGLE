#pragma once

#include <Model/Model.hpp>
#include <ODE/DormandPrince.hpp>
#include <Quaternions/Quaternion.hpp>

class NonLinearFullModel : public ContinuousModel<10, 3> {
  public:
    using VecOmega_t = ColVector<3>;
    using VecN_t     = ColVector<3>;

    constexpr static size_t nx = 10;
    constexpr static size_t nu = 3;

    NonLinearFullModel(const Params &p) : p(p) {}

    VecX_t operator()(const VecX_t &x, const VecU_t &u) override {
        Quaternion q     = getBlock<0, 4, 0, 1>(x);
        VecOmega_t omega = getBlock<4, 7, 0, 1>(x);
        VecN_t n         = getBlock<7, 10, 0, 1>(x);

        Quaternion q_omega               = vcat(zeros<1, 1>(), omega);

        Quaternion q_dot = 0.5 * quatmultiply(q, q_omega);
        VecOmega_t omega_dot =
            p.gamma_n * n + p.gamma_u * u - p.I_inv * cross(omega, p.I * omega);
        VecN_t n_dot = p.k2 * (p.k1 * u - n);

        VecX_t x_dot = vcat(q_dot, omega_dot, n_dot);

        return x_dot;
    }
    const Params p;

    static EulerAngles stateToEuler(const VecX_t &x) {
        Quaternion q = getBlock<0, 4, 0, 1>(x);
        return quat2eul(q);
    }
};