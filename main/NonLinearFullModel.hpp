#pragma once

#include <Model/Model.hpp>
#include <ODE/DormandPrince.hpp>
#include <Quaternions/Quaternion.hpp>

class NonLinearFullModel : public ContinuousModel<10, 3> {
  public:
    using VecOmega_t = ColVector<3>;
    using VecN_t     = ColVector<3>;

    /** Number of states */
    constexpr static size_t nx = 10;
    /** Number of inputs */
    constexpr static size_t nu = 3;

    /** 
     * @brief   Construct an instance of NonLinearFullModel with the given
     *          parameters.
     * @param   p 
     *          The parameters of the drone.
     */
    NonLinearFullModel(const Params &p) : p(p) {}

    /** 
     * @brief   Calculate the derivative of the state vector, given the current
     *          state x and the current input u.
     * @param   x 
     *          The current state of the drone.
     * @param   u 
     *          The current control input u.
     * @return  The derivative of the state, x_dot.
     */
    VecX_t operator()(const VecX_t &x, const VecU_t &u) override {
        Quaternion q     = getBlock<0, 4, 0, 1>(x);
        VecOmega_t omega = getBlock<4, 7, 0, 1>(x);
        VecN_t n         = getBlock<7, 10, 0, 1>(x);

        Quaternion q_omega = vcat(zeros<1, 1>(), omega);

        Quaternion q_dot = 0.5 * quatmultiply(q, q_omega);
        VecOmega_t omega_dot =
            p.gamma_n * n + p.gamma_u * u - p.I_inv * cross(omega, p.I * omega);
        VecN_t n_dot = p.k2 * (p.k1 * u - n);

        VecX_t x_dot = vcat(q_dot, omega_dot, n_dot);

        return x_dot;
    }

    Params p;

    /** 
     * @brief   Get the rotation in Euler angles, given a state vector x.
     */
    static EulerAngles stateToEuler(const VecX_t &x) {
        Quaternion q = getBlock<0, 4, 0, 1>(x);
        return quat2eul(q);
    }
};