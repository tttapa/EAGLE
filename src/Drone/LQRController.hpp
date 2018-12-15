#pragma once

#include "Def.hpp"
#include <Model/Controller.hpp>
#include <Quaternions/QuaternionStateAddSub.hpp>
#include <Quaternions/ReducedQuaternion.hpp>
#include <cassert>
#include <iostream>

/**
 *  Solves the system of equations
 *
 *   @f$ 
 *   \begin{cases}
 *       \dot{x} = A x + B u = 0 \\
 *       y = C x + D u = r
 *   \end{cases} \\
 *   \begin{cases}
 *       x_{k+1} = A x_k + B u = x_k \\
 *       y = C x_k + D u = r
 *   \end{cases}
 *   @f$  
 *   for the continuous and the discrete case respectively, for aNy given 
 *   reference output @f$ r @f$.
 *   
 *   @return A matrix @f$ G @f$ such that
 *           @f$ \begin{pmatrix} x^e \\ u^e \end{pmatrix} = G r @f$.
 */
template <size_t Nx, size_t Nu, size_t Ny>
Matrix<Nx + Nu, Ny> calculateG(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                               const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D,
                               bool continuous = false) {
    Matrix<Nx, Nx> As = continuous ? A : A - eye<Nx>();
    /* W =  [ As B ]
            [ C  D ] */
    Matrix<Nx + Ny, Nx + Nu> W = vcat(  //
        hcat(As, B),                    //
        hcat(C, D)                      //
    );
    Matrix<Nx + Ny, Ny> OI     = vcat(zeros<Nx, Ny>(), eye<Ny>());
    Matrix<Nx + Nu, Ny> G      = solveLeastSquares(W, OI);
    return G;
}

namespace Attitude {

constexpr size_t Nx = Nx_att;
constexpr size_t Nu = Nu_att;
constexpr size_t Ny = Ny_att;

/**
 * @brief   A class for the discrete-time LQR attitude controller for the drone.
 * 
 * @note    This class is specifically for the attitude controller, as it uses
 *          Hamiltonian quaternion multiplication for the difference of the
 *          first four states.
 */
class LQRController : public DiscreteController<Nx, Nu, Ny> {
  public:
    /** 
     * @brief   Construct a new instance of LQRController with the 
     *          given equilibrium matrix G, and the given proportional 
     *          controller K.
     * 
     * @param   G
     *          Equilibrium matrix G.
     * @param   K
     *          Proportional controller matrix K.
     * @param   Ts
     *          The sample time of the discrete controller.
     */
    LQRController(const Matrix<Nx + Nu, Ny> &G, const Matrix<Nu, Nx - 1> &K,
                  double Ts)
        : DiscreteController<Nx, Nu, Ny>{Ts}, K(K), G(G) {}
    /** 
     * @brief   Construct a new instance of LQRController with the 
     *          given system matrices A, B, C, D, and the given proportional 
     *          controller K.
     * 
     * @param   Ad
     *          System matrix A.
     * @param   Bd
     *          System matrix B.
     * @param   Cd
     *          System matrix C.
     * @param   Dd
     *          System matrix D.
     * @param   K
     *          Proportional controller matrix K.
     * @param   Ts
     *          The sample time of the discrete controller.
     */
    LQRController(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                  const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D,
                  const Matrix<Nu, Nx - 1> &K, double Ts)
        : DiscreteController<Nx, Nu, Ny>{Ts}, K(K), G(calculateG(A, B, C, D)) {
        std::cout << "Attitude::LQRController::G = " << G;
    }

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &r) {
        // new equilibrium state
        ColVector<Nx + Nu> eq = G * r;
        ColVector<Nx> xeq     = getBlock<0, Nx, 0, 1>(eq);
        ColVector<Nu> ueq     = getBlock<Nx, Nx + Nu, 0, 1>(eq);

        // error
        ColVector<Nx> x_err       = quaternionStatesSub(x, xeq);
        ColVector<Nx - 1> x_err_r = quat2red(x_err);

        // controller
        ColVector<Nu> u_ctrl = K * x_err_r;
        ColVector<Nu> u      = u_ctrl + ueq;
        return u;
    }

    const Matrix<Nu, Nx - 1> K;
    const Matrix<Nx + Nu, Ny> G;
};

}  // namespace Attitude

namespace Altitude {

constexpr size_t Nx = Nx_alt;
constexpr size_t Nu = Nu_alt;
constexpr size_t Ny = Ny_alt;

/**
 * @brief   A class for the discrete-time LQR altitude controller for the drone.
 */
class LQRController : public DiscreteController<Nx, Nu, Ny> {
  public:
    /** 
     * @brief   Construct a new instance of LQRController with the 
     *          given equilibrium matrix G, and the given proportional 
     *          controller K.
     * 
     * @param   G
     *          Equilibrium matrix G.
     * @param   K
     *          Proportional controller matrix K.
     * @param   Ts
     *          The sample time of the discrete controller.
     */
    LQRController(const Matrix<Nx + Nu, Ny> &G, const Matrix<Ny, Nx> &C,
                  const Matrix<Nu, Nx + Ny> &K_pi, double Ts,
                  double maxIntegralInfluence)
        : DiscreteController<Nx, Nu, Ny>{Ts}, K_pi{K_pi}, G{G}, C{C},
          maxIntegral{fabs(maxIntegralInfluence / K_pi[0][Nx])} {}

    /** 
     * @brief   Construct a new instance of LQRController with the 
     *          given system matrices A, B, C, D, and the given proportional 
     *          controller K.
     * 
     * @param   Ad
     *          System matrix A.
     * @param   Bd
     *          System matrix B.
     * @param   Cd
     *          System matrix C.
     * @param   Dd
     *          System matrix D.
     * @param   K_pi
     *          Proportional and integral controller matrix K.
     * @param   K_i
     *          Integral controller matrix K.
     * @param   Ts
     *          Time step.
     */
    LQRController(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                  const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D,
                  const Matrix<Nu, Nx + Ny> &K_pi, double Ts,
                  double maxIntegralInfluence)
        : DiscreteController<Nx, Nu, Ny>{Ts}, K_pi{K_pi},
          G(calculateG(A, B, C, D)), C{C},
          maxIntegral{fabs(maxIntegralInfluence / K_pi[0][Nx])} {}

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &r) {
        // new equilibrium state
        ColVector<Nx + Nu> eq = G * r;
        VecX_t xeq            = getBlock<0, Nx, 0, 1>(eq);
        VecU_t ueq            = getBlock<Nx, Nx + Nu, 0, 1>(eq);

        VecR_t y = C * x;

        // error
        VecX_t x_err = xeq - x;
        VecR_t y_err = y - r;

        // controller
        VecU_t u_ctrl = K_pi * vcat(x_err, integral);
        VecU_t u      = u_ctrl + ueq;

        // TODO: should the controller be based on the new integral?
        // integral
        double newIntegral = integral + y_err * Ts;
#if 0
        // TODO: is this better?
        if (abs(newIntegral) > maxIntegral)
            newIntegral = copysign(maxIntegral, newIntegral);
        integral = {newIntegral};
#else
        if (abs(newIntegral) <= maxIntegral)
            integral = {newIntegral};
#endif
        return u;
    }

    void reset() override { integral = {}; }

    const Matrix<Nu, Nx + Ny> K_pi;
    const Matrix<Nx + Nu, Ny> G;
    const Matrix<Ny, Nx> C;

    const double maxIntegral;

    VecR_t integral = {};
};

}  // namespace Altitude
