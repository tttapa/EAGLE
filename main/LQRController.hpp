#pragma once

#include "Def.hpp"
#include <Model/Controller.hpp>
#include <Quaternions/QuaternionStateAddSub.hpp>
#include <Quaternions/ReducedQuaternion.hpp>
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
     */
    LQRController(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                  const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D,
                  const Matrix<Nu, Nx - 1> &K, double Ts)
        : DiscreteController<Nx, Nu, Ny>{Ts}, K(K), G(calculateG(A, B, C, D)) {}

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &r) {
        // new equilibrium state
        ColVector<Nx + Nu> eq = G * r;
        ColVector<Nx> xeq     = getBlock<0, Nx, 0, 1>(eq);
        ColVector<Nu> ueq     = getBlock<Nx, Nx + Nu, 0, 1>(eq);

        // error
        ColVector<Nx> xdiff       = quaternionStatesSub(x, xeq);
        ColVector<Nx - 1> xdiff_r = quat2red(xdiff);

        // controller
        ColVector<Nu> u_ctrl = K * xdiff_r;
        ColVector<Nu> u      = u_ctrl + ueq;
        return u;
    }

    const Matrix<Nu, Nx - 1> K;
    const Matrix<Nx + Nu, Ny> G;
};

class ClampedLQRController : public LQRController {
  public:
    ClampedLQRController(const LQRController &controller,
                         const ColVector<Nu> &min, const ColVector<Nu> &max)
        : LQRController{controller}, min{min}, max{max} {}

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        auto u = getRawControllerOutput(x, r);
        clamp(u, min, max);
        return u;
    }

    const ColVector<Nu> min;
    const ColVector<Nu> max;
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
     * @param   K_p
     *          Proportional controller matrix K.
     * @param   K_i
     *          Integral controller matrix K.
     * @param   Ts
     *          Time step.
     */
    LQRController(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                  const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D,
                  const Matrix<Nu, Nx> &K_p, const Matrix<Nu, Nx> &K_i,
                  const VecU_t &uh, double Ts)
        : DiscreteController<Nx, Nu, Ny>{Ts}, K_p(K_p), K_i(K_i),
          G(calculateG(A, B, C, D)), uh{uh} {
              std::cout << "Altitude::LQRController::G = " << G << std::endl;
          }

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &r) {
        // new equilibrium state
        ColVector<Nx + Nu> eq = G * r;
        VecX_t xeq            = getBlock<0, Nx, 0, 1>(eq);
        VecU_t ueq            = getBlock<Nx, Nx + Nu, 0, 1>(eq);

        // error
        VecX_t xdiff = xeq - x;

        // integral
        integral += xdiff;

        // controller
        VecU_t u_ctrl = K_p * xdiff + K_i * integral;
        VecU_t u      = u_ctrl + ueq;
        return u + uh;
    }

    const Matrix<Nu, Nx> K_p;
    const Matrix<Nu, Nx> K_i;
    const Matrix<Nx + Nu, Ny> G;
    const VecU_t uh;

    VecX_t integral = {};
};

}  // namespace Altitude
