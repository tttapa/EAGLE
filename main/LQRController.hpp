#pragma once

#include "MotorControl.hpp"
#include <Model/Controller.hpp>
#include <Model/System.hpp>
#include <Quaternions/QuaternionStateAddSub.hpp>

/**
 * @brief   A class for the LQR attitude controller for the drone.
 * 
 * @note    This class is specifically for the attitude controller, as it uses
 *          Hamiltonian quaternion multiplication for the difference of the
 *          first four states.
 */
class LQRController : public virtual Controller<10, 3, 7> {
  public:
    /** Number of states. */
    static constexpr size_t nx = 10;
    /** Number of inputs. */
    static constexpr size_t nu = 3;
    /** Number of outputs. */
    static constexpr size_t ny = 7;

  protected:
    /** 
     * @brief   Construct a new instance of ContinuousLQRController with the 
     *          given system matrices A, B, C, D, and the given proportional 
     *          controller K.
     * 
     * @param   A
     *          System matrix A.
     * @param   B
     *          System matrix B.
     * @param   C 
     *          System matrix C.
     * @param   D 
     *          System matrix D.
     * @param   K 
     *          Proportional controller matrix K.
     * @param   continuous
     *          A flag that should be `true` for a continuous controller, and 
     *          `false` for a discrete controller.  
     *          This changes the calculation of the new equilibrium point.
     */
    LQRController(const Matrix<nx, nx> &A, const Matrix<nx, nu> &B,
                  const Matrix<ny, nx> &C, const Matrix<ny, nu> &D,
                  const Matrix<nu, nx> &K, bool continuous)
        : K(K), G(calculateG(A, B, C, D, continuous)) {}

  public:
    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &r) {
        // new equilibrium state
        ColVector<nx + nu> eq = G * r;
        ColVector<nx> xeq     = getBlock<0, nx, 0, 1>(eq);
        ColVector<nu> ueq     = getBlock<nx, nx + nu, 0, 1>(eq);

        // error
        ColVector<nx> xdiff = quaternionStatesSub(x, xeq);

        // controller
        ColVector<nu> u_ctrl = K * xdiff;
        ColVector<nu> u      = u_ctrl + ueq;
        return u;
    }

    /**
        Solves the system of equations

        @f$ 
        \begin{cases}
            \dot{x} = A x + B u = 0 \\
            y = C x + D u = r
        \end{cases} \\
        \begin{cases}
            x_{k+1} = A x_k + B u = x_k \\
            y = C x_k + D u = r
        \end{cases}
        @f$  
        for the continuous and the discrete case respectively, for any given 
        reference output @f$ r @f$.
        
        @return A matrix @f$ G @f$ such that
                @f$ \begin{pmatrix} x^e \\ u^e \end{pmatrix} = G r @f$.
    */
    template <size_t Nx, size_t Nu, size_t Ny>
    static Matrix<Nx + Nu, Ny>
    calculateG(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
               const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D,
               bool continuous) {
        Matrix<Nx, Nx> Aa = continuous ? A : A - eye<Nx>();
        /* W =  [ Aa B ]
                [ C  D ] */
        Matrix<Nx + Ny, Nx + Nu> W = vcat(  //
            hcat(Aa, B),                    //
            hcat(C, D)                      //
        );
        Matrix<Nx + Ny, Ny> OI     = vcat(zeros<Nx, Ny>(), eye<Ny>());
        auto G                     = solveLeastSquares(W, OI);
        return G;
    }

    const Matrix<nu, nx> K;
    const Matrix<nx + nu, ny> G;
};

class ContinuousLQRController : public LQRController,
                                public ContinuousController<10, 3, 7> {
  public:
    ContinuousLQRController(const Matrix<nx, nx> &A, const Matrix<nx, nu> &B,
                            const Matrix<ny, nx> &C, const Matrix<ny, nu> &D,
                            const Matrix<nu, nx> &K)
        : LQRController(A, B, C, D, K, true) {}

    ContinuousLQRController(const CTLTISystem<nx, nu, ny> &continuousSystem,
                            const Matrix<nu, nx> &K)
        : LQRController(continuousSystem.A, continuousSystem.B,
                        continuousSystem.C, continuousSystem.D, K, true) {}
};

class DiscreteLQRController : public LQRController,
                              public DiscreteController<10, 3, 7> {
  public:
    DiscreteLQRController(const Matrix<nx, nx> &A, const Matrix<nx, nu> &B,
                          const Matrix<ny, nx> &C, const Matrix<ny, nu> &D,
                          const Matrix<nu, nx> &K, double Ts)
        : LQRController(A, B, C, D, K, false), DiscreteController<10, 3, 7>(
                                                   Ts) {}

    DiscreteLQRController(const DTLTISystem<nx, nu, ny> &discreteSystem,
                          const Matrix<nu, nx> &K)
        : LQRController(discreteSystem.A, discreteSystem.B, discreteSystem.C,
                        discreteSystem.D, K, false),
          DiscreteController<nx, nu, ny>(discreteSystem.Ts) {}
};

class ClampedDiscreteLQRController : public DiscreteLQRController {
  public:
    ClampedDiscreteLQRController(
        const DiscreteLQRController &discreteController,
        const ColVector<nu> &min, const ColVector<nu> &max)
        : DiscreteLQRController{discreteController}, min{min}, max{max} {}

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        auto u = getRawControllerOutput(x, r);
        for (size_t i = 0; i < nu; ++i)
            u[i][0] = clamp(u[i][0], min[i][0], max[i][0]);
        return u;
    }

    const ColVector<nu> min;
    const ColVector<nu> max;
};