#pragma once

#include <Model/Controller.hpp>
#include <Model/System.hpp>
#include <algorithm>  // transform
#include <cassert>    // assert

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
        // new equilibrium state
        ColVector<nx + nu> eq = G * r;
        ColVector<nx> xeq     = getBlock<0, nx, 0, 1>(eq);
        ColVector<nu> ueq     = getBlock<nx, nx + nu, 0, 1>(eq);

        // error
        ColVector<nx> xdiff            = x - xeq;
        Quaternion qx                  = getBlock<0, 4, 0, 1>(x);
        Quaternion qe                  = getBlock<0, 4, 0, 1>(xeq);
        assignBlock<0, 4, 0, 1>(xdiff) = quatDifference(qx, qe);

        // controller
        ColVector<nu> uc = K * xdiff;
        auto u           = uc + ueq;
        return u;
    }

    /**
     * @brief   Given a time and states vector, and a reference signal,
     *          calculate the output of the controller.
     */
    virtual std::vector<VecU_t>
    getControlSignal(const std::vector<double> &time,
                     const std::vector<VecX_t> &states,
                     ReferenceFunction &ref) {
        std::vector<VecU_t> u;
        u.resize(time.size());
        auto fn = [this, &ref](double t, auto x) { return (*this)(x, ref(t)); };
        std::transform(time.begin(), time.end(), states.begin(), u.begin(), fn);
        return u;
    }

  private:
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
    static Matrix<nx + nu, ny> calculateG(const Matrix<nx, nx> &A,
                                          const Matrix<nx, nu> &B,
                                          const Matrix<ny, nx> &C,
                                          const Matrix<ny, nu> &D,
                                          bool continuous) {
        Matrix<nx, nx> Aa = continuous ? A : A - eye<nx>();
        /* W =  [ Aa B ]
                [ C  D ] */
        Matrix<nx + ny, nx + nu> W = vcat(  //
            hcat(Aa, B),                    //
            hcat(C, D)                      //
        );
        Matrix<nx + ny, ny> OI     = vcat(zeros<nx, ny>(), eye<ny>());
        auto G                     = solveLeastSquares(W, OI);
        return G;
    }

  public:
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

    std::vector<VecU_t> getControlSignal(const std::vector<double> &time,
                                         const std::vector<VecX_t> &states,
                                         ReferenceFunction &ref) override {
        assert(time.size() == states.size());
        std::vector<VecU_t> u;
        u.resize(time.size());
        auto t_it = time.begin();
        auto u_it = u.begin();
        auto x_it = states.begin();
        while (t_it != time.end()) {
            double t_end  = *t_it + Ts;
            VecU_t curr_u = (*this)(*x_it, ref(*t_it));
            while (*t_it < t_end && t_it != time.end()) {
                *u_it = curr_u;
                ++u_it;
                ++t_it;
                ++x_it;
            }
        }
        return u;
    }
};