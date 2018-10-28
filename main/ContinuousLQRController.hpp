#pragma once

#include <Model/Controller.hpp>
#include <Model/System.hpp>
#include <algorithm>
#include <cassert>

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
     */
    LQRController(const Matrix<nx, nx> &A, const Matrix<nx, nu> &B,
                  const Matrix<ny, nx> &C, const Matrix<ny, nu> &D,
                  const Matrix<nu, nx> &K, bool continuous)
        : K(K), G(calculateG(A, B, C, D, continuous)) {}

  public:
    /**
     * @brief   Given a state x, and a reference value r, calculate the
     *          controller output.
     * 
     * @param   x 
     *          The current state of the system.
     * @param   r 
     *          The current reference output for the system. 
     * @return  The controller output.
     */
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
        std::vector<LQRController::VecU_t> u;
        u.resize(time.size());
        auto lambda = [this, &ref](double t, auto x) {
            return (*this)(x, ref(t));
        };
        std::transform(time.begin(), time.end(), states.begin(), u.begin(),
                       lambda);
        return u;
    }

  private:
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
};

class DiscreteLQRController : public LQRController,
                              public DiscreteController<10, 3, 7> {
  public:
    DiscreteLQRController(const Matrix<nx, nx> &A, const Matrix<nx, nu> &B,
                          const Matrix<ny, nx> &C, const Matrix<ny, nu> &D,
                          const Matrix<nu, nx> &K, double Ts)
        : LQRController(A, B, C, D, K, false), DiscreteController<10, 3, 7>(
                                                   Ts) {}

    DiscreteLQRController(const DTLTISystem<10, 3, 7> &discreteSystem,
                          const Matrix<nu, nx> &K)
        : LQRController(discreteSystem.A, discreteSystem.B, discreteSystem.C,
                        discreteSystem.D, K, false),
          DiscreteController<10, 3, 7>(discreteSystem.Ts) {}

    std::vector<VecU_t> getControlSignal(const std::vector<double> &time,
                                         const std::vector<VecX_t> &states,
                                         ReferenceFunction &ref) override {
        assert(time.size() == states.size());
        std::vector<LQRController::VecU_t> u;
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