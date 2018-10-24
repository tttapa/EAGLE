#pragma once

#include <Matrix/LeastSquares.hpp>
#include <Model/Model.hpp>
#include <Quaternions/Quaternion.hpp>

template <class T, size_t Nx, size_t Nu, size_t Nr>
class Controller {
  public:
    typedef ColVector<T, Nx> VecX_t;  // state vectors
    typedef ColVector<T, Nu> VecU_t;  // input vectors
    typedef ColVector<T, Nr> VecR_t;  // reference vectors
    typedef InputFunctionU<VecR_t> ReferenceFunction;
    typedef ODEResultX<VecX_t> SimulationResult;

    virtual VecU_t operator()(const VecX_t &x, const VecR_t &r) = 0;
    virtual SimulationResult
    simulate(Model<T, Nx, Nu> &model,       // the model to control
             ReferenceFunction &r,          // reference input
             VecX_t x_start,                // initial state
             const AdaptiveODEOptions &opt  // options
             ) = 0;
};

template <class T, size_t Nx, size_t Nu, size_t Nr>
class ContinuousController : public Controller<T, Nx, Nu, Nr> {
  public:
    using VecX_t = typename Controller<T, Nx, Nu, Nr>::VecX_t;
    using VecU_t = typename Controller<T, Nx, Nu, Nr>::VecU_t;
    using VecR_t = typename Controller<T, Nx, Nu, Nr>::VecR_t;
    using ReferenceFunction =
        typename Controller<T, Nx, Nu, Nr>::ReferenceFunction;
    using SimulationResult =
        typename Controller<T, Nx, Nu, Nr>::SimulationResult;

    SimulationResult simulate(Model<T, Nx, Nu> &model,  // the model to control
                              ReferenceFunction &r,     // reference input
                              VecX_t x_start,           // initial state
                              const AdaptiveODEOptions &opt  // options
                              ) override {
        SimulationFunction f = {model, *this, r};
        return dormandPrince(f, x_start, opt);
    }

  private:
    class SimulationFunction : public ODEFunction<VecX_t> {
      private:
        Model<T, Nx, Nu> &model;
        Controller<T, Nx, Nu, Nr> &ctrl;
        ReferenceFunction &r;

      public:
        SimulationFunction(Model<T, Nx, Nu> &model,
                           Controller<T, Nx, Nu, Nr> &ctrl,
                           ReferenceFunction &r)
            : model(model), ctrl(ctrl), r(r) {}
        VecX_t operator()(double t, const VecX_t &x) override {
            return model(x, ctrl(x, r(t)));
        }
    };
};

class ContinuousLQRController : public ContinuousController<double, 10, 3, 7> {
  public:
    ContinuousLQRController(const Matrix<double, 10, 10> &A,
                            const Matrix<double, 10, 3> &B,
                            const Matrix<double, 7, 10> &C,
                            const Matrix<double, 7, 3> &D,
                            const Matrix<double, 3, 10> &K)
        : K(K) {
        Matrix<double, 10 + 7, 10 + 3> W;
        assignBlock<0, 10, 0, 10>(W)      = A;
        assignBlock<0, 10, 10, 13>(W)     = B;
        assignBlock<10, 17, 0, 10>(W)     = C;
        assignBlock<10, 17, 10, 13>(W)    = D;
        Matrix<double, 10 + 7, 7> OI      = {};
        assignBlock<10, 10 + 7, 0, 7>(OI) = eye<double, 7>();
        G                                 = solveLeastSquares(W, OI);
    }

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        // new equilibrium state
        ColVector<double, 10 + 3> eq = G * r;
        auto xeq                     = getBlock<0, 10, 0, 1>(eq);
        auto ueq                     = getBlock<10, 13, 0, 1>(eq);

        // error
        ColVector<double, 10> xdiff    = x - xeq;
        auto qx                        = getBlock<0, 4, 0, 1>(x);
        auto qe                        = getBlock<0, 4, 0, 1>(eq);
        assignBlock<0, 4, 0, 1>(xdiff) = quatDifference(qx, qe);

        // controller
        ColVector<double, 3> uc = K * xdiff;
        auto u                  = uc + ueq;
        return u;
    }

  private:
    Matrix<double, 10 + 3, 7> G;
    Matrix<double, 3, 10> K;
};