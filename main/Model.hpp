#pragma once

#include "Params.hpp"
#include <DormandPrince.hpp>
#include <Matrix.hpp>

template <class U>
class InputFunctionU {
  public:
    virtual U operator()(double t) = 0;
};

template <class T, size_t Nx, size_t Nu>
class Model {
  public:
    typedef ColVector<T, Nx> VecX_t;
    typedef ColVector<T, Nu> VecU_t;
    typedef InputFunctionU<VecU_t> InputFunction;
    typedef ODEResultX<VecX_t> SimulationResult;

    virtual VecX_t operator()(const VecX_t &x, const VecU_t &u) = 0;
    virtual SimulationResult simulate(InputFunction &u,  // input to the model
                                      VecX_t x_start,    // initial state
                                      const AdaptiveODEOptions &opt  // options
                                      )                         = 0;
};

template <class X>
class ODEFunction {
  public:
    virtual X operator()(double t, const X &x) = 0;
};

template <class T, size_t Nx, size_t Nu>
class ContinuousModel : public Model<T, Nx, Nu> {
  public:
    using VecX_t           = typename Model<T, Nx, Nu>::VecX_t;
    using VecU_t           = typename Model<T, Nx, Nu>::VecU_t;
    using InputFunction    = typename Model<T, Nx, Nu>::InputFunction;
    using SimulationResult = typename Model<T, Nx, Nu>::SimulationResult;

    SimulationResult simulate(InputFunction &u,  // input to the model
                              VecX_t x_start,    // initial state
                              const AdaptiveODEOptions &opt  // options
                              ) override {
        SimulationFunction f = {*this, u};
        return dormandPrince(f, x_start, opt);
    }

  private:
    class SimulationFunction : public ODEFunction<VecX_t> {
      private:
        Model<T, Nx, Nu> &model;
        InputFunction &u;

      public:
        SimulationFunction(Model<T, Nx, Nu> &model, InputFunction &u)
            : model(model), u(u) {}
        VecX_t operator()(double t, const VecX_t &x) override {
            return model(x, u(t));
        }
    };
};

template <class T, size_t Nx, size_t Nu, size_t Ny>
class CTLTISystem : public ContinuousModel<T, Nx, Nu> {
  public:
    using VecX_t = typename Model<T, Nx, Nu>::VecX_t;
    using VecU_t = typename Model<T, Nx, Nu>::VecU_t;
    using VecY_t = ColVector<T, Ny>;

    CTLTISystem(const Matrix<T, Nx, Nx> &A, const Matrix<T, Nx, Nu> &B,
                const Matrix<T, Ny, Nx> &C, const Matrix<T, Ny, Nu> &D)
        : A(A), B(B), C(C), D(D) {}

    VecX_t operator()(const VecX_t &x, const VecU_t &u) override {
        return A * x + B * u;
    }

    VecY_t getOutput(const VecX_t &x, const VecU_t &u) const override {
        return C * x + D * u;
    }

    const Matrix<T, Nx, Nx> A;
    const Matrix<T, Nx, Nu> B;
    const Matrix<T, Ny, Nx> C;
    const Matrix<T, Ny, Nu> D;
};