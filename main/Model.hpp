#pragma once

#include "Params.hpp"
#include <DormandPrince.hpp>
#include <Matrix.hpp>
#include <vector>

template <class V>
struct SimulationResultX {
    std::vector<double> time;
    std::vector<V> solution;
};

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
    typedef SimulationResultX<VecX_t> SimulationResult;

    virtual VecX_t operator()(const VecX_t &x, const VecU_t &u) = 0;
    virtual SimulationResult
    simulate(InputFunction &u,
             double t_start,  // initial value for independent variable
             double t_end,    // final value for independent variable
             double epsilon,  // tolerance
             double h_start,  // initial step size
             VecX_t x_start,  // initial state
             size_t maxiter   // maximum number of iterations
             ) = 0;
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

    SimulationResult
    simulate(InputFunction &u,
             double t_start,  // initial value for independent variable
             double t_end,    // final value for independent variable
             double epsilon,  // tolerance
             double h_start,  // initial step size
             VecX_t x_start,  // initial state
             size_t maxiter   // maximum number of iterations
             ) override {
        SimulationFunction f = {*this, u};
        auto result = dormandPrince(f, t_start, t_end, epsilon, h_start,
                                    x_start, maxiter);
        return {result.first, result.second};
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