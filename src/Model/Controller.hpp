#pragma once

#include <Matrix/LeastSquares.hpp>
#include <Model/Model.hpp>
#include <Quaternions/Quaternion.hpp>

template <size_t Nx, size_t Nu, size_t Nr>
class Controller {
  public:
    typedef ColVector<Nx> VecX_t;  // state vectors
    typedef ColVector<Nu> VecU_t;  // input vectors
    typedef ColVector<Nr> VecR_t;  // reference vectors
    typedef InputFunctionU<VecR_t> ReferenceFunction;
    typedef ODEResultX<VecX_t> SimulationResult;

    virtual VecU_t operator()(const VecX_t &x, const VecR_t &r) = 0;
    virtual SimulationResult
    simulate(Model<Nx, Nu> &model,       // the model to control
             ReferenceFunction &r,          // reference input
             VecX_t x_start,                // initial state
             const AdaptiveODEOptions &opt  // options
             ) = 0;
};

template <size_t Nx, size_t Nu, size_t Nr>
class ContinuousController : public Controller<Nx, Nu, Nr> {
  public:
    using VecX_t = typename Controller<Nx, Nu, Nr>::VecX_t;
    using VecU_t = typename Controller<Nx, Nu, Nr>::VecU_t;
    using VecR_t = typename Controller<Nx, Nu, Nr>::VecR_t;
    using ReferenceFunction =
        typename Controller<Nx, Nu, Nr>::ReferenceFunction;
    using SimulationResult = typename Controller<Nx, Nu, Nr>::SimulationResult;

    SimulationResult simulate(Model<Nx, Nu> &model,  // the model to control
                              ReferenceFunction &r,  // reference input
                              VecX_t x_start,        // initial state
                              const AdaptiveODEOptions &opt  // options
                              ) override {
        SimulationFunction f = {model, *this, r};
        return dormandPrince(f, x_start, opt);
    }

  private:
    class SimulationFunction : public ODEFunction<VecX_t> {
      private:
        Model<Nx, Nu> &model;
        Controller<Nx, Nu, Nr> &ctrl;
        ReferenceFunction &r;

      public:
        SimulationFunction(Model<Nx, Nu> &model, Controller<Nx, Nu, Nr> &ctrl,
                           ReferenceFunction &r)
            : model(model), ctrl(ctrl), r(r) {}
        VecX_t operator()(double t, const VecX_t &x) override {
            return model(x, ctrl(x, r(t)));
        }
    };
};