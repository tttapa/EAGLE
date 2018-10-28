#pragma once

#include <Matrix/LeastSquares.hpp>
#include <Model/Model.hpp>
#include <Quaternions/Quaternion.hpp>

/**
 * @brief   An abstract class for controllers.
 *          A controller produces a control signal @f$ u @f$ given a state 
 *          @f$ x @f$ and a reference target @f$ r @f$.
 */
template <size_t Nx, size_t Nu, size_t Nr>
class Controller {
  public:
    typedef ColVector<Nx> VecX_t;  // state vectors
    typedef ColVector<Nu> VecU_t;  // input vectors
    typedef ColVector<Nr> VecR_t;  // reference vectors
    typedef TimeFunctionT<VecR_t> ReferenceFunction;
    typedef ODEResultX<VecX_t> SimulationResult;

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
    virtual VecU_t operator()(const VecX_t &x, const VecR_t &r) = 0;

    /**
     * @brief   Simulate the given model with the given reference, initial 
     *          state, and integration options.
     * 
     * @param   model
     *          The model to control.
     * @param   r 
     *          The reference function to steer the model towards.
     * @param   x_start
     *          The initial state of the model.
     * @param   opt 
     *          Options for the integration.
     */
    virtual SimulationResult simulate(ContinuousModel<Nx, Nu> &model,
                                      ReferenceFunction &r, VecX_t x_start,
                                      const AdaptiveODEOptions &opt) = 0;
};

template <size_t Nx, size_t Nu, size_t Nr>
class ContinuousController : public virtual Controller<Nx, Nu, Nr> {
  public:
    using VecX_t = typename Controller<Nx, Nu, Nr>::VecX_t;
    using VecU_t = typename Controller<Nx, Nu, Nr>::VecU_t;
    using VecR_t = typename Controller<Nx, Nu, Nr>::VecR_t;
    using ReferenceFunction =
        typename Controller<Nx, Nu, Nr>::ReferenceFunction;
    using SimulationResult = typename Controller<Nx, Nu, Nr>::SimulationResult;

    SimulationResult
    simulate(ContinuousModel<Nx, Nu> &model,  // the model to control
             ReferenceFunction &r,            // reference input
             VecX_t x_start,                  // initial state
             const AdaptiveODEOptions &opt    // options
             ) override {
        auto f = [&model, this, &r](double t, const VecX_t &x) {
            ContinuousController &ctrl = *this;
            return model(x, ctrl(x, r(t)));
        };
        return dormandPrince(f, x_start, opt);
    }
};

template <size_t Nx, size_t Nu, size_t Nr>
class DiscreteController : public virtual Controller<Nx, Nu, Nr> {
  public:
    using VecX_t = typename Controller<Nx, Nu, Nr>::VecX_t;
    using VecU_t = typename Controller<Nx, Nu, Nr>::VecU_t;
    using VecR_t = typename Controller<Nx, Nu, Nr>::VecR_t;
    using ReferenceFunction =
        typename Controller<Nx, Nu, Nr>::ReferenceFunction;
    using SimulationResult = typename Controller<Nx, Nu, Nr>::SimulationResult;

    DiscreteController(double Ts) : Ts(Ts) {}

    SimulationResult simulate(ContinuousModel<Nx, Nu> &model,
                              ReferenceFunction &r, VecX_t x_start,
                              const AdaptiveODEOptions &opt) override {
        SimulationResult result     = {};
        size_t N                    = floor((opt.t_end - opt.t_start) / Ts) + 1;
        VecX_t curr_x               = x_start;
        AdaptiveODEOptions curr_opt = opt;
        for (size_t i = 0; i < N; ++i) {
            double t         = opt.t_start + Ts * i;
            curr_opt.t_start = t;
            curr_opt.t_end   = t + Ts;
            VecR_t curr_ref  = r(t);
            VecU_t curr_u    = (*this)(curr_x, curr_ref);
            result.resultCode |= model.simulate(
                std::back_inserter(result.time),
                std::back_inserter(result.solution), curr_u, curr_x, curr_opt);
            curr_x = result.solution.back();
            result.time.pop_back();
            result.solution.pop_back();
        }
        return result;
    }

    const double Ts;
};
