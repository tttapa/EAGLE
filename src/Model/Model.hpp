#pragma once

#include "Controller.hpp"
#include "Kalman.hpp"
#include "Params.hpp"
#include <ODE/DormandPrince.hpp>
#include <Util/Time.hpp>
#include <Util/TimeFunction.hpp>

/** 
 * @brief   An abstract class for general models that can be simulated.
 */
template <size_t Nx, size_t Nu, size_t Ny>
class Model {
  public:
    typedef ColVector<Nx> VecX_t;
    typedef ColVector<Nu> VecU_t;
    typedef ColVector<Ny> VecY_t;
    typedef ColVector<Ny> VecR_t;
    typedef TimeFunctionT<VecU_t> InputFunction;
    typedef TimeFunctionT<VecR_t> ReferenceFunction;
    typedef ODEResultX<VecX_t> SimulationResult;

    struct ObserverSimulationResult : public SimulationResult {
        std::vector<double> sampledTime;
        std::vector<VecX_t> estimatedSolution;
    };

    /**
     * @brief   Get the state change of the model, given current state
     *          @f$ x @f$ and input @f$ u @f$.
     */
    virtual VecX_t operator()(const VecX_t &x, const VecU_t &u) = 0;

    /** 
     * @brief   Get the output of the model, given the current state
     *          @f$ x @f$ and input @f$ u @f$.
     */
    virtual VecY_t getOutput(const VecX_t &x, const VecU_t &u) = 0;

    /** 
     * @brief   Simulate the model.
     * 
     * @param   u 
     *          The input function that can be evaluated at any time t.
     * @param   x_start
     *          The initial state of the model.
     * @param   opt
     *          Options for the ODE solver.
     * 
     * @return  A struct containing a vector of times and a vector of states 
     *          for each point of the simulation. It also contains a result code
     *          to check if the simulation was successfull.
     * 
     * @todo    Move this function to the Continuous case?
     */
    virtual SimulationResult simulate(InputFunction &u, VecX_t x_start,
                                      const AdaptiveODEOptions &opt) = 0;

    /** 
     * @brief   Simulate the model.
     * 
     * @param   timeresult
     *          A `back_insert_iterator` to store the time points of 
     *          the simulation in.
     * @param   xresult
     *          A `back_insert_iterator` to store the states of the simulation
     *          in.
     * @param   u 
     *          The input function that can be evaluated at any time t.
     * @param   x_start
     *          The initial state of the model.
     * @param   opt
     *          Options for the ODE solver.
     * 
     * @return  A result code to check if the simulation was successfull.
     * 
     * @todo    Move this function to the Continuous case?
     */
    virtual ODEResultCode
    simulate(typename std::back_insert_iterator<std::vector<double>> timeresult,
             typename std::back_insert_iterator<std::vector<VecX_t>> xresult,
             InputFunction &u, VecX_t x_start,
             const AdaptiveODEOptions &opt) = 0;

    /** 
     * @brief   Simulate the model.
     * 
     * @param   u 
     *          A constant input value that will be kept constant during the 
     *          simulation.
     * @param   x_start
     *          The initial state of the model.
     * @param   opt
     *          Options for the ODE solver.
     * 
     * @return  A struct containing a vector of times and a vector of states 
     *          for each point of the simulation. It also contains a result code
     *          to check if the simulation was successfull.
     * 
     * @todo    Move this function to the Continuous case?
     */
    virtual SimulationResult simulate(VecU_t u, VecX_t x_start,
                                      const AdaptiveODEOptions &opt) {
        ConstantTimeFunctionT<VecU_t> fu = {u};
        return simulate(fu, x_start, opt);
    }

    /** 
     * @brief   Simulate the model.
     * 
     * @param   timeresult
     *          A `back_insert_iterator` to store the time points of 
     *          the simulation in.
     * @param   xresult
     *          A `back_insert_iterator` to store the states of the simulation
     *          in.
     * @param   u 
     *          A constant input value that will be kept constant during the 
     *          simulation.
     * @param   x_start
     *          The initial state of the model.
     * @param   opt
     *          Options for the ODE solver.
     * 
     * @return  A result code to check if the simulation was successfull.
     * 
     * @todo    Move this function to the Continuous case?
     */
    virtual ODEResultCode
    simulate(typename std::back_insert_iterator<std::vector<double>> timeresult,
             typename std::back_insert_iterator<std::vector<VecX_t>> xresult,
             VecU_t u, VecX_t x_start, const AdaptiveODEOptions &opt) {
        ConstantTimeFunctionT<VecU_t> fu = {u};
        return simulate(timeresult, xresult, fu, x_start, opt);
    }

    /** 
     * @brief   Simulate the model with the given continuous controller.
     */
    virtual SimulationResult
    simulate(ContinuousController<Nx, Nu, Ny> &controller, ReferenceFunction &r,
             VecX_t x_start, const AdaptiveODEOptions &opt) = 0;

    /** 
     * @brief   Simulate the model with the given discrete controller.
     */
    virtual SimulationResult
    simulate(DiscreteController<Nx, Nu, Ny> &controller, ReferenceFunction &r,
             VecX_t x_start, const AdaptiveODEOptions &opt) = 0;

    /** 
     * @brief   Simulate the model with the given discrete controller and 
     *          observer.
     */
    virtual ObserverSimulationResult
    simulate(DiscreteController<Nx, Nu, Ny> &controller,
             Kalman<Nx, Nu, Ny> &observer, TimeFunctionT<VecU_t> &randFnW,
             TimeFunctionT<VecY_t> &randFnV, ReferenceFunction &r,
             VecX_t x_start, const AdaptiveODEOptions &opt) = 0;
};

/** 
 * @brief   An abstract class for Continuous-Time models.
 */
template <size_t Nx, size_t Nu, size_t Ny>
class ContinuousModel : public Model<Nx, Nu, Ny> {
  public:
    using VecX_t            = typename Model<Nx, Nu, Ny>::VecX_t;
    using VecU_t            = typename Model<Nx, Nu, Ny>::VecU_t;
    using VecY_t            = typename Model<Nx, Nu, Ny>::VecY_t;
    using VecR_t            = typename Model<Nx, Nu, Ny>::VecR_t;
    using InputFunction     = typename Model<Nx, Nu, Ny>::InputFunction;
    using ReferenceFunction = typename Model<Nx, Nu, Ny>::ReferenceFunction;
    using SimulationResult  = typename Model<Nx, Nu, Ny>::SimulationResult;
    using ObserverSimulationResult =
        typename Model<Nx, Nu, Ny>::ObserverSimulationResult;

    SimulationResult simulate(InputFunction &u, VecX_t x_start,
                              const AdaptiveODEOptions &opt) override {
        auto f = [this, &u](double t, const VecX_t &x) {
            ContinuousModel &model = *this;
            return model(x, u(t));
        };
        return dormandPrince(f, x_start, opt);
    }

    ODEResultCode
    simulate(typename std::back_insert_iterator<std::vector<double>> timeresult,
             typename std::back_insert_iterator<std::vector<VecX_t>> xresult,
             InputFunction &u, VecX_t x_start,
             const AdaptiveODEOptions &opt) override {
        auto f = [this, &u](double t, const VecX_t &x) {
            ContinuousModel &model = *this;
            return model(x, u(t));
        };
        return dormandPrince(timeresult, xresult, f, x_start, opt);
    }

    SimulationResult simulate(VecU_t u, VecX_t x_start,
                              const AdaptiveODEOptions &opt) override {
        ConstantTimeFunctionT<VecU_t> fu = {u};
        return simulate(fu, x_start, opt);
    }

    ODEResultCode
    simulate(typename std::back_insert_iterator<std::vector<double>> timeresult,
             typename std::back_insert_iterator<std::vector<VecX_t>> xresult,
             VecU_t u, VecX_t x_start, const AdaptiveODEOptions &opt) override {
        ConstantTimeFunctionT<VecU_t> fu = {u};
        return simulate(timeresult, xresult, fu, x_start, opt);
    }

    SimulationResult simulate(ContinuousController<Nx, Nu, Ny> &controller,
                              ReferenceFunction &r, VecX_t x_start,
                              const AdaptiveODEOptions &opt) override {
        auto f = [this, &controller, &r](double t, const VecX_t &x) {
            ContinuousModel &model = *this;
            return model(x, controller(x, r(t)));
        };
        return dormandPrince(f, x_start, opt);
    }

    SimulationResult simulate(DiscreteController<Nx, Nu, Ny> &controller,
                              ReferenceFunction &r, VecX_t x_start,
                              const AdaptiveODEOptions &opt) override {
        SimulationResult result     = {};
        double Ts                   = controller.Ts;
        size_t N                    = floor((opt.t_end - opt.t_start) / Ts) + 1;
        VecX_t curr_x               = x_start;
        AdaptiveODEOptions curr_opt = opt;
        for (size_t i = 0; i < N; ++i) {
            double t         = opt.t_start + Ts * i;
            curr_opt.t_start = t;
            curr_opt.t_end   = t + Ts;
            VecR_t curr_ref  = r(t);
            VecU_t curr_u    = controller(curr_x, curr_ref);
            result.resultCode |= simulate(std::back_inserter(result.time),
                                          std::back_inserter(result.solution),
                                          curr_u, curr_x, curr_opt);
            curr_x = result.solution.back();
            result.time.pop_back();
            result.solution.pop_back();
        }
        return result;
    }

    ObserverSimulationResult
    simulate(DiscreteController<Nx, Nu, Ny> &controller,
             Kalman<Nx, Nu, Ny> &observer, TimeFunctionT<VecU_t> &randFnW,
             TimeFunctionT<VecY_t> &randFnV, ReferenceFunction &r,
             VecX_t x_start, const AdaptiveODEOptions &opt) override {
        ObserverSimulationResult result = {};
        assert(controller.Ts == observer.Ts);
        double Ts = controller.Ts;
        size_t N  = numberOfSamplesInTimeRange(opt.t_start, Ts, opt.t_end);
        result.sampledTime.reserve(N);
        result.estimatedSolution.reserve(N);
        VecX_t curr_x               = x_start;
        VecX_t curr_x_hat           = x_start;
        AdaptiveODEOptions curr_opt = opt;
        for (size_t i = 0; i < N; ++i) {
            double t         = opt.t_start + Ts * i;
            curr_opt.t_start = t;
            curr_opt.t_end   = t + Ts;
            result.sampledTime.push_back(t);
            result.estimatedSolution.push_back(curr_x_hat);
            auto w          = randFnW(t);
            auto v          = randFnV(t);
            VecR_t curr_ref = r(t);
            VecU_t curr_u   = controller(curr_x_hat, curr_ref);
            result.resultCode |= simulate(std::back_inserter(result.time),
                                          std::back_inserter(result.solution),
                                          curr_u + w, curr_x, curr_opt);
            curr_x     = result.solution.back();
            VecY_t y   = this->getOutput(curr_x, curr_u) + v;
            curr_x_hat = observer.getStateChange(curr_x_hat, y, curr_u);
            result.time.pop_back();
            result.solution.pop_back();
        }
        return result;
    }
};

#include <Model/System.hpp>

/** 
 * @brief   Continuous-Time Linear Time-Invariant Model.
 */
template <size_t Nx, size_t Nu, size_t Ny>
class CTLTIModel : public ContinuousModel<Nx, Nu, Ny>,
                   public CTLTISystem<Nx, Nu, Ny> {
  public:
    using VecX_t = typename ContinuousModel<Nx, Nu, Ny>::VecX_t;
    using VecU_t = typename ContinuousModel<Nx, Nu, Ny>::VecU_t;
    using VecY_t = typename ContinuousModel<Nx, Nu, Ny>::VecY_t;
    using VecR_t = typename ContinuousModel<Nx, Nu, Ny>::VecR_t;

    CTLTIModel(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
               const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D)
        : CTLTISystem<Nx, Nu, Ny>{A, B, C, D} {}

    CTLTIModel(const CTLTISystem<Nx, Nu, Ny> &sys)
        : CTLTISystem<Nx, Nu, Ny>{sys} {}

    VecX_t operator()(const VecX_t &x, const VecU_t &u) override {
        return this->getStateChange(x, u);
    }

    VecY_t getOutput(const VecX_t &x, const VecU_t &u) override {
        return this->getSystemOutput(x, u);
    }
};