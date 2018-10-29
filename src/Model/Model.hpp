#pragma once

#include "Params.hpp"
#include <ODE/DormandPrince.hpp>
#include <Util/TimeFunction.hpp>

/** 
 * @brief   An abstract class for general models that can be simulated.
 */
template <size_t Nx, size_t Nu>
class Model {
  public:
    typedef ColVector<Nx> VecX_t;
    typedef ColVector<Nu> VecU_t;
    typedef TimeFunctionT<VecU_t> InputFunction;
    typedef ODEResultX<VecX_t> SimulationResult;

    /**
     * @brief   Get the state change of the model, given current state @
     *          f$ x @f$ and input @f$ u @f$.
     */
    virtual VecX_t operator()(const VecX_t &x, const VecU_t &u) = 0;

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
};

/** 
 * @brief   An abstract class for Continuous-Time models.
 */
template <size_t Nx, size_t Nu>
class ContinuousModel : public Model<Nx, Nu> {
  public:
    using VecX_t           = typename Model<Nx, Nu>::VecX_t;
    using VecU_t           = typename Model<Nx, Nu>::VecU_t;
    using InputFunction    = typename Model<Nx, Nu>::InputFunction;
    using SimulationResult = typename Model<Nx, Nu>::SimulationResult;

    SimulationResult simulate(InputFunction &u, VecX_t x_start,
                              const AdaptiveODEOptions &opt) override {
        auto f = [this, &u](double t, const VecX_t &x) {
            ContinuousModel<Nx, Nu> &model = *this;
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
};

#include <Model/System.hpp>

/** 
 * @brief   Continuous-Time Linear Time-Invariant Model.
 */
template <size_t Nx, size_t Nu, size_t Ny>
class CTLTIModel : public ContinuousModel<Nx, Nu>,
                   public CTLTISystem<Nx, Nu, Ny> {
  public:
    using VecX_t = typename CTLTISystem<Nx, Nu, Ny>::VecX_t;
    using VecU_t = typename CTLTISystem<Nx, Nu, Ny>::VecU_t;
    using VecY_t = typename CTLTISystem<Nx, Nu, Ny>::VecY_t;

    CTLTIModel(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
               const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D)
        : CTLTISystem<Nx, Nu, Ny>{A, B, C, D} {}

    CTLTIModel(const CTLTISystem<Nx, Nu, Ny> &sys)
        : CTLTISystem<Nx, Nu, Ny>{sys} {}

    VecX_t operator()(const VecX_t &x, const VecU_t &u) override {
        return this->getStateChange(x, u);
    }
};