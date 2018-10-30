#pragma once

#include <Matrix/LeastSquares.hpp>
#include <Quaternions/Quaternion.hpp>
#include <Util/TimeFunction.hpp>
#include <algorithm>  // transform
#include <cassert>    // assert
#include <vector>

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
};

template <size_t Nx, size_t Nu, size_t Nr>
class ContinuousController : public virtual Controller<Nx, Nu, Nr> {
  public:
    using VecX_t = typename Controller<Nx, Nu, Nr>::VecX_t;
    using VecU_t = typename Controller<Nx, Nu, Nr>::VecU_t;
    using VecR_t = typename Controller<Nx, Nu, Nr>::VecR_t;
    using ReferenceFunction =
        typename Controller<Nx, Nu, Nr>::ReferenceFunction;
};

template <size_t Nx, size_t Nu, size_t Nr>
class DiscreteController : public virtual Controller<Nx, Nu, Nr> {
  public:
    using VecX_t = typename Controller<Nx, Nu, Nr>::VecX_t;
    using VecU_t = typename Controller<Nx, Nu, Nr>::VecU_t;
    using VecR_t = typename Controller<Nx, Nu, Nr>::VecR_t;
    using ReferenceFunction =
        typename Controller<Nx, Nu, Nr>::ReferenceFunction;

    DiscreteController(double Ts) : Ts(Ts) {}

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

    const double Ts;
};
