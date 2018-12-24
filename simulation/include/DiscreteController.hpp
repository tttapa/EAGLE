#pragma once

#include <Matrix.hpp>
#include <TimeFunction.hpp>

/**
 * @brief   An abstract class for discrete-time controllers.
 *          A controller produces a control signal @f$ u @f$ given a state 
 *          @f$ x @f$ and a reference target @f$ r @f$.
 */
template <size_t Nx, size_t Nu, size_t Nr>
class DiscreteController {
  public:
    typedef ColVector<Nx> VecX_t;  // state vectors
    typedef ColVector<Nu> VecU_t;  // input vectors
    typedef ColVector<Nr> VecR_t;  // reference vectors
    typedef TimeFunctionT<VecR_t> ReferenceFunction;

    DiscreteController(double Ts) : Ts(Ts) {}
    virtual ~DiscreteController() = default;

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

    virtual void reset() {}

    const double Ts;
};
