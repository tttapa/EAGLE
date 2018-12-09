#pragma once

#include "System.hpp"

template <size_t Nx, size_t Nu, size_t Ny>
class DiscreteObserver {
  public:
    typedef ColVector<Nx> VecX_t;  // state vectors
    typedef ColVector<Nu> VecU_t;  // input vectors
    typedef ColVector<Ny> VecY_t;  // output vectors

    DiscreteObserver(double Ts) : Ts{Ts} {}

    virtual void reset() {}

    /**
     * @brief   Get the state change, given the previous estimated state, the
     *          current sensor reading, and the current control input.
     * 
     * @param   x_hat
     *          The previous estimated state.
     * @param   y_sensor
     *          The current sensor reading.
     * @param   u
     *          The current control input.
     */
    virtual VecX_t getStateChange(const VecX_t &x_hat, const VecY_t &y_sensor,
                                  const VecU_t &u) = 0;

    const double Ts;
};