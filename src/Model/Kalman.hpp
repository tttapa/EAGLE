#pragma once

#include "System.hpp"

template <size_t Nx, size_t Nu, size_t Ny>
class DiscreteObserver {
  public:
    DiscreteObserver(double Ts) : Ts{Ts} {}

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
    virtual ColVector<Nx> getStateChange(const ColVector<Nx> &x_hat,
                                         const ColVector<Ny> &y_sensor,
                                         const ColVector<Nu> &u) = 0;

    const double Ts;
};