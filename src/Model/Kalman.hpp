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

/**
 * @brief   Discrete-Time Kalman filter.
 * 
 * @f$ x_{k+1} = \left(A - L C\right) x_k + 
 *     \begin{pmatrix} L & B \end{pmatrix} 
 *     \begin{pmatrix} y_k \\ u_k \end{pmatrix} @f$
 */
template <size_t Nx, size_t Nu, size_t Ny>
class Kalman : public DiscreteObserver<Nx, Nu, Ny> {
  public:
    Kalman(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
           const Matrix<Ny, Nx> &C, const Matrix<Nx, Ny> &L, double Ts)
        : Ak{A - L * C},                    //
          Bk{hcat(L, B)},                   //
          DiscreteObserver<Nx, Nu, Ny>{Ts}  //
    {}

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
    ColVector<Nx> getStateChange(const ColVector<Nx> &x_hat,
                                 const ColVector<Ny> &y_sensor,
                                 const ColVector<Nu> &u) override {
        ColVector<Ny + Nu> yk = vcat(y_sensor, u);
        return Ak * x_hat + Bk * yk;
    }

    const Matrix<Nx, Nx> Ak;
    const Matrix<Nx, Ny + Nu> Bk;
};