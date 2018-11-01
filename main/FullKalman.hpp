#pragma once

#include <Model/Kalman.hpp>
#include <Quaternions/ReducedQuaternion.hpp>

template <size_t Nx, size_t Nu, size_t Ny>
class FullKalman : public DiscreteObserver<Nx, Nu, Ny> {
  public:
    FullKalman(const Matrix<Nx - 1, Nx - 1> &A, const Matrix<Nx - 1, Nu> &B,
                  const Matrix<Ny - 1, Nx - 1> &C,
                  const Matrix<Nx - 1, Ny - 1> &L, double Ts)
        : DiscreteObserver<Nx, Nu, Ny>{Ts},  //
          Ak{A - L * C},                     //
          Bk{hcat(L, B)}                     //
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
        ColVector<Ny - 1 + Nu> yk   = vcat(getBlock<1, Ny, 0, 1>(y_sensor), u);
        ColVector<Nx - 1> x_hat_red = getBlock<1, Nx, 0, 1>(x_hat);
        x_hat_red                   = Ak * x_hat_red + Bk * yk;
        ColVector<Nx> x_hat_new     = vcat(zeros<1, 1>(), x_hat_red);
        assignBlock<0, 4, 0, 1>(x_hat_new) =
            red2quat(getBlock<0, 3, 0, 1>(x_hat_red));
        return x_hat_new;
    }

    const Matrix<Nx - 1, Nx - 1> Ak;
    const Matrix<Nx - 1, Ny - 1 + Nu> Bk;
};