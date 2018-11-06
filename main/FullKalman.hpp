#pragma once

#include <Model/Kalman.hpp>
#include <Quaternions/QuaternionStateAddSub.hpp>
#include <Quaternions/ReducedQuaternion.hpp>

template <size_t Nx, size_t Nu, size_t Ny>
class FullKalman : public DiscreteObserver<Nx, Nu, Ny> {
  public:
    FullKalman(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
               const Matrix<Ny, Nx> &C, const Matrix<Nx - 1, Ny - 1> &L,
               double Ts)
        : DiscreteObserver<Nx, Nu, Ny>{Ts}, A{A}, B{B}, C{C}, L{L} {}

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
        ColVector<Ny> cx = C * x_hat;

        ColVector<Ny> ydiff = quaternionStatesSub(y_sensor, cx);

        ColVector<Ny - 1> ydiff_red = getBlock<1, Ny, 0, 1>(ydiff);
        ColVector<Nx - 1> Ly_red    = L * ydiff_red;
        ColVector<Nx> Ly            = red2quat(Ly_red);

        ColVector<Nx> x_hat_states = A * x_hat + B * u;
        ColVector<Nx> x_hat_new    = quaternionStatesAdd(x_hat_states, Ly);
        return x_hat_new;
    }

    const Matrix<Nx, Nx> A;
    const Matrix<Nx, Nu> B;
    const Matrix<Ny, Nx> C;
    const Matrix<Nx - 1, Ny - 1> L;
};