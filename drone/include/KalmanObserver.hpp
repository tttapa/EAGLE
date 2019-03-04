#pragma once

#include "Def.hpp"
#include <DiscreteObserver.hpp>
#include <QuaternionStateAddSub.hpp>
#include <ReducedQuaternion.hpp>

namespace Attitude {

/**
 * @brief   The discrete Kalman filter/observer for the attitude controller.
 */
class KalmanObserver : public DiscreteObserver<Nx, Nu, Ny> {
  public:
    KalmanObserver(const Matrix<Nx - 1, Nx - 1> &A_red,
                   const Matrix<Nx - 1, Nu> &B_red, const Matrix<Ny, Nx> &C,

                   const Matrix<Nx - 1, Ny - 1> &L, double Ts)
        : DiscreteObserver<Nx, Nu, Ny>{Ts}, A_red{A_red}, B_red{B_red}, C{C},
          L{L} {}

    /**
     * @brief   Get the state change, given the previous estimated state, the
     *          current sensor reading, and the current control input.
     * 
     * Calculates   @f$ 
     *                  \hat{x}_{k+1} = \left(A \hat{x}_k + B u_k\right) \oplus 
     *                  L \left(y_k \ominus C \hat{x}_k\right)
     *              @f$
     * 
     * The subtraction and addition are implemented as quaternion 
     * multiplications.
     * 
     * @param   x_hat
     *          The previous estimated state.
     * @param   y_sensor
     *          The current sensor reading.
     * @param   u
     *          The current control input. 
     */
    VecX_t getStateChange(const VecX_t &x_hat, const VecY_t &y_sensor,
                          const VecU_t &u) override {
        VecY_t cx = C * x_hat;

        VecY_t ydiff = quaternionStatesSub(y_sensor, cx);

        ColVector<Ny - 1> ydiff_red   = getBlock<1, Ny, 0, 1>(ydiff);
        ColVector<Nx - 1> Ly_diff_red = L * ydiff_red;
        VecX_t Ly_diff                = red2quat(Ly_diff_red);

        ColVector<Nx - 1> x_hat_red       = getBlock<1, Nx, 0, 1>(x_hat);
        ColVector<Nx - 1> x_hat_model_red = A_red * x_hat_red + B_red * u;
        VecX_t x_hat_model                = red2quat(x_hat_model_red);
        VecX_t x_hat_new = quaternionStatesAdd(x_hat_model, Ly_diff);
        return x_hat_new;
    }

    const Matrix<Nx - 1, Nx - 1> A_red;
    const Matrix<Nx - 1, Nu> B_red;
    const Matrix<Ny, Nx> C;
    const Matrix<Nx - 1, Ny - 1> L;
};

}  // namespace Attitude

#include <iostream>  // TODO

namespace Altitude {

/**
 * @brief   The discrete Kalman filter/observer for the altitude controller.
 */
class KalmanObserver : public DiscreteObserver<Nx, Nu, Ny> {
  public:
    KalmanObserver(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                   const Matrix<Ny, Nx> &C, const Matrix<Nx, Ny> &L, double Ts)
        : DiscreteObserver<Nx, Nu, Ny>{Ts}, A{A}, B{B}, C{C}, L{L} {
#if 0
        std::cout << "A_alt = ";
        printMATLAB(std::cout, A);
        std::cout << std::endl;
        std::cout << "B_alt = ";
        printMATLAB(std::cout, B);
        std::cout << std::endl;
        std::cout << "C_alt = ";
        printMATLAB(std::cout, C);
        std::cout << std::endl;
        std::cout << "L_alt = ";
        printMATLAB(std::cout, L);
        std::cout << std::endl;
#endif
    }

    /**
     * @brief   Get the state change, given the previous estimated state, the
     *          current sensor reading, and the current control input.
     * 
     * Calculates   @f$ 
     *                  \hat{x}_{k+1} = \left(A \hat{x}_k + B u_k\right) + 
     *                  L \left(y_k - C \hat{x}_k\right)
     *              @f$
     * 
     * @param   x_hat
     *          The previous estimated state.
     * @param   y_sensor
     *          The current sensor reading.
     * @param   u
     *          The current control input. 
     */
    VecX_t getStateChange(const VecX_t &x_hat, const VecY_t &y_sensor,
                          const VecU_t &u) override {
        VecY_t ydiff       = y_sensor - C * x_hat;
        VecX_t x_hat_model = A * x_hat + B * u;
        VecX_t x_hat_new   = x_hat_model + L * ydiff;
        return x_hat_new;
    }

    const Matrix<Nx, Nx> A;
    const Matrix<Nx, Nu> B;
    const Matrix<Ny, Nx> C;
    const Matrix<Nx, Ny> L;
};

}  // namespace Altitude
