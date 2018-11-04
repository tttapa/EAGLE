#pragma once

#include "../MotorControl.hpp"
#include <Model/Controller.hpp>
#include <Model/System.hpp>

#include "attitude.h"

/**
 * @brief   A class for the LQR attitude controller for the drone.
 *          It uses the generated C code.
 * 
 * @note    This class is specifically for the attitude controller, as it uses
 *          Hamiltonian quaternion multiplication for the difference of the
 *          first four states.
 */
class GeneratedLQRController : public DiscreteController<10, 3, 7> {
  public:
    /** Number of states. */
    static constexpr size_t nx = 10;
    /** Number of inputs. */
    static constexpr size_t nu = 3;
    /** Number of outputs. */
    static constexpr size_t ny = 7;

    GeneratedLQRController() : DiscreteController<10, 3, 7>{1.0 / 238} {}

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &r) {
        ColVector<nu> u;
        getControllerOutput(toArrayPointer(x) + 1, toArrayPointer(r),
                            toArrayPointer(u));
        return u;
    }
};

class ClampedGeneratedLQRController : public GeneratedLQRController {
  public:
    ClampedGeneratedLQRController(double u_h) : u_h(u_h) {}

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        auto u = getRawControllerOutput(x, r);
        u      = clampMotorControlSignal(u, u_h);
        return u;
    }

  private:
    const double u_h;
};