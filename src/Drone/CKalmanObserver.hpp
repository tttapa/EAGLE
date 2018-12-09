#include "KalmanObserver.hpp"
#include <altitude-controller.h>
#include <attitude-controller.h>

namespace Attitude {
class CKalmanObserver : public DiscreteObserver<Nx, Nu, Ny> {
  public:
    CKalmanObserver(double Ts) : DiscreteObserver<Nx, Nu, Ny>{Ts} {}
    VecX_t getStateChange(const VecX_t &x_hat, const VecY_t &y_sensor,
                          const VecU_t &u) override {
        VecX_t x_hat_cpy = x_hat;
        updateAttitudeKFEstimate(toCppArray(x_hat_cpy), toCppArray(u),
                                 toCppArray(y_sensor));
        return x_hat_cpy;
    }
};

}  // namespace Attitude

namespace Altitude {
class CKalmanObserver : public DiscreteObserver<Nx, Nu, Ny> {
  public:
    CKalmanObserver(double Ts) : DiscreteObserver<Nx, Nu, Ny>{Ts} {}
    VecX_t getStateChange(const VecX_t &x_hat, const VecY_t &y_sensor,
                          const VecU_t &u) override {
        VecX_t x_hat_cpy = x_hat;
        updateAltitudeKFEstimate(toCppArray(x_hat_cpy), toCppArray(u),
                                 toCppArray(y_sensor));
        return x_hat_cpy;
    }
};

}  // namespace Altitude
