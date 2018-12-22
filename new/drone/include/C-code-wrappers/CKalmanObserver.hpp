#include "KalmanObserver.hpp"

namespace Attitude {
class CKalmanObserver : public DiscreteObserver<Nx, Nu, Ny> {
  public:
    CKalmanObserver(double Ts) : DiscreteObserver<Nx, Nu, Ny>{Ts} {}
    VecX_t getStateChange(const VecX_t &x_hat, const VecY_t &y_sensor,
                          const VecU_t &u) override;
};

}  // namespace Attitude

namespace Altitude {
class CKalmanObserver : public DiscreteObserver<Nx, Nu, Ny> {
  public:
    CKalmanObserver(double Ts) : DiscreteObserver<Nx, Nu, Ny>{Ts} {}
    VecX_t getStateChange(const VecX_t &x_hat, const VecY_t &y_sensor,
                          const VecU_t &u) override;
};

}  // namespace Altitude
