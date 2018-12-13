#include <Drone/LQRController.hpp>
#include <altitude-controller.h>
#include <attitude-controller.h>

namespace Attitude {
class CLQRController : public DiscreteController<Nx, Nu, Ny> {
  public:
    CLQRController(double Ts) : DiscreteController<Nx, Nu, Ny>{Ts} {}
    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &ref) {
        Quaternion rq = DroneAttitudeOutput{ref}.getOrientation();
        VecU_t u;
        getAttitudeControllerOutput(toCppArray(x), toCppArray(rq),
                                    toCppArray(u), 1);
        return u;
    }
};
}  // namespace Attitude

namespace Altitude {
class CLQRController : public DiscreteController<Nx, Nu, Ny> {
  public:
    CLQRController(double Ts) : DiscreteController<Nx, Nu, Ny>{Ts} {}

    void reset() override { integral = {}; }

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &ref) {
        VecU_t u;
        getAltitudeControllerOutput(toCppArray(x), toCppArray(ref),
                                    toCppArray(u), toCppArray(integral), 1);
        return u;
    }

  private:
    VecR_t integral = {};
};
}  // namespace Altitude