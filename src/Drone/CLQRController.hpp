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
        const auto xbias = vcat(x, zeros<3, 1>());
        Quaternion rq    = DroneAttitudeOutput{ref}.getOrientation();
        VecU_t u;
        getAttitudeControllerOutput(toArrayPointer(xbias), toArrayPointer(rq),
                                    toArrayPointer(u));
        return u;
    }
};
}  // namespace Attitude

namespace Altitude {
class CLQRController : public DiscreteController<Nx, Nu, Ny> {
  public:
    CLQRController(double Ts) : DiscreteController<Nx, Nu, Ny>{Ts} {}
    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &ref) {
        VecU_t u;
        getAltitudeControllerOutput(toArrayPointer(x), toArrayPointer(ref),
                                    toArrayPointer(u));
        return u;
    }
};
}  // namespace Altitude