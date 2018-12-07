#include <attitude-controller.h>
#include <altitude-controller.h>
#include <Drone/LQRController.hpp>

namespace Attitude {
class CLQRController : public DiscreteController<Nx, Nu, Ny> {
  public:
    CLQRController(double Ts) : DiscreteController<Nx, Nu, Ny>{Ts} {}
    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &ref) {
        auto xbias = vcat(x, zeros<3, 1>());
        AttitudeState xa;
        AttitudeReference ra;
        copyToCArray(xa, xbias);
        Quaternion rq = DroneAttitudeOutput{ref}.getOrientation();
        copyToCArray(ra, rq);
        AttitudeControl ua;
        getAttitudeControllerOutput(xa, ra, ua);
        VecU_t u;
        copyFromCArray(u, ua);
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
        AltitudeState xa;
        AltitudeReference ra;
        copyToCArray(xa, x);
        copyToCArray(ra, ref);
        AltitudeOutput ua;
        getAltitudeControllerOutput(xa, ra, ua);
        VecU_t u;
        copyFromCArray(u, ua);
        return u;
    }
};
}  // namespace Altitude