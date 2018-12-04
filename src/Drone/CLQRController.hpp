#include <attitude-controller.h>
#include <Drone/LQRController.hpp>

namespace Attitude {
class CLQRController : public DiscreteController<Nx, Nu, Ny> {
  public:
    CLQRController(double Ts) : DiscreteController<Nx, Nu, Ny>{Ts} {}
    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &ref) {
        AttitudeStateX xa;
        RefQuaternion ra;
        copyToCArray(xa, x);
        Quaternion rq = DroneAttitudeOutput{ref}.getOrientation();
        copyToCArray(ra, rq);
        AttitudeControllerOutputU ua;
        getControllerOutput(xa, ra, ua);
        VecU_t u;
        copyFromCArray(u, ua);
        return u;
    }
};
}  // namespace Attitude