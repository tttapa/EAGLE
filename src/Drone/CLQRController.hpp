extern "C" {
#include <attitude.h>
#include <controllers.h>
};

#include <Drone/LQRController.hpp>

namespace Attitude {
class CLQRController : public DiscreteController<Nx, Nu, Ny> {
  public:
    CLQRController(double Ts) : DiscreteController<Nx, Nu, Ny>{Ts} {}
    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &ref) {
        const ColVector<13> x_bias = vcat(x, zeros<3, 1>());
        copyToCArray(att_hat, x_bias);
        copyToCArray(att_ref, ref);
        updateAttitudeController();
        VecU_t u;
        copyFromCArray(u, att_u);
        return u;
    }
};
}  // namespace Attitude