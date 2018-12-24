#include <LQRController.hpp>

namespace Attitude {
class CLQRController : public DiscreteController<Nx, Nu, Ny> {
  public:
    CLQRController(double Ts) : DiscreteController<Nx, Nu, Ny>{Ts} {}
    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &ref);
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

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &ref);

  private:
    VecR_t integral = {};
};
}  // namespace Altitude