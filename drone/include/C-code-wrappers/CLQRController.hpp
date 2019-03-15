#pragma once

#include <LQRController.hpp>

namespace Attitude {
class CLQRController : public DiscreteController<Nx, Nu, Ny> {
  public:
    CLQRController(double Ts, int config = 1, bool enableIntegral = true)
        : DiscreteController<Nx, Nu, Ny>{Ts}, config(config),
          enableIntegral(enableIntegral) {}

    void reset() override { integral = {}; }

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &ref);

    ColVector<3> getIntegral() const { return integral; }

  private:
    ColVector<3> integral = {};
    int config;
    bool enableIntegral;
};
}  // namespace Attitude

namespace Altitude {
class CLQRController : public DiscreteController<Nx, Nu, Ny> {
  public:
    CLQRController(double Ts, int config = 1, bool enableIntegral = true)
        : DiscreteController<Nx, Nu, Ny>{Ts}, config(config),
          enableIntegral(enableIntegral) {}

    void reset() override { integral = {}; }

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        return getRawControllerOutput(x, r);
    }

    VecU_t getRawControllerOutput(const VecX_t &x, const VecR_t &ref);

    VecR_t getIntegral() const { return integral; }

  private:
    VecR_t integral = {};
    int config;
    bool enableIntegral;
};
}  // namespace Altitude