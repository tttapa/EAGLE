#pragma once

#include <Drone.hpp>

class DisplayReference
    : public Drone::FixedClampAttitudeController::ReferenceFunction {
  public:
    using VecR_t = Drone::FixedClampAttitudeController::VecR_t;
    DisplayReference();
    VecR_t operator()(double t) override;
    const Quaternion qz;
    const Quaternion qy;
    const Quaternion qx;
    const Quaternion qu = eul2quat({0, 0, 0});
    const double m      = 0.5;
};