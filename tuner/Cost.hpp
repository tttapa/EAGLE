#pragma once

#include <CostWeights.hpp>
#include <Drone/Drone.hpp>
#include <StepResponseAnalyzer.hpp>

namespace CostReferences {
constexpr Quaternion qx                   = eul2quat({0, 0, M_PI / 8});
constexpr Quaternion qy                   = eul2quat({0, M_PI / 8, 0});
constexpr Quaternion qz                   = eul2quat({M_PI / 8, 0, 0});
constexpr Array<Quaternion, 5> references = {{
    quatmultiply(qx, quatmultiply(qy, qz)),
    quatmultiply(qx, qy),
    qx,
    qy,
    qz,
}};
}  // namespace CostReferences

double getRiseTimeCost(Drone::FixedClampAttitudeController &attctrl,
                       Drone::AttitudeModel &attmodel, Quaternion q_ref,
                       double factor, const DroneAttitudeState &attx0,
                       const AdaptiveODEOptions &opt, const CostWeights &cost);

double getCost(Drone::FixedClampAttitudeController &attctrl,
               Drone::AttitudeModel &attmodel, double errorfactor,
               const DroneAttitudeState &attx0, const AdaptiveODEOptions &opt,
               const CostWeights &cost);