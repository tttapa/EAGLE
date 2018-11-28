#pragma once

#include <CostWeights.hpp>
#include <Drone/Drone.hpp>
#include <StepResponseAnalyzer.hpp>
#include <Util/Degrees.hpp>

namespace CostReferences {
constexpr Quaternion qz                   = eul2quat({10_deg, 0, 0});
constexpr Quaternion qy                   = eul2quat({0, 10_deg, 0});
constexpr Quaternion qx                   = eul2quat({0, 0, 10_deg});
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