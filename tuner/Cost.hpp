#pragma once

#include <CostWeights.hpp>
#include <Drone.hpp>
#include <StepResponseAnalyzer.hpp>
#include <Degrees.hpp>

namespace CostReferences {
constexpr Quaternion qz                   = eul2quat({22.5_deg, 0, 0});
constexpr Quaternion qy                   = eul2quat({0, 22.5_deg, 0});
constexpr Quaternion qx                   = eul2quat({0, 0, 22.5_deg});
constexpr Quaternion qz3                  = eul2quat({30_deg, 0, 0});
constexpr Quaternion qy3                  = eul2quat({0, 30_deg, 0});
constexpr Quaternion qx3                  = eul2quat({0, 0, 30_deg});
constexpr Array<Quaternion, 7> references = {{
    quatmultiply(qx, quatmultiply(qy, qz)),
    quatmultiply(qx, qy),
    qx,
    qz,
    quatmultiply(qx3, qy3),
    qx3,
    qz3,
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