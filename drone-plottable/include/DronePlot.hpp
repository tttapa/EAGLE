#pragma once

#include <Drone.hpp>
#include <DroneLogLoader.hpp>

struct DroneAttitudePlottable {
    DroneAttitudePlottable(
        const Drone::AttitudeModel::ControllerSimulationResult &d)
        : time{d.time}, sampledTime{d.sampledTime}, states{d.solution},
          control{d.control}, reference{d.reference} {}

    std::vector<double> time;
    std::vector<double> sampledTime;
    std::vector<Drone::AttitudeModel::VecX_t> states;
    std::vector<Drone::AttitudeModel::VecU_t> control;
    std::vector<Drone::AttitudeModel::VecR_t> reference;
};

struct DronePlottable {
    DronePlottable(const Drone::ControllerSimulationResult &d)
        : time{d.time}, sampledTime{d.sampledTime}, states{d.solution},
          control{d.control}, reference{d.reference} {}
    DronePlottable(const Drone::ObserverControllerSimulationResult &d)
        : time{d.time}, sampledTime{d.sampledTime}, states{d.solution},
          stateEstimates{d.estimatedSolution}, control{d.control},
          reference{d.reference} {}

    DronePlottable(const DroneLogLoader &d)
        : time{d.getTimeStamps()}, sampledTime{time}, states{d.getStates()},
          control{d.getControl()}, reference{d.getReference()} {}

    DronePlottable() = default;

    std::vector<double> time;
    std::vector<double> sampledTime;
    std::vector<Drone::VecX_t> states;
    std::vector<Drone::VecX_t> stateEstimates;
    std::vector<Drone::VecU_t> control;
    std::vector<Drone::VecR_t> reference;
};