#pragma once

#include <DroneLogLoader.hpp>
#include <Drone.hpp>
#include <ANSIColors.hpp>

struct DronePlottable {
    DronePlottable(const Drone::ControllerSimulationResult &d)
        : time{d.time}, sampledTime{d.sampledTime}, states{d.solution},
          control{d.control}, reference{d.reference} {}
    DronePlottable(const DroneLogLoader &d)
        : time{d.getTimeStamps()}, sampledTime{time}, states{d.getStates()},
          control{d.getControl()}, reference{d.getReference()} {}
    DronePlottable() = default;

    std::vector<double> time;
    std::vector<double> sampledTime;
    std::vector<Drone::VecX_t> states;
    std::vector<Drone::VecU_t> control;
    std::vector<Drone::VecR_t> reference;
};

template <class T, size_t RS, size_t RP>
void plotDroneSignal(const std::vector<double> &t,
                     const std::vector<ColVector<RS>> &signal,
                     ColVector<RP> (T::*extractor)() const,
                     const std::vector<std::string> &legends = {},
                     const std::vector<std::string> &formats = {},
                     const std::vector<std::string> &colors  = {},
                     const std::string &title                = "") {
    /* TODO */
}

void plotDrone(const DronePlottable &result,
               const std::string &legendSuffix = "", bool plotReference = true);