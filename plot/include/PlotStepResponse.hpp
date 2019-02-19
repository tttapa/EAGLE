#pragma once

#include <pybind11/pytypes.h> // pybind11::object

#include <Drone.hpp>

void plotStepResponseAttitude(
    const Drone &drone, const Matrix<Nx_att - 1, Nx_att - 1> &Q,
    const Matrix<Nu_att, Nu_att> &R, double steperrorfactor,
    const Quaternion &q_ref, const AdaptiveODEOptions &opt,
    pybind11::object axes, const std::string &title = "",
    const std::string &legendSuffix = "", int colorset = 0);
