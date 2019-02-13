#pragma once

#include <DronePlot.hpp>
#include <pybind11/pytypes.h>

/**
 * @brief   This function plots the reference, control signal, and states of the
 *          drone.
 * 
 * It uses Matplotlib and Python to display the result.
 * 
 * @note    Expects the Python interpreter to be active.
 * 
 * @param   result 
 *          The result of a simulation to plot.
 */
pybind11::object plot(const DronePlottable &result, float w = 16, float h = 9);

void show(pybind11::object fig);

void save(pybind11::object fig, std::filesystem::path filename);