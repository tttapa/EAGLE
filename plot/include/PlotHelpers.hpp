#pragma once

#include <pybind11/embed.h>

#include <DronePlot.hpp>

/**
 * @brief   Load the Plot.py Python script and its functions.
 *          The module will be available in Python as a global variable `plot`.
 * 
 * @note    Expects the Python interpreter to be active.
 */
void loadPythonPlotModule();

/**
 * @brief   Load the Plot.py Python script and its functions, and return it.
 * 
 * @note    Expects the Python interpreter to be active.
 */
pybind11::module getPythonPlotModule();

/**
 * @brief   Convert a DronePlottable (a collection of an array of state vectors,
 *          an array of control signal vectors and an array of reference 
 *          vectors) to a Python dictionary (associative array).
 * 
 * @note    Expects the Python interpreter to be active.
 */
pybind11::dict dronePlottableToPythonDict(const DronePlottable &result);

/**
 * @brief   Convert a DroneAttitudePlottable (a collection of an array of state
 *          vectors, an array of control signal vectors and an array of 
 *          reference vectors) to a Python dictionary (associative array).
 * 
 * @note    Expects the Python interpreter to be active.
 */
pybind11::dict
droneAttitudePlottableToPythonDict(const DroneAttitudePlottable &result);