#pragma once

#include <pybind11/embed.h>

#include <DronePlot.hpp>

/**
 * @brief   Load the Plot.py Python script and its functions, or return the
 *          module that was loaded already.
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