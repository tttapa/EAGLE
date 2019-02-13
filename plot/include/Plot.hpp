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


/**
 * @brief   Show the given Matplotlib figure.
 * 
 * @note    Expects the Python interpreter to be active.
 */
void show(pybind11::object fig);

/**
 * @brief   Save the given Matplotlib figure.
 * 
 * @note    Expects the Python interpreter to be active.
 * 
 * @param   fig
 *          The Matplotlib.pyplot.Figure figure to save
 * @param   filename
 *          The path and filename to save it to.
 */

void save(pybind11::object fig, std::filesystem::path filename);