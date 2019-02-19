#pragma once

#ifdef I
#error IIIIIIIIIIIIIIIIIII
#endif
#include <DronePlot.hpp>
#ifdef I
#error IIIIIIIIIIIIIIIIIII
#endif
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
 * @return  The Matplotlib Figure
 */
pybind11::object plot(const DronePlottable &result, float w = 1920,
                      float h = 1080, int colors = 0, std::string title = "");

/**
 * @brief   This function plots the reference, control signal, and states of the
 *          attitude model of the drone.
 * 
 * It uses Matplotlib and Python to display the result.
 * 
 * @note    Expects the Python interpreter to be active.
 * 
 * @param   result 
 *          The result of a simulation to plot.
 * @return  The Matplotlib Figure
 */
pybind11::object plot(const DroneAttitudePlottable &result, float w = 1920,
                      float h = 1080, int colors = 0, std::string title = "");

/**
 * @brief   Show the given Matplotlib figure.
 * 
 * @note    Expects the Python interpreter to be active.
 */
void show(pybind11::object fig);

/**
 * @brief   Show the active Matplotlib figure.
 * 
 * @note    Expects the Python interpreter to be active.
 */
void show();

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

/** 
 * @brief   Create a new figure.
 */
pybind11::object figure(float w = 1920, float h = 1080);

/**
 * @brief   Create a new figure and axes.
 */
pybind11::object axes(float w = 1920, float h = 1080);

/** 
 * @brief   Close the given figure
 */
void close(pybind11::object fig);