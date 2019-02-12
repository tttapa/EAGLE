#pragma once

#include <DronePlot.hpp>

/**
 * @brief   This function plots the reference, control signal, and states of the
 *          drone.
 * 
 * It uses Matplotlib and Python to display the result.
 * 
 * @param   result 
 *          The result of a simulation to plot.
 */
void plot(const DronePlottable &result);