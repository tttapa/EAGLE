#pragma once

#include "Drone.hpp"
#include <ODE/ODEOptions.hpp>
#include <string>

namespace Config {

// Home directory
const std::string home = getenv("HOME");

// Filename for simulation output
const extern std::string outputFile;

// Sample rate for simulation output
extern const double fs;  // Sample frequency
extern const double Ts;  // Sample time step

// Plot the samled version instead of "continuous"
extern const bool plotSampled;
// Plot the result of the continuous controller instead of the discrete one.
extern const bool simulateContinuousController;

/* ------ Define drone parameters ------------------------------------------- */
constexpr Drone drone = {};

/* ------ LQR weighting matrices Q and R ------------------------------------ */
constexpr double u_att_max = 1 - drone.p.nh / drone.p.k1;
constexpr double n_att_max = drone.p.k1 * u_att_max;

extern const RowVector<3> Qn;
extern const RowVector<3> Qomega;
extern const RowVector<3> Qq;

extern const Matrix<9, 9> Q;
extern const Matrix<3, 3> R;

/* ------ Simulation options (for ODE solver) ------------------------------- */
extern const AdaptiveODEOptions odeopt;

}  // namespace Config