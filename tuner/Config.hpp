#pragma once

#include "TunerDrone.hpp"
#include <ODE/ODEOptions.hpp>
#include <string>

namespace Config {

// Home directory
const std::string home = getenv("HOME");

// Sample rate for simulation output
extern const double fs;  // Sample frequency
extern const double Ts;  // Sample time step

// Clamping boundaries
extern const ColVector<3> clampMin;
extern const ColVector<3> clampMax;

/* ------ Define drone parameters ------------------------------------------- */
constexpr Drone drone = {};

/* ------ LQR weighting matrices Q and R ------------------------------------ */
constexpr double u_att_max = 1 - drone.p.nh / drone.p.k1;
constexpr double n_att_max = drone.p.k1 * u_att_max;

extern const RowVector<3> Qn_initial;
extern const RowVector<3> Qomega_initial;
extern const RowVector<3> Qq_initial;

extern const ColVector<9> Q_diag_initial;
extern const ColVector<3> R_diag_initial;

extern const ColVector<9> Qmin;
extern const ColVector<3> Rmin;

extern const ColVector<9> Qmax;
extern const ColVector<3> Rmax;

/* ------ Tuner mutation variance ------------------------------------------- */
extern const double varQ[9];
extern const double varR[3];

/* ------ Simulation options (for ODE solver) ------------------------------- */
extern const AdaptiveODEOptions odeopt;

}  // namespace Config