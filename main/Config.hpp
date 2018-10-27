#pragma once

#include "Drone.hpp"
#include <ODE/ODEOptions.hpp>
#include <string>

namespace Config {

// Home directory
const std::string home = getenv("HOME");

// Filename for simulation output
const std::string outputFile = home + "/Random/data.csv";

// Sample rate for simulation output
constexpr double fs = 60.0;      // Sample frequency
constexpr double Ts = 1.0 / fs;  // Sample time step

// Plot the samled version instead of "continuous"
constexpr bool plotSampled = false;

/* ------ Define drone parameters ------------------------------------------- */
const Drone drone = {};

/* ------ LQR weighting matrices Q and R ------------------------------------ */

auto invsq = [](double x) { return 1.0 / (x * x); };

const double u_att_max = 1 - drone.p.nh / drone.p.k1;
const double n_att_max = drone.p.k1 * u_att_max;

const auto Qn     = invsq(n_att_max) * ones<1, 3>();
const auto Qomega = zeros<1, 3>();
const auto Qq     = 3.0 * ones<1, 3>();

const auto Q = diag(hcat(Qq, Qomega, Qn));
const auto R = invsq(u_att_max) * eye<3>();

/* ------ Simulation options (for ODE solver) ------------------------------- */

const AdaptiveODEOptions odeopt = {
    .t_start = 0,
    .t_end   = 11,
    .epsilon = 1e-6,
    .h_start = 1e-2,
    .h_min   = 1e-6,
    .maxiter = 10000000,
};

}  // namespace Config