#pragma once

#include "CostWeights.hpp"
#include <Matrix.hpp>
#include <ODEOptions.hpp>
#include <filesystem>

namespace Config {

/* ------ Tuner settings ---------------------------------------------------- */
namespace Tuner {
/* ------ Matrix & Parameter data loading ----------------------------------- */
extern const std::filesystem::path loadPath;

extern const AdaptiveODEOptions odeopt;
extern const AdaptiveODEOptions odeoptdisp;

/* ------ LQR --------------------------------------------------------------- */
extern const ColVector<9> Q_diag_initial;
extern const ColVector<3> R_diag_initial;

/* ------ Tuner mutation variance ------------------------------------------- */
extern const ColVector<9> varQ;
extern const ColVector<3> varR;

extern const ColVector<9> Qmin;
extern const ColVector<3> Rmin;

extern const ColVector<9> Qmax;
extern const ColVector<3> Rmax;

/* ------ Genetic algorithm settings ---------------------------------------- */
extern const size_t population;
extern const size_t generations;
extern const size_t survivors;

/* ------ Cost function parameters ------------------------------------------ */
extern const CostWeights stepcostweights;

/* ------ Time step error factor (for rise and settling time) --------------- */
extern const double steperrorfactor;

/* ------ Plot settings ----------------------------------------------------- */
extern const bool plotSimulationResult;
extern const bool plotStepResponse;

extern const bool plotAllAtOnce;

}  // namespace Tuner

}  // namespace Config