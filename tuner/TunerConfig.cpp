#include "TunerConfig.hpp"

/* ------ Tuner settings ---------------------------------------------------- */
namespace Tuner {

/* ------ Matrix & Parameter data loading ----------------------------------- */
const std::filesystem::path loadPath = Config::loadPath;

const AdaptiveODEOptions odeopt = {
    .t_start = 0.0,
    .t_end   = 3.0,
    .epsilon = 1e-4,
    .h_start = 1e-4,
    .h_min   = 1e-7,
    .maxiter = (unsigned long) 1e5,
};

const AdaptiveODEOptions odeoptdisp = {
    .t_start = 0.0,
    .t_end   = 0.5 * (7 + 1),
    .epsilon = 1e-4,
    .h_start = 1e-4,
    .h_min   = 1e-7,
    .maxiter = (unsigned long) 1e5,
};

/* ------ LQR --------------------------------------------------------------- */
#if 1
/** Weighting matrix for states in LQR design. */
const ColVector<9> Q_diag_initial = {
    1, 1, 1, 1, 1, 1, 1, 1, 1,
};
/** Weighting matrix for inputs in LQR design. */
const ColVector<3> R_diag_initial = {
    1,
    1,
    1,
};
#else
const ColVector<9> Q_diag_initial = transpose(Config::Attitude::Qdiag);
const ColVector<3> R_diag_initial = transpose(Config::Attitude::Rdiag);
#endif

/* ------ Tuner mutation variance ------------------------------------------- */
const ColVector<9> varQ = 0.05 * Q_diag_initial;
const ColVector<3> varR = 0.05 * R_diag_initial;

const ColVector<9> Qmin = 1e-10 * ones<9, 1>();
const ColVector<3> Rmin = 1e-2 * ones<3, 1>();

const ColVector<9> Qmax = 1e6 * ones<9, 1>();
const ColVector<3> Rmax = 1e2 * ones<3, 1>();

/* ------ Genetic algorithm settings ---------------------------------------- */
const size_t population  = 16 * 64;
const size_t generations = 50;
const size_t survivors   = 16;

/* ------ Cost function parameters ------------------------------------------ */
const CostWeights stepcostweights = {
    .notRisen   = 1e20,
    .notSettled = 1e10,
    .risetime   = 1e1,
    .overshoot  = 5e2,
    .settleTime = 5e-1,
};

/* ------ Time step error factor (for rise and settling time) --------------- */
const double steperrorfactor = 0.01;  // 1% of step size // TODO

/* ------ Plot settings ----------------------------------------------------- */
const bool plotSimulationResult = true;
const bool plotStepResponse     = true;

const bool plotAllAtOnce = true;

}  // namespace Tuner
