#include "Config.hpp"

namespace Config {

/* ------ Discretization options -------------------------------------------- */

/** Sample frequency for discrete controller. */
const double fs = 238.0;
/** Time step for discrete controller. */
const double Ts = 1.0 / fs;

/* ------ Simulation settings ----------------------------------------------- */

/** Clamping boundaries */
const ColVector<3> clampMin = -1.0 / 3 * ones<3, 1>();
const ColVector<3> clampMax = 1.0 / 3 * ones<3, 1>();

/* ------ LQR weighting matrices Q and R ------------------------------------ */

static auto invsq = [](double x) { return 1.0 / (x * x); };

const RowVector<3> Qn_initial     = {{1e-06, 1e-06, 0.00064052634}};
const RowVector<3> Qomega_initial = {{0.034129327, 0.034129327, 1e-06}};
const RowVector<3> Qq_initial     = {{127.26504, 127.26504, 17719.678}};

/** Weighting matrix for states in LQR design. */
const ColVector<9> Q_diag_initial =
    transpose(hcat(Qq_initial, Qomega_initial, Qn_initial));
/** Weighting matrix for inputs in LQR design. */
const ColVector<3> R_diag_initial = ones<3, 1>();

/* ------ Tuner mutation variance ------------------------------------------- */
const double varQ[9] = {1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};
const double varR[3] = {1e-2, 1e-2, 1e-2};

const ColVector<9> Qmin = 1e-10 * ones<9, 1>();
const ColVector<3> Rmin = 1e-10 * ones<3, 1>();

const ColVector<9> Qmax = 1e6 * ones<9, 1>();
const ColVector<3> Rmax = 1e6 * ones<3, 1>();

/* ------ Simulation options (for ODE solver) ------------------------------- */

/** Options for numerical integration for simulation. */
const AdaptiveODEOptions odeopt = {
    .t_start = 0,
    .t_end   = 1,
    .epsilon = 1e-6,
    .h_start = 1e-2,
    .h_min   = 1e-6,
    .maxiter = (unsigned long) 1e6,
};

}  // namespace Config