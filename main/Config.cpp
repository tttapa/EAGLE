#include "Config.hpp"

namespace Config {

// Filename for simulation output
const std::string outputFile = home + "/Random/data.csv";

// Sample rate for simulation output
const double fs = 10.0;     // Sample frequency
const double Ts = 1.0 / fs;  // Sample time step

// Plot the samled version instead of "continuous"
const bool plotSampled = false;
// Plot the result of the continuous controller instead of the discrete one.
const bool simulateContinuousController = false;

/* ------ LQR weighting matrices Q and R ------------------------------------ */
static auto invsq = [](double x) { return 1.0 / (x * x); };

const RowVector<3> Qn     = invsq(n_att_max) * ones<1, 3>();
const RowVector<3> Qomega = zeros<1, 3>();
const RowVector<3> Qq     = 10.0 * ones<1, 3>();

const Matrix<9, 9> Q = diag(hcat(Qq, Qomega, Qn));
const Matrix<3, 3> R = invsq(u_att_max) * eye<3>();

/* ------ Simulation options (for ODE solver) ------------------------------- */

const AdaptiveODEOptions odeopt = {
    .t_start = 0,
    .t_end   = 12,
    .epsilon = 1e-6,
    .h_start = 1e-2,
    .h_min   = 1e-6,
    .maxiter = (unsigned long) 1e6,
};

}  // namespace Config