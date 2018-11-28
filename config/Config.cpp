#include "Config.hpp"

namespace Config {

/* ------ CSV export settings ----------------------------------------------- */

/** Filenames for simulation output. */
const std::filesystem::path rotationCSVFile =
    home / "PO-EAGLE/Groups/ANC/Blender/Animation-Data/rotation.csv";
const std::filesystem::path locationCSVFile =
    home / "PO-EAGLE/Groups/ANC/Blender/Animation-Data/location.csv";
/** Export the simulation data as CSV. */
const bool exportCSV = true;
/** Sample frequency for CSV output (fps). */
const double CSV_fs = 30.0;
/** Time step for discrete controller. */
const double CSV_Ts = 1.0 / CSV_fs;

/* ------ Matrix & Parameter data loading ----------------------------------- */
const std::filesystem::path loadPath =
    home / "PO-EAGLE/Groups/ANC/MATLAB/Codegen";

/* ------ Image export dimensions ------------------------------------------- */
const size_t px_x = 1920;
const size_t px_y = 1080;

/* ------ Plot settings ----------------------------------------------------- */
const bool plotSimulationResult = true;
const bool plotMotorControls    = false;
const bool plotStepResponse     = true;

const bool plotAllAtOnce = true;

/* ------ Time step error factor (for rise and settling time) --------------- */
const double steperrorfactor = 0.01;  // 1% of step size

/* ------ Attitude LQR & LQE ------------------------------------------------ */

namespace Attitude {
const RowVector<3> Qq     = {{
    248.2566541613234,
    285.3564859658812,
    285.3564859658812,
}};
const RowVector<3> Qomega = {{
    0.06913895186952017,
    0.06913895186952017,
    0.01378019322006485,
}};
const RowVector<3> Qn     = {{
    1.39385333888989e-05,
    1.39385333888989e-05,
    1.256521771966514e-10,
}};
const RowVector<3> Rr     = {{
    1,
    1,
    0.1822721786710156,
}};

/** Weighting matrix for states in LQR design. */
const Matrix<9, 9> Q = diag(hcat(Qq, Qomega, Qn));
/** Weighting matrix for inputs in LQR design. */
const Matrix<3, 3> R = diag(Rr);

/** @todo   Tune */
const RowVector<3> varDynamics = {{
    1e-4,
    1e-4,
    1e-6,
}};
const RowVector<6> varSensors  = hcat(  //
    M_PI / 180.0 * ones<1, 3>(),       //
    0.005 * ones<1, 3>()               //
);
}  // namespace Attitude

/* ------ Altitude PI controller and LQE ------------------------------------ */

namespace Altitude {
/** Proporional altitude controller */
const Matrix<1, 3> K_p = {0.0001, 1.2, 0.54};  // n, z, v
/** Integral altitude controller */
const Matrix<1, 3> K_i = {0.0, 0.001, 0.0};

const RowVector<1> varDynamics = {0.01};
const RowVector<1> varSensors  = {0.02};  // 2 cm
}  // namespace Altitude

/* ------ Simulation options (for ODE solver) ------------------------------- */

/** Options for numerical integration for simulation. */
const AdaptiveODEOptions odeopt = {
    .t_start = 0,
    .t_end   = 16,
    .epsilon = 1e-6,
    .h_start = 1e-6,
    .h_min   = 1e-10,
    .maxiter = (unsigned long) 1e6,
};

/* ------ Tuner settings ---------------------------------------------------- */
namespace Tuner {

/* ------ Matrix & Parameter data loading ----------------------------------- */
const std::filesystem::path loadPath = home / "Private" / "EAGLE-Params";

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
#if 0
const RowVector<3> Qq_initial     = {1, 1, 1};
const RowVector<3> Qomega_initial = {1, 1, 1};
const RowVector<3> Qn_initial     = {1, 1, 1};
const RowVector<3> Rr_initial     = {1, 1, 1};
#else
const RowVector<3> Qq_initial     = Config::Attitude::Qq;
const RowVector<3> Qomega_initial = Config::Attitude::Qomega;
const RowVector<3> Qn_initial     = Config::Attitude::Qn;
const RowVector<3> Rr_initial     = Config::Attitude::Rr;
#endif

/** Weighting matrix for states in LQR design. */
const ColVector<9> Q_diag_initial =
    transpose(hcat(Qq_initial, Qomega_initial, Qn_initial));
/** Weighting matrix for inputs in LQR design. */
const ColVector<3> R_diag_initial = transpose(Rr_initial);

/* ------ Tuner mutation variance ------------------------------------------- */
const ColVector<9> varQ = 0.025 * Q_diag_initial;
const ColVector<3> varR = 0.025 * R_diag_initial;

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
    .overshoot  = 5e0,
    .settleTime = 5e-1,
};

/* ------ Time step error factor (for rise and settling time) --------------- */
const double steperrorfactor = 0.01;  // 1% of step size // TODO

/* ------ Plot settings ----------------------------------------------------- */
const bool plotSimulationResult = true;
const bool plotStepResponse     = true;

const bool plotAllAtOnce = true;

}  // namespace Tuner

}  // namespace Config