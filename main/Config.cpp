#include "Config.hpp"

namespace Config {

/* ------ CSV export settings ----------------------------------------------- */

/** Filename for simulation output. */
const std::string outputFile =
    home + "/PO-EAGLE/Groups/ANC/Blender/Animation-Data/rotation.csv";
/** Export the simulation data as CSV. */
const bool exportCSV = true;
/** Sample frequency for CSV output (fps). */
const double CSV_fs = 30.0;
/** Time step for discrete controller. */
const double CSV_Ts = 1.0 / CSV_fs;

/* ------ Discretization options -------------------------------------------- */

/** Sample frequency for discrete controller. */
const double fs = 238.0;
/** Time step for discrete controller. */
const double Ts = 1.0 / fs;

/* ------ Simulation settings ----------------------------------------------- */

/** Clamp the motor control outputs. */
const bool clampController = true;
/** Clamping boundaries */
const ColVector<3> clampMin = -1.0 / 3 * ones<3, 1>();
const ColVector<3> clampMax = 1.0 / 3 * ones<3, 1>();

/* ------ LQR weighting matrices Q and R ------------------------------------ */

static auto invsq = [](double x) { return 1.0 / (x * x); };

const RowVector<3> Qq     = {{448.85340, 448.85340, 381.83708}};
const RowVector<3> Qomega = {{0.17101848, 0.17101848, 0.0025328247}};
const RowVector<3> Qn     = {{3.807243e-05, 3.807243e-05, 3.0956794e-06}};

/** Weighting matrix for states in LQR design. */
const Matrix<9, 9> Q = diag(hcat(Qq, Qomega, Qn));
/** Weighting matrix for inputs in LQR design. */
const Matrix<3, 3> R = eye<3>();

/* ------ Kalman variance matrices ------------------------------------------ */
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

/* ------ Simulation options (for ODE solver) ------------------------------- */

/** Options for numerical integration for simulation. */
const AdaptiveODEOptions odeopt = {
    .t_start = 0,
    .t_end   = 36,
    .epsilon = 1e-6,
    .h_start = 1e-2,
    .h_min   = 1e-6,
    .maxiter = (unsigned long) 1e6,
};

}  // namespace Config