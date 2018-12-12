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
    home / "PO-EAGLE/Groups/ANC/MATLAB/Codegen/ParamsAndMatrices/Output";

/* ------ Logger data loading ----------------------------------------------- */
const std::filesystem::path logLoadPath = home / "eagle.dat";

/* ------ Image export dimensions ------------------------------------------- */
const size_t px_x = 1920;
const size_t px_y = 1080;

/* ------ Plot settings ----------------------------------------------------- */
const bool plotSimulationResult = false;
const bool plotMotorControls    = false;
const bool plotStepResponse     = false;

const bool plotCSimulationResult = false;

const bool plotLoggedDroneData = true;

const bool plotAllAtOnce = true;

/* ------ Time step error factor (for rise and settling time) --------------- */
const double steperrorfactor = 0.01;  // 1% of step size

/* ------ Attitude LQR & LQE ------------------------------------------------ */

namespace Attitude {
const RowVector<9> Qdiag = {{
    139.6245112700232,
    139.6245112700232,
    24.2811761590895,
    1.1505204155597211,
    1.1505204155597211,
    0.3409919487616804,
    9.976475759487083e-08,
    9.976475759487083e-08,
    9.976475759487083e-09,
}};
const RowVector<3> Rdiag = {{
    1,
    1,
    1.001966068300933,
}};

/** Weighting matrix for states in LQR design. */
const Matrix<9, 9> Q = diag(Qdiag);
/** Weighting matrix for inputs in LQR design. */
const Matrix<3, 3> R = 8 * diag(Rdiag);

/* ------ Compare two controllers ------------------------------------------- */
namespace Compare {
const RowVector<9> Qdiag1 = Config::Attitude::Qdiag;
const RowVector<3> Rdiag1 = Config::Attitude::Rdiag;

const Matrix<9, 9> Q1 = diag(Qdiag1);
const Matrix<3, 3> R1 = diag(Rdiag1);

const RowVector<9> Qdiag2 = {{
    510.25,
    510.25,
    477.64,
    0.202,
    0.202,
    0.000329,
    3.78906e-06,
    3.789057e-06,
    7.8753e-08,
}};
const RowVector<3> Rdiag2 = {{
    9.1215,
    9.1215,
    9.1215,
}};

const Matrix<9, 9> Q2 = diag(Qdiag2);
const Matrix<3, 3> R2 = diag(Rdiag2);

const bool compare = false;
}  // namespace Compare

/** @todo   Tune */
const RowVector<12> varDynamics = {{
    1e-2,
    1e-2,
    1e-2,
    1e-2,
    1e-2,
    1e-2,
    1e-2,
    1e-2,
    1e-2,
    1e-4,
    1e-4,
    1e-6,
}};
const RowVector<6> varSensors   = hcat(  //
    M_PI / 180.0 * ones<1, 3>(),       //
    0.005 * ones<1, 3>()               //
);
}  // namespace Attitude

/* ------ Altitude PI controller and LQE ------------------------------------ */

namespace Altitude {
/** Proporional altitude controller */
const Matrix<1, 3> K_p = {0.0, 0.4, 0.4};  // n, z, v
/** Integral altitude controller */
const Matrix<1, 1> K_i = {-0.005};
/** Complete altitude controller */
const Matrix<1, 4> K_pi = hcat(K_p, K_i);

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
    .h_min   = 1e-8,
    .maxiter = (unsigned long) 1e6,
};

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

}  // namespace Config