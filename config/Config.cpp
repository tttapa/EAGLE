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

/* ------ Attitude LQR & LQE ------------------------------------------------ */

namespace Attitude {
// const RowVector<3> Qq     = {{
//     103.23848252443499,
//     106.42413118528627,
//     106.42413118528627,
// }};
// const RowVector<3> Qomega = {{
//     0.030929762777928395,
//     0.030929762777928395,
//     0.0043504717329451159,
// }};
// const RowVector<3> Qn     = {{
//     1e-10,
//     1e-10,
//     1e-10,
// }};
// const RowVector<3> Rr     = {{
//     1.7142847029476442,
//     1.7142847029476442,
//     0.43951272021883059,
// }};

const RowVector<3> Qq     = {{
    1.2578246839673057,
    1.2664023040438583,
    1.2664023040438583,
}};
const RowVector<3> Qomega = {{
    0.030276768974950123,
    0.030276768974950123,
    1e-10,
}};
const RowVector<3> Qn     = {{
    0.065644174665042174,
    0.065644174665042174,
    0.025955955595682777,
}};
const RowVector<3> Rr     = {{
    1e-10,
    1e-10,
    1e-10,
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
    .t_end   = 2.5,
    .epsilon = 1e-4,
    .h_start = 1e-4,
    .h_min   = 1e-6,
    .maxiter = (unsigned long) 1e5,
};

const AdaptiveODEOptions odeoptdisp = {
    .t_start = 0.0,
    .t_end   = 0.5 * (7 + 1),
    .epsilon = 1e-4,
    .h_start = 1e-4,
    .h_min   = 1e-6,
    .maxiter = (unsigned long) 1e5,
};

/* ------ LQR --------------------------------------------------------------- */
// const RowVector<3> Qq_initial     = {1, 1, 1};
// const RowVector<3> Qomega_initial = {1, 1, 1};
// const RowVector<3> Qn_initial     = {1, 1, 1};

const RowVector<3> Qq_initial     = Config::Attitude::Qq;
const RowVector<3> Qomega_initial = Config::Attitude::Qomega;
const RowVector<3> Qn_initial     = Config::Attitude::Qn;

/** Weighting matrix for states in LQR design. */
const ColVector<9> Q_diag_initial =
    transpose(hcat(Qq_initial, Qomega_initial, Qn_initial));
/** Weighting matrix for inputs in LQR design. */
const ColVector<3> R_diag_initial = transpose(Config::Attitude::Rr);

/* ------ Tuner mutation variance ------------------------------------------- */
const ColVector<9> varQ = 0.01 * Q_diag_initial;
const ColVector<3> varR = 0.01 * R_diag_initial;

const ColVector<9> Qmin = 1e-10 * ones<9, 1>();
const ColVector<3> Rmin = 1e-10 * ones<3, 1>();

const ColVector<9> Qmax = 1e6 * ones<9, 1>();
const ColVector<3> Rmax = 1e6 * ones<3, 1>();

/* ------ Genetic algorithm settings ---------------------------------------- */
const size_t population  = 16 * 64;
const size_t generations = 50;
const size_t survivors   = 16;

/* ------ Image export dimensions ------------------------------------------- */
const size_t px_x = 1920;
const size_t px_y = 1080;

}  // namespace Tuner

}  // namespace Config