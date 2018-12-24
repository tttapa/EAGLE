#include "Config.hpp"

namespace Config {

/* ------ Matrix & Parameter data loading ----------------------------------- */
const std::filesystem::path loadPath =
    home / "PO-EAGLE/Groups/ANC/MATLAB/Codegen/ParamsAndMatrices/Output";

/* ------ Image export dimensions ------------------------------------------- */
const size_t px_x = 1920 * 1.5;
const size_t px_y = round(px_x / sqrt(2)) - 360;

/* ------ Attitude LQR & LQE ------------------------------------------------ */

namespace Attitude {
const RowVector<9> Qdiag = {{
    139.6245112700232,139.6245112700232,15.2811761590895,
    1.1505204155597211,1.1505204155597211,0.1209919487616804,
    9.976475759487083e-08,9.976475759487083e-08,9.976475759487083e-09,
}};
const RowVector<3> Rdiag = {{
    8,
    8,
    8,
}};

/** Weighting matrix for states in LQR design. */
const Matrix<9, 9> Q = diag(Qdiag);
/** Weighting matrix for inputs in LQR design. */
const Matrix<3, 3> R = diag(Rdiag);

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
const Matrix<1, 3> K_p = {0.0, 0.9, 0.5};  // n, z, v
/** Integral altitude controller */
const Matrix<1, 1> K_i = {-0.01};
/** Complete altitude controller */
const Matrix<1, 4> K_pi = hcat(K_p, K_i);
/** Anti-windup for integral controller */
const double maxIntegralInfluence = 0.1;

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

}  // namespace Config