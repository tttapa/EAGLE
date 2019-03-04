#include "Config.hpp"
#include <Quaternion.hpp>

namespace Config {

/* ------ Matrix & Parameter data loading ----------------------------------- */
const std::filesystem::path loadPath =
    home / "PO-EAGLE/Groups/ANC/Cleanup-Pieter/Code-Generators/"
           "ParamsAndMatrices/Output";

/* ------ Image export dimensions ------------------------------------------- */
const size_t px_x = 1920;
const size_t px_y = 1080;

/* ------ Attitude LQR & LQE ------------------------------------------------ */

namespace Attitude {
const RowVector<9> Qdiag = {{
    139.6245112700232,
    139.6245112700232,
    15.2811761590895,
    1.1505204155597211,
    1.1505204155597211,
    0.1209919487616804,
    9.976475759487083e-08,
    9.976475759487083e-08,
    9.976475759487083e-09,
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
const RowVector<3> varDynamics = {{
    //    1e-2,
    //    1e-2,
    //    1e-2,
    //    1e-2,
    //    1e-2,
    //    1e-2,
    //    1e-2,
    //    1e-2,
    //    1e-2,
    1e-4,
    1e-4,
    1e-6,
}};
const RowVector<7> varSensors  = hcat(                  //
    transpose(eul2quat(M_PI / 180.0 * ones<3, 1>())),  //
    0.005 * ones<1, 3>()                               //
);
}  // namespace Attitude

/* ------ Altitude PI controller and LQE ------------------------------------ */

namespace Altitude {
/** Proporional altitude controller */
const Matrix<1, 3> Qdiag = {0.001, 1, 0.5};  // n, z, v
const Matrix<3, 3> Q = diag(Qdiag);
/** Integral altitude controller */
const Matrix<1, 1> K_i = {-0.01};
/** Anti-windup for integral controller */
const double maxIntegralInfluence = 0.03;

const RowVector<1> varDynamics = {1e-1};
const RowVector<1> varSensors  = {1e-2};  // 1 cm
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
