#pragma once

#include <Matrix.hpp>
#include <ODEOptions.hpp>
#include <filesystem>

namespace Config {

// Home directory
const std::filesystem::path home = getenv("HOME");

/* ------ Matrix & Parameter data loading ----------------------------------- */
extern const std::filesystem::path loadPath;

/* ------ Image export dimensions ------------------------------------------- */
extern const size_t px_x;
extern const size_t px_y;

/* ------ Attitude LQR & LQE ------------------------------------------------ */

namespace Attitude {
extern const Matrix<9, 9> Q;
extern const Matrix<3, 3> R;

extern const RowVector<3> varDynamics;
extern const RowVector<7> varSensors;
}  // namespace Attitude

/* ------ Altitude PI controller and LQE ------------------------------------ */

namespace Altitude {
extern const Matrix<1, 3> K_p;
extern const Matrix<1, 1> K_i;
extern const Matrix<1, 4> K_pi;
extern const double maxIntegralInfluence;

extern const RowVector<1> varDynamics;
extern const RowVector<1> varSensors;
}  // namespace Altitude

/* ------ Simulation options (for ODE solver) ------------------------------- */
extern const AdaptiveODEOptions odeopt;

}  // namespace Config