#pragma once

#include "Drone.hpp"
#include <ODE/ODEOptions.hpp>
#include <filesystem>

namespace Config {

// Home directory
const std::filesystem::path home = getenv("HOME");

/* ------ CSV export settings ----------------------------------------------- */
// Filenames for simulation output
extern const std::filesystem::path rotationCSVFile;
extern const std::filesystem::path locationCSVFile;
// Export the simulation data as CSV.
extern const bool exportCSV;
// Sample frequency for CSV output (fps).
extern const double CSV_fs;
// Time step for discrete controller.
extern const double CSV_Ts;

/* ------ Matrix & Parameter data loading ----------------------------------- */
extern const std::filesystem::path loadPath;

/* ------ LQR weighting matrices Q and R ------------------------------------ */
extern const RowVector<3> Qn;
extern const RowVector<3> Qomega;
extern const RowVector<3> Qq;

extern const Matrix<9, 9> Q;
extern const Matrix<3, 3> R;

/* ------ PI constants altitude controller ---------------------------------- */
extern const Matrix<1, 3> K_alt_p;
extern const Matrix<1, 3> K_alt_i;

/* ------ Kalman variance matrices ------------------------------------------ */
extern const RowVector<3> varDynamics;
extern const RowVector<6> varSensors;

/* ------ Simulation options (for ODE solver) ------------------------------- */
extern const AdaptiveODEOptions odeopt;

}  // namespace Config