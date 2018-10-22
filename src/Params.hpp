#include "Matrix.hpp"
#include <cmath>

struct Params {
  // QUAT_PARAMS returns the nomimal parameters of the quadcopter in a
  // structure.
  //
  // Parameters:
  //
  // SYMBOL   | UNITS | NAME
  // --------------------------------------------------------------
  // Vmax     | V     | maximum voltage
  // Vmin     | V     | minimum voltage
  // Nm       | -     | number of motors
  // m        | kg    | total mass
  // L        | m     | arm length
  // rho      | kg/m3 | air density (nominal, at 15C, sea level)
  // g        | m/s2  | gravitational acceleration
  // ct       | -     | thrust coefficient
  // cp       | -     | power coefficient
  // mp       | kg    | propeller mass
  // Dp       | m     | propeller diameter
  // Kv       | rpm/V | motor speed constant
  // tau_m    | s     | motor's time constant
  // mr       | kg    | rotor mass (moving parts)
  // mm       | kg    | motor total mass
  // rr       | m     | rotor mass
  // nh       | rps   | hovering frequency of rotation
  // Ip       | kgm2  | propeller moment of inertia
  // Im       | kgm2  | motor's moment of inertia
  // Ixx      | kgm2  | moment of inertia about the x-axis
  // Iyy      | kgm2  | moment of inertia about the y-axis
  // Izz      | kgm2  | moment of inertia about the z-axis
  // I        | kgm2  | diagonal matrix (Ixx, Iyy, Izz)
  // k1       |       | parameter k1
  // k2       | 1/s   | parameter k2
  // k3       | (fun) | parameter k3
  // k4       |       | parameter k4 (reaction wheel effect)
  // gamma_n  |       |
  // gamma_u  |       |
  // --------------------------------------------------------------
  //
  // Parameters gamma_n and gamma_u define the linearised n-s dynamics as
  // follows
  //
  // dn/dt = k2 k1 u - k2 n
  //     s = gamma_n n + gamma_u u

  // ---- Fixed Quantities ----

  // general parameters
  double Vmax = 11.1;       // V ....... maximum voltage
  double Vmin = 0.1 * Vmax; // V ....... minimum voltage
  int Nm = 4;               // - ....... number of motors
  double m = 1.850;         // kg ...... total mass
  double L = 0.27;          // m ....... arm length
  double rho = 1.225; // kg/m3 ... air density (nominal, at 15C, sea level)
  double g = 9.81;    // m/s2 .... gravitational acceleration

  // propellers
  double ct = 0.1;             // - ....... thrust coefficient
  double cp = 0.04;            // - ....... power coefficient
  double mp = 20 / 1000;       // kg ...... propeller mass (20g)
  double Dp = 11 * 2.54 / 100; // m ....... propeller diameter (11")

  // motors
  double Kv = 700;          // rpm/V ... motor speed constant
  double tau_m = 35 / 1000; // s ....... motor's time constant (35ms)
  double mr = 42 / 1000;    // kg ...... rotor mass (moving parts) (42g)
  double mm = 102 / 1000;   // kg ...... motor total mass (102g)
  double rr = 1.9 / 100;    // m ....... rotor radius (1.9cm)

  // Moments of intertia
  double Ixx = 0.0321; // kgm2 .... Ixx moment of inertia
  double Iyy = 0.0340; // kgm2 .... Iyy moment of inertia
  double Izz = 0.0575; // kgm2 .... Izz moment of inertia

  // ---- Computed Quantities ----

  double nh = // rps ... hovering n
      sqrt((m * g) / (ct * rho * pow(Dp, 4) * Nm));

  // Very rough estimation of moments of inertia
  double Ip = mp * pow(Dp, 2) / 12; // kgm2 .... propeller moment of inertia
  double Im = mr * pow(rr, 2);      // kgm2 .... rotor moment of inertia
  Matrix<double, 3, 3> I =
      diag<double, 3>({Ixx, Iyy, Izz}); // kgm3 .... Inertia matrix

  // model constants
  double k1 = Kv * (Vmax - Vmin) / 60;
  double k2 = 1 / tau_m;
  Matrix<double, 3, 3> k3 =
      diag<double, 3>({2 * ct * rho * nh * pow(Dp, 4) * Nm * L / sqrt(2) / Ixx,
                       2 * ct *rho *nh *pow(Dp, 4) * Nm *L / sqrt(2) / Iyy,
                       2 * cp *rho *nh *pow(Dp, 5) * Nm / (2 * M_PI * Izz)});
  Matrix<double, 3, 3> k4 =
      diag<double, 3>({0, 0, 2 * M_PI *Nm *(Im + Ip) / Izz});

  // Matrix Gamma_n
  Matrix<double, 3, 3> gamma_n = k3 - k2 * k4;

  // Matrix Gamma_u
  Matrix<double, 3, 3> gamma_u = diag<double, 3>({0, 0, k4[3][3] * k2 *k1});
};