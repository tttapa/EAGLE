#pragma once

#include <Matrix.hpp>
#include <pybind11/embed.h>

#include <DroneParamsAndMatrices.hpp>

#include <iostream> // cerr

#if 0
// A
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::Aa_att));
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::Ad_att_r));
// B
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::Ba_att));
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::Bd_att_r));
// C
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::Ca_att));
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::Cd_att_r));
// D
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::Da_att));
// G
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::G_att));
//
// ALTITUDE
// A
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::Aa_alt));
// B
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::Ba_alt));
// C
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::Ca_alt));
// D
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::Da_alt));
// G
PYBIND11_MAKE_OPAQUE(decltype(DroneParamsAndMatrices::G_alt));
#endif

#include <PyMatrix.hpp>

PYBIND11_EMBEDDED_MODULE(PyDrone, pydronemodule) {
  pybind11::class_<DroneParamsAndMatrices>(pydronemodule,
                                           "DroneParamsAndMatrices")
      .def(pybind11::init<>())
//
// ATTITUDE
// A
#if 1
      .def_readwrite("Aa_att", &DroneParamsAndMatrices::Aa_att)
      .def_readwrite("Ad_att", &DroneParamsAndMatrices::Ad_att)
      .def_readwrite("Ad_att_r", &DroneParamsAndMatrices::Ad_att_r)
      // B
      .def_readwrite("Ba_att", &DroneParamsAndMatrices::Ba_att)
      .def_readwrite("Bd_att", &DroneParamsAndMatrices::Bd_att)
      .def_readwrite("Bd_att_r", &DroneParamsAndMatrices::Bd_att_r)
      // C
      .def_readwrite("Ca_att", &DroneParamsAndMatrices::Ca_att)
      .def_readwrite("Cd_att", &DroneParamsAndMatrices::Cd_att)
      .def_readwrite("Cd_att_r", &DroneParamsAndMatrices::Cd_att_r)
      // D
      .def_readwrite("Da_att", &DroneParamsAndMatrices::Da_att)
      .def_readwrite("Dd_att", &DroneParamsAndMatrices::Dd_att)
      // G
      .def_readwrite("G_att", &DroneParamsAndMatrices::G_att)
      //
      // ALTITUDE
      // A
      .def_readwrite("Aa_alt", &DroneParamsAndMatrices::Aa_alt)
      .def_readwrite("Ad_alt", &DroneParamsAndMatrices::Ad_alt)
      // B
      .def_readwrite("Ba_alt", &DroneParamsAndMatrices::Ba_alt)
      .def_readwrite("Bd_alt", &DroneParamsAndMatrices::Bd_alt)
      // C
      .def_readwrite("Ca_alt", &DroneParamsAndMatrices::Ca_alt)
      .def_readwrite("Cd_alt", &DroneParamsAndMatrices::Cd_alt)
      // D
      .def_readwrite("Da_alt", &DroneParamsAndMatrices::Da_alt)
      .def_readwrite("Dd_alt", &DroneParamsAndMatrices::Dd_alt)
      // G
      .def_readwrite("G_alt", &DroneParamsAndMatrices::G_alt)
      //
      // SYSTEM PARAMETERS
      .def_readwrite("Ts_att", &DroneParamsAndMatrices::Ts_att)
      .def_readwrite("Ts_alt", &DroneParamsAndMatrices::Ts_alt)
      //
      .def_readwrite("gamma_n", &DroneParamsAndMatrices::gamma_n)
      .def_readwrite("gamma_u", &DroneParamsAndMatrices::gamma_u)
#endif
      .def_readwrite("Id", &DroneParamsAndMatrices::Id)
#if 1
      .def_readwrite("Id_inv", &DroneParamsAndMatrices::Id_inv)
      .def_readwrite("k1", &DroneParamsAndMatrices::k1)
      .def_readwrite("k2", &DroneParamsAndMatrices::k2)
      //
      .def_readonly("Nm", &DroneParamsAndMatrices::Nm)
      .def_readonly("g", &DroneParamsAndMatrices::g)
      .def_readonly("rho", &DroneParamsAndMatrices::rho)
      //
      .def_readwrite("m", &DroneParamsAndMatrices::m)
      .def_readwrite("ct", &DroneParamsAndMatrices::ct)
      .def_readwrite("Dp", &DroneParamsAndMatrices::Dp)
      .def_readwrite("nh", &DroneParamsAndMatrices::nh)
#endif
      .def_readwrite("uh", &DroneParamsAndMatrices::uh)
#if 1
      //
      // METHODS
      .def("load", [](DroneParamsAndMatrices& d, const std::string &path) {
          d.load(std::filesystem::path(path));
      })
#endif
      ;
}
