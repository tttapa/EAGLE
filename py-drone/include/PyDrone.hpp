#pragma once

#include <Drone.hpp>
#include <DronePlot.hpp>
#include <Matrix.hpp>
#include <PlotHelpers.hpp>
#include <PyMatrix.hpp>
#include <iostream>  // cerr
#ifndef EXTERNAL_PY_MODULE
#include <Plot.hpp>
#include <pybind11/embed.h>
#else
#include <pybind11/pybind11.h>
#endif

class __attribute__((visibility("hidden"))) PythonDroneReferenceFunction
    : public Drone::ReferenceFunction {
  public:
    PythonDroneReferenceFunction(pybind11::object callable)
        : callable(callable) {}

    Drone::VecR_t operator()(double t) override {
        return callable(t).cast<Drone::VecR_t>();
    }

  private:
    pybind11::object callable;
};

using PythonGaussianNoiseGeneratorInput =
    GaussianNoiseGenerator<Drone::VecU_t::length()>;
using PythonGaussianNoiseGeneratorOutput =
    GaussianNoiseGenerator<Drone::VecY_t::length()>;

#ifdef EXTERNAL_PY_MODULE
PYBIND11_MODULE(py_drone_module, pydronemodule) {
#else
PYBIND11_EMBEDDED_MODULE(PyDrone, pydronemodule) {
#endif
    pybind11::class_<DroneParamsAndMatrices>(pydronemodule,
                                             "DroneParamsAndMatrices")
        .def(pybind11::init<>())
        //
        // ATTITUDE
        // A
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
        //
        .def_readwrite("Ts_att", &DroneParamsAndMatrices::Ts_att)
        .def_readwrite("Ts_alt", &DroneParamsAndMatrices::Ts_alt)
        //
        .def_readwrite("gamma_n", &DroneParamsAndMatrices::gamma_n)
        .def_readwrite("gamma_u", &DroneParamsAndMatrices::gamma_u)
        .def_readwrite("Id", &DroneParamsAndMatrices::Id)
        .def_readwrite("Id_inv", &DroneParamsAndMatrices::Id_inv)
        .def_readwrite("k1", &DroneParamsAndMatrices::k1)
        .def_readwrite("k2", &DroneParamsAndMatrices::k2)
        //
        .def_readwrite("Nm", &DroneParamsAndMatrices::Nm)
        .def_readwrite("g", &DroneParamsAndMatrices::g)
        .def_readwrite("rho", &DroneParamsAndMatrices::rho)
        //
        .def_readwrite("m", &DroneParamsAndMatrices::m)
        .def_readwrite("ct", &DroneParamsAndMatrices::ct)
        .def_readwrite("Dp", &DroneParamsAndMatrices::Dp)
        .def_readwrite("nh", &DroneParamsAndMatrices::nh)
        .def_readwrite("uh", &DroneParamsAndMatrices::uh)
        //
        // METHODS
        .def("load",
             [](DroneParamsAndMatrices &d, const std::string &path) {
                 d.load(std::filesystem::path(path));
             })
        .def("calculate", [](DroneParamsAndMatrices &d) {
            d.calculateReducedAttitudeSystemMatrices();
            d.calculateG();
        });

    pybind11::class_<DroneState>(pydronemodule, "DroneState")
        .def(pybind11::init<>())
        .def(pybind11::init<const ColVector<17> &>())
        // getters
        .def("getAttitude", &DroneState::getAttitude)
        .def("getOrientation", &DroneState::getOrientation)
        .def("getAngularVelocity", &DroneState::getAttitude)
        .def("getMotorSpeed", &DroneState::getMotorSpeed)
        .def("getVelocity", &DroneState::getVelocity)
        .def("getPosition", &DroneState::getPosition)
        .def("getThrustMotorSpeed", &DroneState::getThrustMotorSpeed)
        .def("getAltitude", &DroneState::getAltitude)
        // setters
        .def("setAttitude", &DroneState::setAttitude)
        .def("setOrientation", &DroneState::setOrientation)
        .def("setAngularVelocity", &DroneState::setAttitude)
        .def("setMotorSpeed", &DroneState::setMotorSpeed)
        .def("setVelocity", &DroneState::setVelocity)
        .def("setPosition", &DroneState::setPosition)
        .def("setThrustMotorSpeed", &DroneState::setThrustMotorSpeed)
        .def("setAltitude", &DroneState::setAltitude)
        // cast to vector
        .def("asColVector",
             [](const DroneState &d) { return ColVector<17>(d); })
        // string conversion
        .def("__str__", [](const DroneState &d) {
            return pybind11::str(pybind11::cast(ColVector<17>(d)));
        });

    pybind11::class_<DroneReference>(pydronemodule, "DroneReference")
        .def(pybind11::init<>())
        .def(pybind11::init<const ColVector<10> &>())
        // getters
        .def("getAttitude", &DroneReference::getAttitude)
        .def("getOrientation", &DroneReference::getOrientation)
        .def("getAngularVelocity", &DroneReference::getAttitude)
        .def("getPosition", &DroneReference::getPosition)
        .def("getAltitude", &DroneReference::getAltitude)
        // setters
        .def("setOrientation", &DroneReference::setOrientation)
        .def("setPosition", &DroneReference::setPosition)
        // cast to vector
        .def("asColVector",
             [](const DroneReference &d) { return ColVector<10>(d); })
        // string conversion
        .def("__str__", [](const DroneReference &d) {
            return pybind11::str(pybind11::cast(ColVector<10>(d)));
        });

    pybind11::class_<Drone::ControllerSimulationResult>(
        pydronemodule, "DroneControllerSimulationResult")
        .def(pybind11::init<>())
        .def_readonly("time", &Drone::ControllerSimulationResult::time)
        .def_readonly("solution", &Drone::ControllerSimulationResult::solution)
        .def_readonly("resultCode",
                      &Drone::ControllerSimulationResult::resultCode)
        .def_readonly("iterations",
                      &Drone::ControllerSimulationResult::iterations)
        .def_readonly("sampledTime",
                      &Drone::ControllerSimulationResult::sampledTime)
        .def_readonly("control", &Drone::ControllerSimulationResult::control)
        .def_readonly("reference",
                      &Drone::ControllerSimulationResult::reference);

    pybind11::class_<Drone::ObserverControllerSimulationResult>(
        pydronemodule, "DroneObserverControllerSimulationResult")
        .def(pybind11::init<>())
        .def_readonly("time", &Drone::ObserverControllerSimulationResult::time)
        .def_readonly("solution",
                      &Drone::ObserverControllerSimulationResult::solution)
        .def_readonly("resultCode",
                      &Drone::ObserverControllerSimulationResult::resultCode)
        .def_readonly("iterations",
                      &Drone::ObserverControllerSimulationResult::iterations)
        .def_readonly("sampledTime",
                      &Drone::ObserverControllerSimulationResult::sampledTime)
        .def_readonly("control",
                      &Drone::ObserverControllerSimulationResult::control)
        .def_readonly("reference",
                      &Drone::ObserverControllerSimulationResult::reference)
        .def_readonly("estimatedSolution",
                      &Drone::ObserverControllerSimulationResult::reference)
        .def_readonly("output",
                      &Drone::ObserverControllerSimulationResult::reference);

    pybind11::class_<Drone::Controller>(pydronemodule, "DroneController")
        .def("__call__", &Drone::Controller::operator())
        .def("reset", &Drone::Controller::reset);

    pybind11::class_<Drone::Observer>(pydronemodule, "DroneObserver")
        .def("getStateChange", &Drone::Observer::getStateChange)
        .def("reset", &Drone::Observer::reset);

    pybind11::class_<PythonDroneReferenceFunction>(pydronemodule,
                                                   "DroneReferenceFunction")
        .def(pybind11::init<pybind11::object>())
        .def("__call__", &PythonDroneReferenceFunction::operator());

    pybind11::class_<AdaptiveODEOptions>(pydronemodule, "AdaptiveODEOptions")
        .def(pybind11::init<>())
        .def(pybind11::init<const AdaptiveODEOptions &>())
        .def_readwrite("t_start", &AdaptiveODEOptions::t_start)
        .def_readwrite("t_end", &AdaptiveODEOptions::t_end)
        .def_readwrite("epsilon", &AdaptiveODEOptions::epsilon)
        .def_readwrite("h_start", &AdaptiveODEOptions::h_start)
        .def_readwrite("h_min", &AdaptiveODEOptions::h_min)
        .def_readwrite("maxiter", &AdaptiveODEOptions::maxiter);

    pybind11::class_<Drone>(pydronemodule, "Drone")
        .def(pybind11::init<const std::filesystem::path &>())
        .def(pybind11::init<const DroneParamsAndMatrices &>())
        .def(pybind11::init<>())
        .def("__call__", &Drone::operator())
        .def("__call__", [](Drone &d, const DroneState &x,
                            const ColVector<4> &u) { return d(x, u); })
        .def("getOutput", &Drone::getOutput)
        .def("getStableState", &Drone::getStableState)
        .def("getController",
             pybind11::overload_cast<const Matrix<Nx_att - 1, Nx_att - 1> &,
                                     const Matrix<Nu_att, Nu_att> &,
                                     const Matrix<1, 4> &, double>(
                 &Drone::getController))
        .def("getController",
             pybind11::overload_cast<const Matrix<Nx_att - 1, Nx_att - 1> &,
                                     const Matrix<Nu_att, Nu_att> &,
                                     const Matrix<3, 3> &, const Matrix<1, 1> &,
                                     double>(&Drone::getController))
        .def("getCController", &Drone::getCController,
             pybind11::arg("config")         = 1,
             pybind11::arg("enableIntegral") = true)
        .def("getObserver", &Drone::getObserver)
        .def("simulate",
             [](Drone &drone, Drone::Controller &controller,
                PythonDroneReferenceFunction &reference, const DroneState &x0,
                AdaptiveODEOptions odeopt) {
                 return drone.simulate(controller, reference, x0, odeopt);
             })
        .def("simulate",
             [](Drone &drone, Drone::Controller &controller,
                Drone::Observer &observer,
                PythonGaussianNoiseGeneratorInput &randFnW,
                PythonGaussianNoiseGeneratorOutput &randFnV,
                PythonDroneReferenceFunction &reference, const DroneState &x0,
                const AdaptiveODEOptions &odeopt) {
                 return drone.simulate(controller, observer, randFnW, randFnV,
                                       reference, x0, odeopt);
             });

#ifndef EXTERNAL_PY_MODULE
    pydronemodule.def("plot",
                      pybind11::overload_cast<const DronePlottable &, float,
                                              float, int, std::string>(&plot));
    pydronemodule.def("plot",
                      [](const Drone::ControllerSimulationResult &result,
                         float w = 1920, float h = 1080, int colors = 0,
                         std::string title = "") {
                          return plot(result, w, h, colors, title);
                      },
                      pybind11::arg("result"), pybind11::arg("w") = 1920,
                      pybind11::arg("h") = 1080, pybind11::arg("colors") = 0,
                      pybind11::arg("title") = "");
#endif

    pybind11::class_<DronePlottable>(pydronemodule, "DronePlottable")
        .def(pybind11::init<>())
        .def(pybind11::init<const Drone::ControllerSimulationResult &>())
        .def(
            pybind11::init<const Drone::ObserverControllerSimulationResult &>())
        .def(pybind11::init<const DroneLogLoader>())
        .def_readwrite("time", &DronePlottable::time)
        .def_readwrite("sampledTime", &DronePlottable::sampledTime)
        .def_readwrite("states", &DronePlottable::states)
        .def_readwrite("control", &DronePlottable::control)
        .def_readwrite("reference", &DronePlottable::reference)
        .def("toDict", [](const DronePlottable &d) {
            return dronePlottableToPythonDict(d);
        });

    pybind11::class_<DroneLogLoader>(pydronemodule, "DroneLogLoader")
        .def(pybind11::init<const std::string &>())
        .def("slice", &DroneLogLoader::slice)
        .def("__len__", &DroneLogLoader::size);

    pydronemodule.def("eul2quat", eul2quat);
    pydronemodule.def("quatmultiply", quatmultiply);
    pydronemodule.def("quatconjugate", quatconjugate);

    pybind11::class_<PythonGaussianNoiseGeneratorInput>(
        pydronemodule, "GaussianNoiseGeneratorInput")
        .def(pybind11::init<const RowVector<Drone::VecU_t::length()> &>())
        .def("__call__", &PythonGaussianNoiseGeneratorInput::operator())
        .def("reset", &PythonGaussianNoiseGeneratorInput::reset);

    pybind11::class_<PythonGaussianNoiseGeneratorOutput>(
        pydronemodule, "GaussianNoiseGeneratorOutput")
        .def(pybind11::init<const RowVector<Drone::VecY_t::length()> &>())
        .def("__call__", &PythonGaussianNoiseGeneratorOutput::operator())
        .def("reset", &PythonGaussianNoiseGeneratorOutput::reset);
}
