#include <pybind11/stl.h>

#include <PlotHelpers.hpp>

/**
 * @brief   Load the Plot.py Python script and its functions.
 * 
 * @note    Expects the Python interpreter to be active.
 */
static pybind11::module getPythonPlotModuleInitial() {
    using pybind11::dict;
    using namespace pybind11::literals;

    std::filesystem::path filepath = __FILE__;
    auto path                      = filepath.parent_path();
    auto locals                    = dict{"path"_a = path.c_str()};
    pybind11::exec(R"(
        import sys
        sys.path.append(path)
    )",
                   pybind11::globals(), locals);
    pybind11::module pm = pybind11::module::import("Plot");
    return pm;
}

pybind11::module getPythonPlotModule() {
    static pybind11::module pm = getPythonPlotModuleInitial();
    return pm;
}

pybind11::dict dronePlottableToPythonDict(const DronePlottable &result) {
    using namespace pybind11::literals;
    using pybind11::dict;
    return dict{
        "reference_orientation"_a =
            dict{
                "x"_a = Drone::extractSignal(
                    result.reference, &DroneReference::getOrientationEuler, 0),
                "y"_a = Drone::extractSignal(
                    result.reference, &DroneReference::getOrientationEuler, 1),
                "z"_a = Drone::extractSignal(
                    result.reference, &DroneReference::getOrientationEuler, 2),
            },
        "reference_position"_a =
            dict{
                "x"_a = Drone::extractSignal(result.reference,
                                             &DroneReference::getPosition, 0),
                "y"_a = Drone::extractSignal(result.reference,
                                             &DroneReference::getPosition, 1),
                "z"_a = Drone::extractSignal(result.reference,
                                             &DroneReference::getPosition, 2),
            },

        "orientation"_a =
            dict{
                "x"_a = Drone::extractSignal(
                    result.states, &DroneState::getOrientationEuler, 0),
                "y"_a = Drone::extractSignal(
                    result.states, &DroneState::getOrientationEuler, 1),
                "z"_a = Drone::extractSignal(
                    result.states, &DroneState::getOrientationEuler, 2),
            },
        "position"_a =
            dict{
                "x"_a = Drone::extractSignal(result.states,
                                             &DroneState::getPosition, 0),
                "y"_a = Drone::extractSignal(result.states,
                                             &DroneState::getPosition, 1),
                "z"_a = Drone::extractSignal(result.states,
                                             &DroneState::getPosition, 2),
            },

        "angular_velocity"_a =
            dict{
                "x"_a = Drone::extractSignal(
                    result.states, &DroneState::getAngularVelocity, 0),
                "y"_a = Drone::extractSignal(
                    result.states, &DroneState::getAngularVelocity, 1),
                "z"_a = Drone::extractSignal(
                    result.states, &DroneState::getAngularVelocity, 2),
            },
        "linear_velocity"_a =
            dict{
                "x"_a = Drone::extractSignal(result.states,
                                             &DroneState::getVelocity, 0),
                "y"_a = Drone::extractSignal(result.states,
                                             &DroneState::getVelocity, 1),
                "z"_a = Drone::extractSignal(result.states,
                                             &DroneState::getVelocity, 2),
            },

        "torque_motor_velocity"_a =
            dict{
                "x"_a = Drone::extractSignal(result.states,
                                             &DroneState::getMotorSpeed, 0),
                "y"_a = Drone::extractSignal(result.states,
                                             &DroneState::getMotorSpeed, 1),
                "z"_a = Drone::extractSignal(result.states,
                                             &DroneState::getMotorSpeed, 2),
            },
        "thrust_motor_velocity"_a =
            dict{
                "z"_a = Drone::extractSignal(
                    result.states, &DroneState::getThrustMotorSpeed, 0),
            },

        "torque_control"_a =
            dict{
                "x"_a = Drone::extractSignal(
                    result.control, &DroneControl::getAttitudeControl, 0),
                "y"_a = Drone::extractSignal(
                    result.control, &DroneControl::getAttitudeControl, 1),
                "z"_a = Drone::extractSignal(
                    result.control, &DroneControl::getAttitudeControl, 2),
            },
        "thrust_control"_a =
            dict{
                "z"_a = Drone::extractSignal(
                    result.control, &DroneControl::getThrustControl, 0),
            },
    };
}

pybind11::dict
droneAttitudePlottableToPythonDict(const DroneAttitudePlottable &result) {
    using namespace pybind11::literals;
    using pybind11::dict;
    return dict{
        "reference_orientation"_a =
            dict{
                "x"_a = Drone::extractSignal(
                    result.reference, &DroneReference::getOrientationEuler, 0),
                "y"_a = Drone::extractSignal(
                    result.reference, &DroneReference::getOrientationEuler, 1),
                "z"_a = Drone::extractSignal(
                    result.reference, &DroneReference::getOrientationEuler, 2),
            },

        "orientation"_a =
            dict{
                "x"_a = Drone::extractSignal(
                    result.states, &DroneAttitudeState::getOrientationEuler, 0),
                "y"_a = Drone::extractSignal(
                    result.states, &DroneAttitudeState::getOrientationEuler, 1),
                "z"_a = Drone::extractSignal(
                    result.states, &DroneAttitudeState::getOrientationEuler, 2),
            },

        "angular_velocity"_a =
            dict{
                "x"_a = Drone::extractSignal(
                    result.states, &DroneAttitudeState::getAngularVelocity, 0),
                "y"_a = Drone::extractSignal(
                    result.states, &DroneAttitudeState::getAngularVelocity, 1),
                "z"_a = Drone::extractSignal(
                    result.states, &DroneAttitudeState::getAngularVelocity, 2),
            },

        "torque_motor_velocity"_a =
            dict{
                "x"_a = Drone::extractSignal(
                    result.states, &DroneAttitudeState::getMotorSpeed, 0),
                "y"_a = Drone::extractSignal(
                    result.states, &DroneAttitudeState::getMotorSpeed, 1),
                "z"_a = Drone::extractSignal(
                    result.states, &DroneAttitudeState::getMotorSpeed, 2),
            },

        "torque_control"_a =
            dict{
                "x"_a = Drone::extractSignal(
                    result.control, &DroneAttitudeControl::getAttitudeControl,
                    0),
                "y"_a = Drone::extractSignal(
                    result.control, &DroneAttitudeControl::getAttitudeControl,
                    1),
                "z"_a = Drone::extractSignal(
                    result.control, &DroneAttitudeControl::getAttitudeControl,
                    2),
            },
    };
}