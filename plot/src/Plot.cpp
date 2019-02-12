#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <DronePlot.hpp>
#include <Plot.hpp>

#include <filesystem>

/**
 * @brief   Convert a DronePlottable (a collection of an array of state vectors,
 *          an array of control signal vectors and an array of reference 
 *          vectors) to a Python dictionary (associative array).
 */
static pybind11::dict dronePlottableToPythonDict(const DronePlottable &result);

void plot(const DronePlottable &result) {
    namespace py = pybind11;
    using py::dict;
    using namespace py::literals;
    
    py::scoped_interpreter guard{};

    std::filesystem::path filepath = __FILE__;
    auto path                      = filepath.parent_path();
    auto locals                    = dict{"path"_a = path.c_str()};
    py::exec(R"(
        import sys
        sys.path.append(path)
        import Plot as plotmodule
    )",
             py::globals(), locals);
    auto pm     = locals["plotmodule"];
    auto pyplot = pm.attr("plot");

    auto time  = result.time;
    auto dtime = result.sampledTime;
    auto states = dronePlottableToPythonDict(result);

    pyplot(time, dtime, states);
}

static pybind11::dict dronePlottableToPythonDict(const DronePlottable &result) {
    using namespace pybind11::literals;
    using pybind11::dict;
    return dict{
        "reference_orientation"_a =
            dict{
                "x"_a = Drone::extractSignal(
                    result.reference, &DroneOutput::getOrientationEuler, 0),
                "y"_a = Drone::extractSignal(
                    result.reference, &DroneOutput::getOrientationEuler, 1),
                "z"_a = Drone::extractSignal(
                    result.reference, &DroneOutput::getOrientationEuler, 2),
            },
        "reference_position"_a =
            dict{
                "x"_a = Drone::extractSignal(result.reference,
                                             &DroneOutput::getPosition, 0),
                "y"_a = Drone::extractSignal(result.reference,
                                             &DroneOutput::getPosition, 1),
                "z"_a = Drone::extractSignal(result.reference,
                                             &DroneOutput::getPosition, 2),
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
