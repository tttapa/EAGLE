#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <DronePlot.hpp>
#include <Plot.hpp>

#include <filesystem>

/**
 * @brief   Convert a DronePlottable (a collection of an array of state vectors,
 *          an array of control signal vectors and an array of reference 
 *          vectors) to a Python dictionary (associative array).
 * 
 * @note    Expects the Python interpreter to be active.
 */
static pybind11::dict dronePlottableToPythonDict(const DronePlottable &result);

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
    )", pybind11::globals(), locals);
    pybind11::module pm = pybind11::module::import("Plot");
    return pm;
}

/**
 * @brief   Load the Plot.py Python script and its functions, or return the
 *          module that was loaded already.
 * 
 * @note    Expects the Python interpreter to be active.
 */
static pybind11::module getPythonPlotModule() {
    static pybind11::module pm = getPythonPlotModuleInitial();
    return pm;
}

pybind11::object plot(const DronePlottable &result, float w, float h) {
    auto pm     = getPythonPlotModule();
    auto pyplot = pm.attr("plot");

    auto time   = result.time;
    auto dtime  = result.sampledTime;
    auto states = dronePlottableToPythonDict(result);

    return pyplot(time, dtime, states, w, h);
}

void show(pybind11::object fig) {
    auto pm     = getPythonPlotModule();
    auto pyshow = pm.attr("show");

    pyshow(fig);
} 

void save(pybind11::object fig, std::filesystem::path filename) {
    auto pm = getPythonPlotModule();
    auto pysave = pm.attr("save");

    pysave(fig, filename.c_str());
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
