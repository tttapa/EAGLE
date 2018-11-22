#include "Config.hpp"
#include "InputSignals.hpp"
#include "MotorControl.hpp"
#include "Plot.hpp"
#include "PrintCSV.hpp"

#include <ODE/ODEEval.hpp>

#include <iostream>

using namespace std;

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;

    Drone drone = Config::loadPath;

    Drone::Controller controller =
        drone.getController(Config::Attitude::Q, Config::Attitude::R,
                            Config::Altitude::K_p, Config::Altitude::K_i);

    Drone::Observer observer = drone.getObserver(
        Config::Attitude::varDynamics, Config::Attitude::varSensors,
        Config::Altitude::varDynamics, Config::Altitude::varSensors);

    Drone::VecX_t x0 = drone.getStableState();

    TestReferenceFunction ref = {};

    /* ------ Simulate the drone with the controller ------------------------ */
    auto result = drone.simulate(controller, ref, x0, Config::odeopt);
    result.resultCode.verbose();

#ifdef PLOT_CONTROLLER

    /* ------ Plot the simulation result ------------------------------------ */
    plotDrone(result);
    plt::show();

    auto motorControl = convertControlSignalToMotorOutputs(result.control);

    plt::figure();
    plotVectors(result.sampledTime, motorControl, {0, 4},
                {"motor 1", "motor 2", "motor 3", "motor 4"},
                {"r", "g", "b", "orange"}, "Motor PWM control");
    plt::tight_layout();
    plt::show();

#endif

    /* ------ Export the simulation result as CSV --------------------------- */

    if (Config::exportCSV) {
        // Sample/interpolate the simulation result using a fixed time step
        auto sampled = sampleODEResult(result, Config::odeopt.t_start,
                                       Config::CSV_Ts, Config::odeopt.t_end);
        // Extract Quaternion orientation
        vector<Quaternion> sampledOrientation =
            Drone::extractState(sampled, &DroneState::getOrientation);
        // Extract position
        vector<ColVector<3>> sampledLocation =
            Drone::extractState(sampled, &DroneState::getPosition);
        // Export to the given output file
        printCSV(Config::rotationCSVFile, 0.0, Config::CSV_Ts,
                 sampledOrientation);
        printCSV(Config::locationCSVFile, 0.0, Config::CSV_Ts, sampledLocation);
    }

    return EXIT_SUCCESS;
}
