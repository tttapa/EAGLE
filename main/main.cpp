#include "Config.hpp"
#include "InputSignals.hpp"
#include "Plot.hpp"
#include "PrintCSV.hpp"
#include <ArgParser/ArgParser.hpp>
#include <Drone/MotorControl.hpp>
#include <ODE/ODEEval.hpp>
#include <PlotStepReponse.hpp>
#include <Util/Degrees.hpp>
#include <iostream>

using namespace std;

int main(int argc, char const *argv[]) {
    filesystem::path loadPath = Config::loadPath;
    filesystem::path outPath  = "";
    double steperrorfactor    = Config::steperrorfactor;
    size_t px_x               = Config::px_x;
    size_t px_y               = Config::px_y;

    ArgParser parser;
    parser.add("--out", "-o", [&](const char *argv[]) {
        outPath = argv[1];
        cout << "Setting output path to: " << argv[1] << endl;
    });
    parser.add("--load", "-l", [&](const char *argv[]) {
        loadPath = argv[1];
        cout << "Setting load path to: " << argv[1] << endl;
    });
    parser.add("--width", "-w", [&](const char *argv[]) {
        px_x = strtoul(argv[1], nullptr, 10);
        cout << "Setting the image width to: " << px_x << endl;
    });
    parser.add("--height", "-h", [&](const char *argv[]) {
        px_y = strtoul(argv[1], nullptr, 10);
        cout << "Setting the image height to: " << px_y << endl;
    });
    cout << ANSIColors::blue;
    parser.parse(argc, argv);
    cout << ANSIColors::reset << endl;

    Drone drone = loadPath;

    Drone::Controller controller =
        drone.getController(Config::Attitude::Q, Config::Attitude::R,
                            Config::Altitude::K_p, Config::Altitude::K_i);

    // Drone::Observer observer = drone.getObserver(
    //     Config::Attitude::varDynamics, Config::Attitude::varSensors,
    //     Config::Altitude::varDynamics, Config::Altitude::varSensors);

    DroneState x0 = drone.getStableState();

    TestReferenceFunction ref = {};

    /* ------ Simulate the drone with the controller ------------------------ */
    auto result = drone.simulate(controller, ref, x0, Config::odeopt);
    result.resultCode.verbose();

    /* ------ Plot the simulation result ------------------------------------ */
    if (Config::plotSimulationResult) {
        plt::figure_size(px_x, px_y);
        plotDrone(result);
        if (!Config::plotAllAtOnce)
            plt::show();
    }

    /* ------ Plot the motor control signals -------------------------------- */
    if (Config::plotMotorControls) {
        auto motorControl = convertControlSignalToMotorOutputs(result.control);
        plt::figure_size(px_x, px_y);
        plotVectors(result.sampledTime, motorControl, {0, 4},
                    {"motor 1", "motor 2", "motor 3", "motor 4"},
                    {"r", "g", "b", "orange"}, "Motor PWM control");
        plt::tight_layout();
        if (!Config::plotAllAtOnce)
            plt::show();
    }

    /* ------ Plot the step response ---------------------------------------- */
    if (Config::plotStepResponse) {
        plt::figure_size(px_x, px_y);
        Quaternion q_ref = eul2quat({0, 10_deg, 10_deg});
        plotStepResponseAttitude(drone, Config::Attitude::Q,
                                 Config::Attitude::R, steperrorfactor, q_ref,
                                 Config::odeopt);
        plt::tight_layout();
        if (!Config::plotAllAtOnce)
            plt::show();
    }

    if (Config::plotAllAtOnce)
        plt::show();

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
