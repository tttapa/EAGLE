#include "Config.hpp"
#include "DroneLogLoader.hpp"
#include "InputSignals.hpp"
#include "Plot.hpp"
#include "PrintCSV.hpp"
#include <ArgParser/ArgParser.hpp>
#include <Drone/MotorControl.hpp>
#include <ODE/ODEEval.hpp>
#include <PlotStepReponse.hpp>
#include <Util/Degrees.hpp>
#include <Util/MeanSquareError.hpp>
#include <iostream>

using namespace std;

int main(int argc, char const *argv[]) {
    /* ------ Parse command line arguments ---------------------------------- */
    filesystem::path loadPath    = Config::loadPath;
    filesystem::path outPath     = "";
    double steperrorfactor       = Config::steperrorfactor;
    size_t px_x                  = Config::px_x;
    size_t px_y                  = Config::px_y;
    filesystem::path logLoadPath = Config::logLoadPath;

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
    parser.add("--load-log", [&](const char *argv[]) {
        logLoadPath = argv[1];
        cout << "Setting the log load path to: " << logLoadPath << endl;
    });
    cout << ANSIColors::blue;
    parser.parse(argc, argv);
    cout << ANSIColors::reset << endl;

    /* ------ Load drone data and get models, controllers and observers ----- */

    Drone drone = loadPath;

    Drone::Controller controller = drone.getController(
        Config::Attitude::Q, Config::Attitude::R, Config::Altitude::K_pi,
        Config::Altitude::maxIntegralInfluence);

    Drone::Controller ccontroller = drone.getCController();

    // Drone::Observer observer = drone.getObserver(
    //     Config::Attitude::varDynamics, Config::Attitude::varSensors,
    //     Config::Altitude::varDynamics, Config::Altitude::varSensors);

    DroneState x0 = drone.getStableState();

    TestReferenceFunction ref = {};

    /* ------ Simulate the drone with the controller ------------------------ */
    auto result = drone.simulate(controller, ref, x0, Config::odeopt);
    result.resultCode.verbose();

    /* ------ Simulate the drone with the generated C controller ------------ */
    auto cresult = drone.simulate(ccontroller, ref, x0, Config::odeopt);
    cresult.resultCode.verbose();

    /* ------ Compare the C controller to the C++ controller ---------------- */
    auto sampled  = sampleODEResult(result, Config::odeopt.t_start,
                                   controller.Ts, Config::odeopt.t_end);
    auto csampled = sampleODEResult(cresult, Config::odeopt.t_start,
                                    ccontroller.Ts, Config::odeopt.t_end);

    bool correctCController = true;
    if (sampled.size() != csampled.size()) {
        cerr << ANSIColors::redb
             << "The C controller simulation result length is not correct"
             << ANSIColors::reset << endl;
        correctCController = false;
    } else if (meanSquareError(sampled.begin(), sampled.end(),
                               csampled.begin()) > 1e-15) {
        cerr << ANSIColors::redb
             << "The C controller simulation result is not correct. MSE = "
             << meanSquareError(sampled.begin(), sampled.end(),
                                csampled.begin())
             << ", max error squared = "
             << maxSquareError(sampled.begin(), sampled.end(), csampled.begin())
             << ANSIColors::reset << endl;
        correctCController = false;
        vector<double> errorSq(sampled.size());
        transform(sampled.begin(), sampled.end(), csampled.begin(),
                  errorSq.begin(),
                  [](auto x, auto cx) { return (x - cx) * (x - cx); });
        auto sampledTime = makeTimeVector(Config::odeopt.t_start, controller.Ts,
                                          Config::odeopt.t_end);
        plt::figure_size(px_x, px_y);
        plt::plot(sampledTime, errorSq);
        plt::title("$\\left|x_{\\mathrm{C++}} - x_{C}\\right|^2$");
        plt::tight_layout();
        if (!Config::plotAllAtOnce)
            plt::show();
    } else {
        cout << ANSIColors::greenb
             << "✔   The C controller simulation result is correct"
             << ANSIColors::reset << endl;
    }

    /* ------ Test the C Attitude controller -------------------------------- */

    Attitude::LQRController attCtrl =
        drone.getAttitudeController(Config::Attitude::Q, Config::Attitude::R);
    Attitude::CLQRController cattCtrl = {drone.Ts_att};

    Attitude::LQRController::VecX_t x =
        vcat(eul2quat({25_deg, 26_deg, 22_deg}), ColVector<3>{2, 4, 3},
             ColVector<3>{20, 30, 40});
    Attitude::LQRController::VecR_t r =
        vcat(eul2quat({2_deg, 3_deg, -2_deg}), zeros<3, 1>());

    Attitude::LQRController::VecU_t u   = attCtrl(x, r);
    Attitude::CLQRController::VecU_t cu = cattCtrl(x, r);

    if (!isAlmostEqual(u, cu, 1e-10)) {
        cerr << ANSIColors::redb
             << "error u  = " << asrowvector(u - cu, ", ", 10)
             << ANSIColors::reset << endl;
        correctCController = false;
    }

    /* ------ Plot the simulation result ------------------------------------ */

    if (Config::plotSimulationResult) {
        plt::figure_size(px_x, px_y);
        plotDrone(result);
        plt::axhline(drone.uh, "--", "r");
        if (!Config::plotAllAtOnce)
            plt::show();
    }

    /* ------ Plot the C simulation result ---------------------------------- */

    if (Config::plotCSimulationResult) {
        plt::figure_size(px_x, px_y);
        plotDrone(cresult);
        plt::axhline(drone.uh, "--", "r");
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
        Quaternion q_ref = eul2quat({0, 0, 10_deg});
        plotStepResponseAttitude(
            drone, Config::Attitude::Q, Config::Attitude::R, steperrorfactor,
            q_ref, Config::odeopt, "$(0\\degree, 0\\degree, 10\\degree)$");
        plt::tight_layout();
        if (!Config::plotAllAtOnce)
            plt::show();

        plt::figure_size(px_x, px_y);
        q_ref = eul2quat({0, 10_deg, 10_deg});
        plotStepResponseAttitude(
            drone, Config::Attitude::Q, Config::Attitude::R, steperrorfactor,
            q_ref, Config::odeopt, "$(0\\degree, 10\\degree, 10\\degree)$");
        plt::tight_layout();
        if (!Config::plotAllAtOnce)
            plt::show();
    }

    /* ------ Compare two controllers --------------------------------------- */

    if (Config::Attitude::Compare::compare) {
        auto ctrl1 = drone.getController(
            Config::Attitude::Compare::Q1, Config::Attitude::Compare::R1,
            Config::Altitude::K_pi, Config::Altitude::maxIntegralInfluence);
        auto ctrl2 = drone.getController(
            Config::Attitude::Compare::Q2, Config::Attitude::Compare::R2,
            Config::Altitude::K_pi, Config::Altitude::maxIntegralInfluence);
        auto result1 = drone.simulate(ctrl1, ref, x0, Config::odeopt);
        result1.resultCode.verbose();
        auto result2 = drone.simulate(ctrl2, ref, x0, Config::odeopt);
        result2.resultCode.verbose();
        plt::figure_size(px_x, px_y);
        plotDrone(result1, 0);
        plotDrone(result2, 1);
        if (!Config::plotAllAtOnce)
            plt::show();

        plt::figure_size(px_x, px_y);
        Quaternion q_ref = eul2quat({0, 30_deg, 30_deg});
        plotStepResponseAttitude(drone, Config::Attitude::Compare::Q1,
                                 Config::Attitude::Compare::R1, steperrorfactor,
                                 q_ref, Config::odeopt,
                                 "$(0\\degree, 30\\degree, 30\\degree)$", 0);
        plotStepResponseAttitude(drone, Config::Attitude::Compare::Q2,
                                 Config::Attitude::Compare::R2, steperrorfactor,
                                 q_ref, Config::odeopt,
                                 "$(0\\degree, 30\\degree, 30\\degree)$", 1);
        plt::tight_layout();
        if (!Config::plotAllAtOnce)
            plt::show();

        plt::figure_size(px_x, px_y);
        q_ref = eul2quat({0, 0, 10_deg});
        plotStepResponseAttitude(drone, Config::Attitude::Compare::Q1,
                                 Config::Attitude::Compare::R1, steperrorfactor,
                                 q_ref, Config::odeopt,
                                 "$(0\\degree, 0\\degree, 10\\degree)$", 0);
        plotStepResponseAttitude(drone, Config::Attitude::Compare::Q2,
                                 Config::Attitude::Compare::R2, steperrorfactor,
                                 q_ref, Config::odeopt,
                                 "$(0\\degree, 0\\degree, 10\\degree)$", 1);
        plt::tight_layout();
        if (!Config::plotAllAtOnce)
            plt::show();
    }

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

    /* ------ Plot the logged actual drone data ----------------------------- */

    if (Config::plotLoggedDroneData) {
        DroneLogLoader dronelog = logLoadPath;
        if (dronelog) {
            plt::figure_size(px_x, px_y);
            plotDrone(dronelog);
            if (!Config::plotAllAtOnce)
                plt::show();
        }
    }

    /* ------ Plot all figures ---------------------------------------------- */

    if (Config::plotAllAtOnce)
        plt::show();

    cout << "Done." << endl;

    return correctCController ? EXIT_SUCCESS : EXIT_FAILURE;
}
