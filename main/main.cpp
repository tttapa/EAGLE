#include "Config.hpp"
#include "InputSignals.hpp"
#include "Plot.hpp"
#include "PrintCSV.hpp"
#include "StepResponseAnalyzer.hpp"
#include <Drone/MotorControl.hpp>

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

    DroneState x0 = drone.getStableState();

    TestReferenceFunction ref = {};

    /* ------ Simulate the drone with the controller ------------------------ */
    auto result = drone.simulate(controller, ref, x0, Config::odeopt);
    result.resultCode.verbose();

// #ifdef PLOT_CONTROLLER
#if 1
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

    /* ------ Plot the step response ---------------------------------------- */
    auto attctrl =
        drone.getAttitudeController(Config::Attitude::Q, Config::Attitude::R);
    auto attmodel = drone.getAttitudeModel();

    Quaternion q_ref = eul2quat({0, M_PI / 8, M_PI / 8});
    DroneAttitudeOutput y_ref;
    y_ref.setOrientation(q_ref);
    AdaptiveODEOptions opt                           = Config::odeopt;
    opt.t_end                                        = 2.5;
    ConstantTimeFunctionT<ColVector<Ny_att>> y_ref_f = {y_ref};

    const DroneAttitudeState attx0  = x0.getAttitude();
    const DroneAttitudeOutput atty0 = attmodel.getOutput(attx0, {0});
    const Quaternion q0             = atty0.getOrientation();

    constexpr double factor = 0.01;  // 1% criterium

    StepResponseAnalyzerPlotter<4> stepAnalyzerPlt = {q_ref, factor, q0, false};
    auto f = [&](double t, const ColVector<Nx_att> &x,
                 const ColVector<Nu_att> &u) {
        DroneAttitudeOutput y = attmodel.getOutput(x, u);
        return stepAnalyzerPlt(t, y.getOrientation());
    };

    ODEResultCode resultCode =
        attmodel.simulateRealTime(attctrl, y_ref_f, attx0, opt, f);
    resultCode.verbose();

    Quaternion risetimes   = stepAnalyzerPlt.getResult().risetime;
    Quaternion overshoots  = stepAnalyzerPlt.getResult().overshoot;
    Quaternion settletimes = stepAnalyzerPlt.getResult().settletime;

    cout << ANSIColors::whiteb << endl
         << "Rise time:   \t" << asrowvector(risetimes) << endl
         << "Overshoot:   \t" << asrowvector(overshoots) << endl
         << "Settle time: \t" << asrowvector(settletimes) << endl
         << ANSIColors::reset;

    stepAnalyzerPlt.plot({1, 4}, {"q1", "q2", "q3"}, {"r", "g", "b"},
                         "Step Response of Attitude Controller");
    plt::tight_layout();
    plt::xlabel("time [s]");
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
