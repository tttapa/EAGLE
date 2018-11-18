#include "InputSignals.hpp"
#include "PrintCSV.hpp"
#include <iostream>

#include "ANSIColors.hpp"
#include "Config.hpp"
#include "Plot.hpp"

#include "Generated/GeneratedController.hpp"

#include "CodeGen/CodeGen.hpp"

#include <debug.hpp>

using namespace std;
using namespace Config;

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;

    Drone drone                  = {std::filesystem::path(home) /
                   "PO-EAGLE/Groups/ANC/MATLAB/Codegen"};
    Drone::Controller controller = {
        drone.getClampedAttitudeController(clampMin, clampMax, Q, R), drone.uh};

    Drone::VecX_t x0 = drone.getInitialState();

    /* ------ Reference function -------------------------------------------- */
    TestReferenceFunction ref = {};

    /* ------ Simulate the drone with the controller ------------------------ */
    auto result = drone.simulate(controller, ref, x0, odeopt);

    double t_end = odeopt.t_end;
    if (result.resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED) {
        std::cerr << ANSIColors::redb
                  << "Error: maximum number of iterations exceeded"
                  << ANSIColors::reset << endl;
        t_end = result.time.back();
    }
    if (result.resultCode & ODEResultCodes::MINIMUM_STEP_SIZE_REACHED)
        std::cerr << ANSIColors::yellow << "Warning: minimum step size reached"
                  << ANSIColors::reset << endl;

    /* ------ Plot the simulation result ------------------------------------ */
    // Time and state solutions
    auto time   = result.time;
    auto states = result.solution;

    // Convert the quaternions of the state to euler angles
    vector<EulerAngles> orientation = Drone::statesToEuler(states);

    // Convert the quaternions of the reference to euler angles
    vector<EulerAngles> refOrientation;
    refOrientation.resize(time.size());
    transform(
        time.begin(), time.end(), refOrientation.begin(),
        [&ref](double t) { return quat2eul(getBlock<0, 4, 0, 1>(ref(t))); });

#ifdef PLOT_CONTROLLER

    // Plot all results
    plt::subplot(5, 2, 1);
    plotResults(time, refOrientation, {0, 3}, {"z", "y", "x"}, {"b-", "g-", "r-"},
                "Reference orientation");
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
    plt::subplot(5, 2, 3);
    plotResults(time, orientation, {0, 3}, {"z", "y", "x"}, {"b-", "g-", "r-"},
                "Orientation of drone");
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
    plt::subplot(5, 2, 5);
    plotResults(time, states, {4, 7}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Angular velocity of drone");
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
    plt::subplot(5, 2, 7);
    plotResults(time, states, {7, 10}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Angular velocity of torque motors");
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
    plt::subplot(5, 2, 4);
    plotResults(time, states, {13, 16}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Position");
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
    plt::subplot(5, 2, 6);
    plotResults(time, states, {10, 13}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Velocity");
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
    plt::subplot(5, 2, 8);
    plotResults(time, states, {16}, {"z'"}, {"r-"},
                "Angular velocity of thrust motor");
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);

    // TODO
    // plotResults(time, u, {0, 3}, {"x", "y", "z"}, {"r-", "g-", "b-"},
    //             "Control signal");
    // plotResults(generatedTime, generatedU, {0, 3},
    //             {"generated x", "generated y", "generated z"},
    //             {"r--", "g--", "b--"});
    // plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);

    plt::tight_layout();
    plt::show();

#endif

    return EXIT_SUCCESS;
}
