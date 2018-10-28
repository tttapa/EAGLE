#include "InputSignals.hpp"
#include "PrintCSV.hpp"
#include <Matrix/DLQR.hpp>
#include <Matrix/LQR.hpp>
#include <Model/System.hpp>
#include <ODE/ODEEval.hpp>
#include <iostream>

#include "Config.hpp"

#include "Plot.hpp"

using namespace std;
using namespace Config;

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;

    /* ------ Drone full non-linear model ----------------------------------- */
    NonLinearFullModel nonlinfull = drone.p;  // Using the drone's parameters

    // TODO: use zero-order hold?
    CTLTISystem<10, 3, 7> linfull = {drone.A, drone.B, drone.C, drone.D};
    auto discretized = linfull.discretize(Ts, DiscretizationMethod::Bilinear);

    /* ------ Reduced system matrices --------------------------------------- */
    auto A_red = getBlock<1, 10, 1, 10>(drone.A);
    auto B_red = getBlock<1, 10, 0, 3>(drone.B);

    auto Ad_red = getBlock<1, 10, 1, 10>(discretized.A);
    auto Bd_red = getBlock<1, 10, 0, 3>(discretized.B);

    /* ------ Design controller --------------------------------------------- */
    auto K_red = -lqr(A_red, B_red, Q, R).K;
    auto K = hcat(zeros<3, 1>(), K_red);  // add column of zeros for full model
    ContinuousLQRController ctrl = {drone.A, drone.B, drone.C, drone.D, K};

    auto Kd_red                 = -dlqr(Ad_red, Bd_red, Q, R).K;
    auto Kd                     = hcat(zeros<3, 1>(), Kd_red);
    DiscreteLQRController dctrl = {discretized, Kd};

    /* ------ Reference function -------------------------------------------- */
    TestReferenceFunction ref = {};

    /* ------ Simulate the drone with the controller ----------------------- */
    LQRController::SimulationResult result =
        ctrl.simulate(nonlinfull, ref, x0, odeopt);

    // LQRController::SimulationResult result =
    //     dctrl.simulate(nonlinfull, ref, x0, odeopt);

    double t_end = odeopt.t_end;
    if (result.resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED) {
        std::cerr << "Error: maximum number of iterations exceeded" << endl;
        t_end = result.time.back();  // TODO
    }
    if (result.resultCode & ODEResultCodes::MINIMUM_STEP_SIZE_REACHED)
        std::cerr << "Error: minimum step size reached" << endl;

    /* ------ Export the simulation result as CSV -------------------------- */

    // Sample and/or interpolate the simulation result using a fixed time step
    auto sampled = sampleODEResult(result, odeopt.t_start, Ts, t_end);
    // Export to the given output file
    printCSV(outputFile, odeopt.t_start, Ts, sampled);

    /* ------ Plot the simulation result ------------------------------------*/

    // Decide what data to use: either the sampled version or the raw result
    auto t =
        plotSampled ? makeTimeVector(odeopt.t_start, Ts, t_end) : result.time;
    auto data = plotSampled ? sampled : result.solution;

    // Calculate controller output
    auto u = ctrl.getControlSignal(t, data, ref);

    // Convert the quaternions of the state to euler angles
    vector<EulerAngles> orientation;
    orientation.resize(data.size());
    transform(data.begin(), data.end(), orientation.begin(),
              NonLinearFullModel::stateToEuler);

    // Plot all results
    plt::subplot(4, 1, 1);
    plotResults(t, orientation, {0, 3}, {"z", "y", "x"}, {"b-", "g-", "r-"},
                "Orientation of drone");
    plt::xlim(odeopt.t_start, odeopt.t_end);
    plt::subplot(4, 1, 2);
    plotResults(t, data, {4, 7}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Angular velocity of drone");
    plt::xlim(odeopt.t_start, odeopt.t_end);
    plt::subplot(4, 1, 3);
    plotResults(t, data, {7, 10}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Angular velocity of motors");
    plt::xlim(odeopt.t_start, odeopt.t_end);
    plt::subplot(4, 1, 4);
    plotResults(t, u, {0, 3}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Control signal");
    plt::xlim(odeopt.t_start, odeopt.t_end);

    plt::tight_layout();
    plt::show();

    /* ---------------------------------------------------------------------- */

    return EXIT_SUCCESS;
}
