#include "Drone.hpp"
#include "InputSignals.hpp"
#include "PrintCSV.hpp"
#include <ODE/ODEEval.hpp>
#include <iostream>

#include <Matrix/LQR.hpp>

#include "Plot.hpp"

using namespace std;
using Matrices::T;

string outputFile = (string) getenv("HOME") + "/Random/data.csv";

constexpr double f  = 60;
constexpr double Ts = 1.0 / f;

constexpr bool plotSampled = false;

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;

    /* ------ Define drone parameters, model and reference signal ---------- */

    const Drone drone             = {};
    NonLinearFullModel nonlinfull = drone.p;
    const Params &p               = drone.p;

    cout << "n_h = " << drone.p.nh << " rps = " << drone.p.nh * 60 << " rpm"
         << endl;
    cout << "u_h = " << drone.p.uh << endl;

    double u_att_max = 1 - p.nh / p.k1;
    double n_att_max = p.k1 * u_att_max;

    auto Qn =
        1.0 / pow(n_att_max, 2) * ones<1, 3>();  // very small, around 7e-4
    auto Qomega = zeros<1, 3>();
    auto Qq =
        3.0 *
        ones<1, 3>();  // The factor determines how aggressive the controller is
    auto Q = diag(hcat(Qq, Qomega, Qn));
    auto R = 1.0 / pow(u_att_max, 2) * eye<3>();  // around 9

    auto AK = getBlock<1, 10, 1, 10>(drone.A);  // reduced system matrices
    auto BK = getBlock<1, 10, 0, 3>(drone.B);

    auto lqrres = lqr(AK, BK, Q, R);
    auto K_red  = -lqrres.K;
    auto K      = hcat(zeros<3, 1>(), K_red);

    cout << "K = " << K << endl;

    ContinuousLQRController ctrl = {drone.A, drone.B, drone.C, drone.D, K};
    TestReferenceFunction ref    = {};

    /* ------ Simulation options (for ODE solver) -------------------------- */

    AdaptiveODEOptions opt = {};
    opt.t_start            = 0;
    opt.t_end              = 11;
    opt.epsilon            = 1e-6;
    opt.maxiter            = 1e7;

    /* ------ Simulate the drone with the controller ----------------------- */

    ContinuousLQRController::SimulationResult result =
        ctrl.simulate(nonlinfull, ref, x0, opt);

    if (result.resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED) {
        std::cerr << "Error: maximum number of iterations exceeded" << endl;
        opt.t_end = result.time.back();  // TODO
    }
    if (result.resultCode & ODEResultCodes::MINIMUM_STEP_SIZE_REACHED)
        std::cerr << "Error: minimum step size reached" << endl;

    /* ------ Export the simulation result as CSV -------------------------- */

    // Sample and/or interpolate the simulation result using a fixed time step
    auto sampled = sampleODEResult(result, opt.t_start, Ts, opt.t_end);
    printCSV(outputFile, opt.t_start, Ts, sampled);

    /* ------ Plot the simulation result ------------------------------------*/

    // Decide what data to use: either the sampled version or the raw result
    auto t =
        plotSampled ? makeTimeVector(opt.t_start, Ts, opt.t_end) : result.time;
    auto data = plotSampled ? sampled : result.solution;

    // Calculate controller output
    auto u = ctrl.getControlSignal(t, data, ref);

    // Convert the quaternions of the state to euler angles
    vector<EulerAngles> orientation;
    orientation.resize(data.size());
    transform(data.begin(), data.end(), orientation.begin(),
              NonLinearFullModel::stateToEuler);

    plt::subplot(4, 1, 1);
    plotResults(t, orientation, {0, 3}, {"z", "y", "x"}, {"b-", "g-", "r-"},
                "Orientation of drone");
    plt::xlim(opt.t_start, opt.t_end);
    plt::subplot(4, 1, 2);
    plotResults(t, data, {4, 7}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Angular velocity of drone");
    plt::xlim(opt.t_start, opt.t_end);
    plt::subplot(4, 1, 3);
    plotResults(t, data, {7, 10}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Angular velocity of motors");
    plt::xlim(opt.t_start, opt.t_end);
    plt::subplot(4, 1, 4);
    plotResults(t, u, {0, 3}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Control signal");
    plt::xlim(opt.t_start, opt.t_end);

    plt::tight_layout();
    plt::show();

    // ---------------------------------------------------------------------- //

    return EXIT_SUCCESS;
}
