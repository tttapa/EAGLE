#include "Drone.hpp"
#include "InputSignals.hpp"
#include "K.hpp"
#include "PrintCSV.hpp"
#include <ODE/ODEEval.hpp>
#include <iostream>

#include "Plot.hpp"

using namespace std;
using Matrices::T;

string outputFile = (string) getenv("HOME") + "/Random/data.csv";

constexpr double f  = 60;
constexpr double Ts = 1.0 / f;

constexpr bool plotSampled = false;

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;

    const Drone drone             = {};
    NonLinearFullModel nonlinfull = drone.p;
    TestReferenceFunction ref     = {};

    ContinuousLQRController ctrl = {drone.A, drone.B, drone.C, drone.D, K};

    AdaptiveODEOptions opt = {};
    opt.t_start            = 0;
    opt.t_end              = 22;
    opt.epsilon            = 1e-5;
    opt.maxiter            = 10e7;

    ContinuousLQRController::SimulationResult result =
        ctrl.simulate(nonlinfull, ref, x0, opt);

    if (result.resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED)
        std::cerr << "Error: maximum number of iterations exceeded" << endl;
    if (result.resultCode & ODEResultCodes::MINIMUM_STEP_SIZE_REACHED)
        std::cerr << "Error: minimum step size reached" << endl;

    auto sampled = sampleODEResult(result, opt.t_start, Ts, opt.t_end);
    printCSV(outputFile, opt.t_start, Ts, sampled);

    auto t =
        plotSampled ? makeTimeVector(opt.t_start, Ts, opt.t_end) : result.time;
    auto data = plotSampled ? sampled : result.solution;

    vector<EulerAngles> orientation;
    orientation.resize(data.size());
    transform(data.begin(), data.end(), orientation.begin(),
              NonLinearFullModel::stateToEuler);

    matplotlibcpp::subplot(3, 1, 1);
    plotResults(t, orientation, {0, 3}, {"z", "y", "x"}, {"b-", "g-", "r-"},
                "Orientation of drone");
    matplotlibcpp::xlim(opt.t_start, opt.t_end);
    matplotlibcpp::subplot(3, 1, 2);
    plotResults(t, data, {4, 7}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Angular velocity of drone");
    matplotlibcpp::xlim(opt.t_start, opt.t_end);
    matplotlibcpp::subplot(3, 1, 3);
    plotResults(t, data, {7, 10}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Angular velocity of motors");
    matplotlibcpp::xlim(opt.t_start, opt.t_end);
    matplotlibcpp::tight_layout();
    matplotlibcpp::show();

    return EXIT_SUCCESS;
}
