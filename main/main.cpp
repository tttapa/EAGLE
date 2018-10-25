#include "Drone.hpp"
#include "InputSignals.hpp"
#include "PrintCSV.hpp"
#include <ODE/ODEEval.hpp>
#include <iostream>

#include <K.hpp>

using namespace std;
using Matrices::T;

string outputFile = (string) getenv("HOME") + "/Random/data.csv";

constexpr double f  = 10000;
constexpr double Ts = 1.0 / f;

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;

    const Drone d                 = {};
    NonLinearFullModel nonlinfull = d.p;
    TestInputFunction u           = {};
    TestReferenceFunction ref     = {};
    
    cout << "K = " << K << endl;

    ContinuousLQRController ctrl = {d.A, d.B, d.C, d.D, K};

    AdaptiveODEOptions opt = {};
    opt.t_start            = 0;
    opt.t_end              = 8;
    opt.epsilon            = 1e-6;
    opt.h_start            = 1e-2;
    opt.h_min              = 1e-6;
    opt.maxiter            = 1e6;

    // NonLinearFullModel::SimulationResult result =
    //     nonlinfull.simulate(u, x0, opt);

    ContinuousLQRController::SimulationResult result =
        ctrl.simulate(nonlinfull, ref, x0, opt);

    if (result.resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED)
        std::cerr << "Error: maximum number of iterations exceeded" << endl;
    if (result.resultCode & ODEResultCodes::MINIMUM_STEP_SIZE_REACHED)
        std::cerr << "Error: minimum step size reached" << endl;

    auto sampled = sampleODEResult(result, opt.t_start, Ts, opt.t_end);
    printCSV(outputFile, opt.t_start, Ts, sampled);

    return 0;
}
