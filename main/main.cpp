#include "NonLinearFullModel.hpp"
#include "Params.hpp"
#include "PrintCSV.hpp"
#include <ODE/ODEEval.hpp>
#include <iostream>

using namespace std;
using Matrices::T;

string outputFile = (string) getenv("HOME") + "/Random/data.csv";

constexpr double Ts = 1.0 / 30;

struct TestInputFunction : public NonLinearFullModel::InputFunction {
    NonLinearFullModel::VecU_t operator()(double t) override {
        return {{
            {0},
            {0},
            {-0.1 * (t < 2) + 0.2 * (t < 3) - 0.1 * (t < 5)},
        }};
    }
};

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;
    const Params p                = {};
    NonLinearFullModel nonlinfull = p;
    TestInputFunction u           = {};
    NonLinearFullModel::VecX_t x0 = {{
        {1},    // q0
        {0},    // q1
        {0},    // q2
        {0},    // q3 ____
        {0.1},  // ωx
        {0},    // ωy
        {0},    // ωz ____
        {0},    // nx
        {0},    // ny
        {0},    // nz
    }};

    AdaptiveODEOptions opt = {};
    opt.t_start            = 0;
    opt.t_end              = 20;
    opt.epsilon            = 1e-6;
    opt.h_start            = 1e-2;
    opt.h_min              = 1e-6;
    opt.maxiter            = 1e6;

    NonLinearFullModel::SimulationResult result =
        nonlinfull.simulate(u, x0, opt);

    if (result.resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED)
        std::cerr << "Error: maximum number of iterations exceeded" << endl;
    if (result.resultCode & ODEResultCodes::MINIMUM_STEP_SIZE_REACHED)
        std::cerr << "Error: minimum step size reached" << endl;

    auto sampled = sampleODEResult(result, opt.t_start, Ts, opt.t_end);
    printCSV(outputFile, opt.t_start, Ts, sampled);

    return 0;
}
