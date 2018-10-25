#include "Controller.hpp"
#include "NonLinearFullModel.hpp"
#include "Params.hpp"
#include "PrintCSV.hpp"
#include <ODE/ODEEval.hpp>
#include <iostream>

using namespace std;
using Matrices::T;

string outputFile = (string) getenv("HOME") + "/Random/data.csv";

constexpr double f  = 1000;
constexpr double Ts = 1.0 / f;

struct TestInputFunction : public NonLinearFullModel::InputFunction {
    NonLinearFullModel::VecU_t operator()(double t) override {
        return {{
            {0},
            {0},
            {-0.1 * (t < 2) + 0.2 * (t < 3) - 0.1 * (t < 5)},
        }};
    }
};

struct TestReferenceFunction
    : public ContinuousLQRController::ReferenceFunction {
    ContinuousLQRController::VecR_t operator()(double t) override {
        ContinuousLQRController::VecR_t ref = {1};
        if (t > 1 && t < 2)
            assignBlock<0, 4, 0, 1>(ref) = q1;
        return ref;
    }
    const Quaternion q1 = eul2quat({M_PI / 4, 0, 0});
};

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;
    const Params p                = {};
    NonLinearFullModel nonlinfull = p;
    TestInputFunction u           = {};
    TestReferenceFunction ref     = {};
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

    constexpr size_t Nx = 10;
    constexpr size_t Nu = 3;
    constexpr size_t Ny = 7;

    /* Matrix<double, Nx, Nx> A     = {}
    assignBlock<1, 4, 4, 7>(A)   = 0.5 * eye<double, 3>();
    assignBlock<4, 7, 7, 10>(A)  = p.gamma_n;
    assignBlock<7, 10, 7, 10>(A) = -p.k2 * eye<double, 3>(); */
    Matrix<double, Nx, Nx> A =
        vcat(zeros<double, 1, 10>(),
             hcat(zeros<double, 3, 4>(), 0.5 * eye<double, 3>(),
                  zeros<double, 3, 3>()),
             hcat(zeros<double, 3, 7>(), p.gamma_n),
             hcat(zeros<double, 3, 7>(), -p.k2 * eye<double, 3>()));

    cout << "A = " << A << endl;

    Matrix<double, Nx, Nu> B    = {};
    assignBlock<4, 7, 0, 3>(B)  = p.gamma_u;
    assignBlock<7, 10, 0, 3>(B) = p.k2 * p.k1 * eye<double, 3>();

    cout << "B = " << B << endl;

    Matrix<double, Ny, Nx> C   = {};
    assignBlock<0, 7, 0, 7>(C) = eye<double, 7>();

    cout << "C = " << C << endl;

    Matrix<double, Ny, Nu> D = {};

    cout << "D = " << D << endl;

    Matrix<double, Nu, Nx> K = {{
        {0, -10, 0, 0, -1.075087, 0, 0, -0.034567, 0, 0},
        {0, 0, -10, 0, 0, -1.077362, 0, 0, -0.033435, 0},
        {0, 0, 0, -10, 0, 0, -1.029377, 0, 0, 0.007721},
    }};

    cout << "K = " << K << endl;

    ContinuousLQRController ctrl = {A, B, C, D, K};

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
