#include "ContinuousLQRController.hpp"
#include "NonLinearFullModel.hpp"
#include "Params.hpp"
#include "PrintCSV.hpp"
#include <ODE/ODEEval.hpp>
#include <iostream>

using namespace std;
using Matrices::T;

string outputFile = (string) getenv("HOME") + "/Random/data.csv";

constexpr double f  = 10000;
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


    /* A =  [  0   0   0   0   0   0   0   0   0   0  ]
            [  0   0   0   0  ┌─────────┐  0   0   0  ]
            [  0   0   0   0  │  0.5 I3 │  0   0   0  ]
            [  0   0   0   0  └─────────┘  0   0   0  ]
            [  0   0   0   0   0   0   0  ┌─────────┐ ]
            [  0   0   0   0   0   0   0  │   Γ_n   │ ]
            [  0   0   0   0   0   0   0  └─────────┘ ]
            [  0   0   0   0   0   0   0  ┌─────────┐ ]
            [  0   0   0   0   0   0   0  │ -k2 I3  │ ]
            [  0   0   0   0   0   0   0  └─────────┘ ] */
    Matrix<double, Nx, Nx> A =
        vcat(zeros<double, 1, 10>(),
             hcat(zeros<double, 3, 4>(), 0.5 * eye<double, 3>(),
                  zeros<double, 3, 3>()),
             hcat(zeros<double, 3, 7>(), p.gamma_n),
             hcat(zeros<double, 3, 7>(), -p.k2 * eye<double, 3>()));

    cout << "A = " << A << endl;

    /* B =  [  0   0   0  ]
            [  0   0   0  ]
            [  0   0   0  ]
            [  0   0   0  ]
            [ ┌─────────┐ ]
            [ │   Γ_u   │ ]
            [ └─────────┘ ]
            [ ┌─────────┐ ]
            [ │ k1k2 I3 │ ]
            [ └─────────┘ ] */
    Matrix<double, Nx, Nu> B =
        vcat(zeros<double, 4, 3>(), p.gamma_u, p.k2 * p.k1 * eye<double, 3>());

    cout << "B = " << B << endl;

    Matrix<double, Ny, Nx> C = hcat(eye<double, 7>(), zeros<double, 7, 3>());

    cout << "C = " << C << endl;

    Matrix<double, Ny, Nu> D = {};

    cout << "D = " << D << endl;

    Matrix<double, Nu, Nx> K = {{
        {
            0.000000,
            -10.000000,
            0.000000,
            0.000000,
            -1.927990,
            0.000000,
            0.000000,
            -0.993059,
            0.000000,
            -0.000000,
        },
        {
            0.000000,
            0.000000,
            -10.000000,
            -0.000000,
            0.000000,
            -1.982692,
            -0.000000,
            0.000000,
            -0.993012,
            0.000000,
        },
        {
            0.000000,
            -0.000000,
            -0.000000,
            -10.000000,
            0.000000,
            -0.000000,
            -6.423163,
            0.000000,
            -0.000000,
            -0.584187,
        },
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
