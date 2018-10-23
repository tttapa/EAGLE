#include "Model.hpp"
#include "Params.hpp"
#include <DormandPrince.hpp>
#include <LeastSquares.hpp>
#include <Matrix.hpp>
#include <Quaternion.hpp>
#include <iostream>

using namespace std;
using Matrices::T;

class NonLinearFullModel : public ContinuousModel<double, 10, 3> {
  public:
    using VecOmega_t = ColVector<double, 3>;
    using VecN_t     = ColVector<double, 3>;

    NonLinearFullModel(const Params &p) : p(p) {}
    VecX_t operator()(const VecX_t &x, const VecU_t &u) override {
        Quaternion q     = getBlock<0, 4, 0, 1>(x);
        VecOmega_t omega = getBlock<4, 7, 0, 1>(x);
        VecN_t n         = getBlock<7, 10, 0, 1>(x);

        Quaternion q_omega               = {};
        assignBlock<1, 4, 0, 1>(q_omega) = omega;

        Quaternion q_dot = 0.5 * quatmultiply(q, q_omega);
        VecOmega_t omega_dot =
            p.gamma_n * n + p.gamma_u * u -
            solveLeastSquares(p.I, cross(omega, p.I * omega));
        VecN_t n_dot = p.k2 * (p.k1 * u - n);

        VecX_t x_dot;
        assignBlock<0, 4, 0, 1>(x_dot)  = q_dot;
        assignBlock<4, 7, 0, 1>(x_dot)  = omega_dot;
        assignBlock<7, 10, 0, 1>(x_dot) = n_dot;

        return x_dot;
    }
    Params p;
};

struct TestInputFunction : public NonLinearFullModel::InputFunction {
    NonLinearFullModel::VecU_t operator()(double t) override {
        return {{
            {0},
            {0},
            {0.5 * (t > 2)},
        }};
    }
};

void printCSV(const NonLinearFullModel::SimulationResult &result) {
    for (size_t i = 0; i < result.time.size(); ++i) {
        cout << result.time[i] << ',';
        for (size_t j = 0; j < result.solution[0].length; ++j) {
            cout << result.solution[i][j][0]
                 << (j == result.solution[0].length - 1 ? "\r\n" : ",");
        }
    }
}

int main(int argc, char const *argv[]) {
    const Params p = {};
    // cout << "I" << endl << p.I << endl;
    // cout << "k3" << endl << p.k3 << endl;
    // cout << "k4" << endl << p.k4 << endl;
    // cout << "gamma_n" << endl << p.gamma_n << endl;
    // cout << "gamma_u" << endl << p.gamma_u << endl;
    // return 0;
    NonLinearFullModel nonlinfull = p;
    TestInputFunction u           = {};
    NonLinearFullModel::VecX_t x0 = {{
        {1},
        {0},
        {0},
        {0},
        {0.1},
    }};
    NonLinearFullModel::SimulationResult result =
        nonlinfull.simulate(u, 0, 10, 1e-3, 1e-2, x0, 1e4);
    printCSV(result);
    cout << endl;
    return 0;
}
