#include <Matrix/DLQR.hpp>
#include <Matrix/Randn.hpp>
#include <Util/AlmostEqual.hpp>
#include <iostream>
#include <parallel/algorithm>

#include <ANSIColors.hpp>
#include <Plot.hpp>

#include "Config.hpp"

using namespace std;
using namespace Config;

struct Weights {
    std::remove_const<decltype(Q_diag_initial)>::type Q_diag;
    std::remove_const<decltype(R_diag_initial)>::type R_diag;
    auto Q() const -> decltype(diag(Q_diag)) { return diag(Q_diag); }
    auto R() const -> decltype(diag(R_diag)) { return diag(R_diag); }
    double score;
    bool operator<(const Weights &rhs) const { return this->score < rhs.score; }
};

template <class T>
static T clamp(T v, T lo, T hi) {
    return (v < lo) ? lo : ((hi < v) ? hi : v);
}
template <size_t R, size_t C>
void clamp(Matrix<R, C> &m, const Matrix<R, C> &min, const Matrix<R, C> &max) {
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            m[r][c] = clamp(m[r][c], min[r][c], max[r][c]);
}

class SimulationClampedDiscreteLQRController
    : public DiscreteController<9, 3, 4> {
  public:
    SimulationClampedDiscreteLQRController()
        : DiscreteController<9, 3, 4>{Ts} {}

    SimulationClampedDiscreteLQRController(const Matrix<3, 9> &K)
        : DiscreteController<9, 3, 4>{Ts}, K{K} {}

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        const double *x_hat_r = &x[0][0];
        const double *ref     = &r[0][0];
        ColVector<3> u_v;
        double *u = &u_v[0][0];
        /* Calculate the real part of the quaternion from the reduced state. */
        double xq0 = sqrt(1.0 - x_hat_r[0] * x_hat_r[0] -
                          x_hat_r[1] * x_hat_r[1] - x_hat_r[2] * x_hat_r[2]);
        /* Generated calculations for control signal. */
        u[0] = K[1 - 1][4 - 1] * x_hat_r[4 - 1] +
               K[1 - 1][5 - 1] * x_hat_r[5 - 1] +
               K[1 - 1][6 - 1] * x_hat_r[6 - 1] +
               K[1 - 1][7 - 1] * x_hat_r[7 - 1] +
               K[1 - 1][8 - 1] * x_hat_r[8 - 1] +
               K[1 - 1][9 - 1] * x_hat_r[9 - 1] +
               K[1 - 1][1 - 1] *
                   (ref[1 - 1] * x_hat_r[1 - 1] + ref[3 - 1] * x_hat_r[3 - 1] -
                    ref[4 - 1] * x_hat_r[2 - 1] - ref[2 - 1] * xq0) +
               K[1 - 1][2 - 1] *
                   (ref[1 - 1] * x_hat_r[2 - 1] - ref[2 - 1] * x_hat_r[3 - 1] +
                    ref[4 - 1] * x_hat_r[1 - 1] - ref[3 - 1] * xq0) +
               K[1 - 1][3 - 1] *
                   (ref[1 - 1] * x_hat_r[3 - 1] + ref[2 - 1] * x_hat_r[2 - 1] -
                    ref[3 - 1] * x_hat_r[1 - 1] - ref[4 - 1] * xq0);
        u[1] = K[2 - 1][4 - 1] * x_hat_r[4 - 1] +
               K[2 - 1][5 - 1] * x_hat_r[5 - 1] +
               K[2 - 1][6 - 1] * x_hat_r[6 - 1] +
               K[2 - 1][7 - 1] * x_hat_r[7 - 1] +
               K[2 - 1][8 - 1] * x_hat_r[8 - 1] +
               K[2 - 1][9 - 1] * x_hat_r[9 - 1] +
               K[2 - 1][1 - 1] *
                   (ref[1 - 1] * x_hat_r[1 - 1] + ref[3 - 1] * x_hat_r[3 - 1] -
                    ref[4 - 1] * x_hat_r[2 - 1] - ref[2 - 1] * xq0) +
               K[2 - 1][2 - 1] *
                   (ref[1 - 1] * x_hat_r[2 - 1] - ref[2 - 1] * x_hat_r[3 - 1] +
                    ref[4 - 1] * x_hat_r[1 - 1] - ref[3 - 1] * xq0) +
               K[2 - 1][3 - 1] *
                   (ref[1 - 1] * x_hat_r[3 - 1] + ref[2 - 1] * x_hat_r[2 - 1] -
                    ref[3 - 1] * x_hat_r[1 - 1] - ref[4 - 1] * xq0);
        u[2] = K[3 - 1][4 - 1] * x_hat_r[4 - 1] +
               K[3 - 1][5 - 1] * x_hat_r[5 - 1] +
               K[3 - 1][6 - 1] * x_hat_r[6 - 1] +
               K[3 - 1][7 - 1] * x_hat_r[7 - 1] +
               K[3 - 1][8 - 1] * x_hat_r[8 - 1] +
               K[3 - 1][9 - 1] * x_hat_r[9 - 1] +
               K[3 - 1][1 - 1] *
                   (ref[1 - 1] * x_hat_r[1 - 1] + ref[3 - 1] * x_hat_r[3 - 1] -
                    ref[4 - 1] * x_hat_r[2 - 1] - ref[2 - 1] * xq0) +
               K[3 - 1][2 - 1] *
                   (ref[1 - 1] * x_hat_r[2 - 1] - ref[2 - 1] * x_hat_r[3 - 1] +
                    ref[4 - 1] * x_hat_r[1 - 1] - ref[3 - 1] * xq0) +
               K[3 - 1][3 - 1] *
                   (ref[1 - 1] * x_hat_r[3 - 1] + ref[2 - 1] * x_hat_r[2 - 1] -
                    ref[3 - 1] * x_hat_r[1 - 1] - ref[4 - 1] * xq0);
        clamp(u_v, clampMin, clampMax);
        return u_v;
    }
    Matrix<3, 9> K;

    const static double Ts;

    const static ColVector<3> clampMin;
    const static ColVector<3> clampMax;
};

const double SimulationClampedDiscreteLQRController::Ts = 1.0 / 238;

const ColVector<3> SimulationClampedDiscreteLQRController::clampMin =
    -0.3 * ones<3, 1>();
const ColVector<3> SimulationClampedDiscreteLQRController::clampMax =
    0.3 * ones<3, 1>();

double getScore(SimulationClampedDiscreteLQRController &ctrl) {
    static auto model             = drone.getNonLinearFullModel();
    static Quaternion xref        = eul2quat({0, 0, M_PI / 16});
    static double xrefnormsqthres = 0.01 * normsq(xref);
    static size_t max_sim_steps   = 200;
    static ColVector<10> x0       = {1};
    auto x                        = x0;
    ODEResultCode resultCode      = {};
    AdaptiveODEOptions opt        = Config::odeopt;
    for (size_t i = 0; i < max_sim_steps; ++i) {
        double t    = opt.t_start + Ts * i;
        opt.t_start = t;
        opt.t_end   = t + Ts;
        auto curr_u = ctrl(getBlock<1, 10, 0, 1>(x), xref);
        vector<double> time;
        vector<ColVector<10>> states;
        resultCode |=
            model.simulate(std::back_inserter(time), std::back_inserter(states),
                           curr_u, x, opt);
        x = states.back();
        if (normsq(getBlock<0, 4, 0, 1>(x) - xref) < xrefnormsqthres)
            return 1.0 / i;
    }
    return 0.0;
}

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;

    auto sysd_r = drone.getLinearReducedDiscreteSystem(
        Ts, DiscretizationMethod::Bilinear);
    const auto Ad_r = sysd_r.A;
    const auto Bd_r = sysd_r.B;

    constexpr size_t populationSize = 512*64;

    vector<Weights> populationWeights;
    populationWeights.resize(populationSize);

    populationWeights[0].Q_diag = Q_diag_initial;
    populationWeights[0].R_diag = R_diag_initial;

#pragma omp parallel for
    for (size_t i = 1; i < populationSize; ++i) {
        auto dQ                     = randn(varQ);
        populationWeights[i].Q_diag = Q_diag_initial + dQ;
        clamp(populationWeights[i].Q_diag, Qmin, Qmax);
        auto dR                     = randn(varR);
        populationWeights[i].R_diag = R_diag_initial + dR;
        clamp(populationWeights[i].R_diag, Rmin, Rmax);
    }

#pragma omp parallel for
    for (size_t i = 0; i < populationSize; ++i) {
        auto &w = populationWeights[i];
        auto K =
            drone.getReducedDiscreteControllerMatrixK(w.Q(), w.R(), Ad_r, Bd_r);
        SimulationClampedDiscreteLQRController ctrl = K;
        w.score                                     = getScore(ctrl);
    }

    __gnu_parallel::sort(populationWeights.begin(), populationWeights.end());
    cout << "Best:" << endl;
    cout << "\tQ = " << populationWeights.back().Q_diag << endl;
    cout << "\tR = " << populationWeights.back().R_diag << endl;

    return EXIT_SUCCESS;
}