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
    constexpr static size_t Nq = 9;
    constexpr static size_t Nr = 3;
    ColVector<Nq> Q_diag;
    ColVector<Nr> R_diag;
    auto Q() const -> decltype(diag(Q_diag)) { return diag(Q_diag); }
    auto R() const -> decltype(diag(R_diag)) { return diag(R_diag); }
    double cost;
    bool operator<(const Weights &rhs) const { return this->cost < rhs.cost; }

    void mutate() {
        auto dQ = randn(varQ);
        Q_diag += dQ;
        clamp(Q_diag, Qmin, Qmax);
        auto dR = randn(varR);
        R_diag += dR;
        clamp(R_diag, Rmin, Rmax);
    }

    void crossOver(const Weights &parent1, const Weights &parent2) {
        static std::default_random_engine generator;
        static std::uniform_int_distribution<size_t> q_distr(0, Nq);
        static std::uniform_int_distribution<size_t> r_distr(0, Nr);
        size_t q_idx = q_distr(generator);
        size_t r_idx = r_distr(generator);

        for (size_t i = 0; i < q_idx; ++i)
            this->Q_diag[i] = parent1.Q_diag[i];
        for (size_t i = q_idx; i < Nq; ++i)
            this->Q_diag[i] = parent2.Q_diag[i];
        for (size_t i = 0; i < r_idx; ++i)
            this->R_diag[i] = parent1.R_diag[i];
        for (size_t i = r_idx; i < Nr; ++i)
            this->R_diag[i] = parent2.R_diag[i];
    }
};

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

const double SimulationClampedDiscreteLQRController::Ts = Config::Ts;

const ColVector<3> SimulationClampedDiscreteLQRController::clampMin =
    -0.3 * ones<3, 1>();
const ColVector<3> SimulationClampedDiscreteLQRController::clampMax =
    0.3 * ones<3, 1>();

class FullClampedController : public DiscreteController<10, 3, 7> {
  public:
    FullClampedController(const SimulationClampedDiscreteLQRController &ctrl)
        : DiscreteController<10, 3, 7>{ctrl.Ts}, ctrl{ctrl} {}

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        auto x_r = getBlock<1, 10, 0, 1>(x);
        auto r_r = getBlock<0, 4, 0, 1>(r);
        return ctrl(x_r, r_r);
    }

    SimulationClampedDiscreteLQRController ctrl;
};

double getCost(SimulationClampedDiscreteLQRController &ctrl) {
    static auto model             = drone.getNonLinearFullModel();
    static Quaternion xref        = eul2quat({M_PI / 16, M_PI / 16, M_PI / 16});
    static double xrefnormsqthres = 0.001 * normsq(xref);
    static size_t max_sim_steps =
        numberOfSamplesInTimeRange(odeopt.t_start, Ts, odeopt.t_end);
    static ColVector<10> x0  = {1};
    auto x                   = x0;
    ODEResultCode resultCode = {};
    AdaptiveODEOptions opt   = Config::odeopt;
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
            return i;
    }
    return max_sim_steps + normsq(getBlock<0, 4, 0, 1>(x) - xref);
}

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;

    auto sysd_r = drone.getLinearReducedDiscreteSystem(
        Ts, DiscretizationMethod::Bilinear);
    const auto Ad_r = sysd_r.A;
    const auto Bd_r = sysd_r.B;

    constexpr size_t populationSize      = 16 * 64;
    constexpr size_t numberOfGenerations = 50;
    constexpr size_t survivors           = populationSize / 64;

    vector<Weights> populationWeights;
    populationWeights.resize(populationSize);

    populationWeights[0].Q_diag = Q_diag_initial;
    populationWeights[0].R_diag = R_diag_initial;

#pragma omp parallel for
    for (size_t i = 1; i < populationSize; ++i) {
        populationWeights[i] = populationWeights[0];
        populationWeights[i].mutate();
    }

    for (size_t g = 0; g < numberOfGenerations; ++g) {

#pragma omp parallel for
        for (size_t i = 0; i < populationSize; ++i) {
            auto &w = populationWeights[i];
            try {
                auto K = drone.getReducedDiscreteControllerMatrixK(w.Q(), w.R(),
                                                                   Ad_r, Bd_r);
                SimulationClampedDiscreteLQRController ctrl = K;
                w.cost                                      = getCost(ctrl);
            } catch (std::runtime_error &e) {
                /* cerr << ANSIColors::redb << "Error: " << e.what()
                     << ANSIColors::reset << endl;
                cerr << "i = " << i << endl;
                cerr << "Q = " << transpose(w.Q_diag) << endl;
                cerr << "R = " << transpose(w.R_diag) << endl; */
                w.cost = std::numeric_limits<double>::infinity();
            }
        }

        __gnu_parallel::sort(populationWeights.begin(),
                             populationWeights.end());

        auto &best = populationWeights[0];

        cout << "Best of generation " << (g) << ':' << endl;
        cout << "\tQ = " << transpose(best.Q_diag) << endl;
        cout << "\tR = " << transpose(best.R_diag) << endl;
        cout << "\tCost = " << best.cost << endl;

        //

#ifdef PLOT_CONTROLLER

        static Quaternion ref = eul2quat({M_PI / 16, M_PI / 16, M_PI / 16});
        static ConstantTimeFunctionT<ColVector<7>> reff =
            vcat(ref, zeros<3, 1>());
        static ColVector<10> x0 = {1};
        static auto model       = drone.getNonLinearFullModel();

        auto K = drone.getReducedDiscreteControllerMatrixK(best.Q(), best.R(),
                                                           Ad_r, Bd_r);
        double t_end               = odeopt.t_end;
        FullClampedController ctrl = {K};
        auto result                = model.simulate(ctrl, reff, x0, odeopt);
        if (result.resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED) {
            std::cerr << ANSIColors::redb
                      << "Error: maximum number of iterations exceeded"
                      << ANSIColors::reset << endl;
            t_end = result.time.back();
        }
        if (result.resultCode & ODEResultCodes::MINIMUM_STEP_SIZE_REACHED)
            std::cerr << ANSIColors::yellow
                      << "Warning: minimum step size reached"
                      << ANSIColors::reset << endl;

        /* ------ Plot the simulation result ------------------------------------ */
        // Time and state solutions
        auto t    = result.time;
        auto data = result.solution;

        // Calculate controller output
        auto u = ctrl.getControlSignal(t, data, reff);

        // Convert the quaternions of the state to euler angles
        vector<EulerAngles> orientation =
            NonLinearFullDroneModel::statesToEuler(data);

        // Plot all results
        plt::figure();
        std::string gstr = std::to_string(g);
        plt::subplot(4, 1, 1);
        plotResults(t, orientation, {0, 3}, {"z", "y", "x"}, {"b-", "g-", "r-"},
                    "Orientation of drone (Generation #" + gstr + ")");
        plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
        plt::subplot(4, 1, 2);
        plotResults(t, data, {4, 7}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                    "Angular velocity of drone");
        plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
        plt::subplot(4, 1, 3);
        plotResults(t, data, {7, 10}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                    "Angular velocity of motors");
        plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
        plt::subplot(4, 1, 4);
        plotResults(t, u, {0, 3}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                    "Control signal");
        plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);

        plt::tight_layout();
        std::stringstream filename;
        filename << "generation" << std::setw(3) << std::setfill('0') << g
                 << ".png";
        plt::save(filename.str());

#endif

        //

        // mutate
        std::default_random_engine generator;
        std::uniform_int_distribution<size_t> distribution(0, survivors - 1);

#pragma omp parallel for
        for (size_t i = survivors; i < populationSize; ++i) {
            size_t idx1 = distribution(generator);
            size_t idx2 = distribution(generator);
            populationWeights[i].crossOver(populationWeights[idx1],
                                           populationWeights[idx2]);
            populationWeights[i].mutate();
        }
    }
    return EXIT_SUCCESS;
}