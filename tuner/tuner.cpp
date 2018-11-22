#include <Matrix/DLQR.hpp>
#include <Matrix/Randn.hpp>
#include <Util/AlmostEqual.hpp>
#include <iostream>
#include <parallel/algorithm>

#include <Plot.hpp>
#include <Util/ANSIColors.hpp>

#include <Config.hpp>

using namespace std;

constexpr static size_t Nq = ::Attitude::Nx - 1;
constexpr static size_t Nr = ::Attitude::Nu;

/* ------ Config ------------------------------------------------------------ */
constexpr RowVector<3> Qn_initial     = {1, 1, 1};
constexpr RowVector<3> Qomega_initial = {1, 1, 1};
constexpr RowVector<3> Qq_initial     = {1, 1, 1};

/** Weighting matrix for states in LQR design. */
constexpr ColVector<9> Q_diag_initial =
    transpose(hcat(Qq_initial, Qomega_initial, Qn_initial));
/** Weighting matrix for inputs in LQR design. */
constexpr ColVector<3> R_diag_initial = ones<3, 1>();

/* ------ Tuner mutation variance ------------------------------------------- */
constexpr double varQ[9] = {1, 1, 1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
constexpr double varR[3] = {1e-2, 1e-2, 1e-2};

constexpr ColVector<9> Qmin = 1e-6 * ones<9, 1>();
constexpr ColVector<3> Rmin = 1e-6 * ones<3, 1>();

constexpr ColVector<9> Qmax = 1e6 * ones<9, 1>();
constexpr ColVector<3> Rmax = 1e6 * ones<3, 1>();

struct Weights {
    ColVector<Nq> Q_diag;
    ColVector<Nr> R_diag;
    auto Q() const { return diag(Q_diag); }
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

double getCost(Drone::FixedClampAttitudeController &ctrl,
               ContinuousModel<Nx_att, Nu_att, Ny_att> &model) {
    static ColVector<Ny_att> xref =
        vcat(eul2quat({M_PI / 16, M_PI / 16, M_PI / 16}), zeros<3, 1>());
    static double xrefnormsqthres = 0.001 * normsq(xref);
    static size_t max_sim_steps   = numberOfSamplesInTimeRange(
        Config::odeopt.t_start, ctrl.Ts, Config::odeopt.t_end);
    static ColVector<10> x0  = {1};
    auto x                   = x0;
    ODEResultCode resultCode = {};
    AdaptiveODEOptions opt   = Config::odeopt;
    for (size_t i = 0; i < max_sim_steps; ++i) {
        double t    = opt.t_start + ctrl.Ts * i;
        opt.t_start = t;
        opt.t_end   = t + ctrl.Ts;
        auto curr_u = ctrl(x, xref);
        vector<double> time;
        vector<ColVector<10>> states;
        resultCode |=
            model.simulate(std::back_inserter(time), std::back_inserter(states),
                           curr_u, x, opt);
        x = states.back();
        if (normsq(getBlock<0, Ny_att, 0, 1>(x) - xref) < xrefnormsqthres)
            return i;
    }
    return max_sim_steps + normsq(getBlock<0, Ny_att, 0, 1>(x) - xref);
}

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;

    Drone drone = Config::loadPath;

    Drone::AttitudeModel model = drone.getAttitudeModel();

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
                Drone::FixedClampAttitudeController ctrl =
                    drone.getFixedClampAttitudeController(w.Q(), w.R());
                w.cost = getCost(ctrl, model);
            } catch (std::runtime_error &e) {
                // LAPACK sometimes fails for certain Q and R
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

        /* ------ Simulate the drone with the best controller so far -------- */

        static ColVector<Ny_att> ref =
            vcat(eul2quat({M_PI / 16, M_PI / 16, M_PI / 16}), zeros<3, 1>());
        static ConstantTimeFunctionT reff = ref;
        static ColVector<10> x0 = {1};

        Drone::FixedClampAttitudeController ctrl =
            drone.getFixedClampAttitudeController(best.Q(), best.R());
        auto result = model.simulate(ctrl, reff, x0, Config::odeopt);
        result.resultCode.verbose();

        /* ------ Plot the simulation result -------------------------------- */
        // Plot all results
        const double t_start = result.time[0];
        const double t_end   = result.time.back();

        plt::figure();
        std::string gstr = std::to_string(g);
        plt::subplot(4, 1, 1);
        plt::tight_layout();
        plotDroneSignal(result.sampledTime, result.reference,
                        &DroneOutput::getOrientationEuler, {"z", "y'", "x\""},
                        {"b" DISCRETE_FMT, "g" DISCRETE_FMT, "r" DISCRETE_FMT},
                        "Reference orientation");
        plt::xlim(t_start, t_end * 1.1);

        plt::subplot(4, 1, 2);
        plotDroneSignal(result.time, result.solution,
                        &DroneAttitudeState::getOrientationEuler,
                        {"z", "y'", "x\""}, {"b-", "g-", "r-"},
                        "Orientation of drone");
        plt::xlim(t_start, t_end * 1.1);

        plt::subplot(4, 1, 3);
        plotDroneSignal(result.time, result.solution,
                        &DroneAttitudeState::getMotorSpeed, {"x", "y", "z"},
                        {"r-", "g-", "b-"},
                        "Angular velocity of torque motors");
        plt::xlim(t_start, t_end * 1.1);

        plt::subplot(4, 1, 4);
        plotDroneSignal(result.sampledTime, result.control,
                        &DroneControl::getAttitudeControl, {"x", "y", "z"},
                        {"r" DISCRETE_FMT, "g" DISCRETE_FMT, "b" DISCRETE_FMT},
                        "Torque motor control");
        plt::xlim(t_start, t_end * 1.1);

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