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

struct Weights {
    ColVector<Nq> Q_diag;
    ColVector<Nr> R_diag;
    auto Q() const { return diag(Q_diag); }
    auto R() const -> decltype(diag(R_diag)) { return diag(R_diag); }
    double cost;
    bool operator<(const Weights &rhs) const { return this->cost < rhs.cost; }

    void mutate() {
        auto dQ = randn(Config::Tuner::varQ);
        Q_diag += dQ;
        clamp(Q_diag, Config::Tuner::Qmin, Config::Tuner::Qmax);
        auto dR = randn(Config::Tuner::varR);
        R_diag += dR;
        clamp(R_diag, Config::Tuner::Rmin, Config::Tuner::Rmax);
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
        if (resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED)
            return std::numeric_limits<double>::infinity();
        x = states.back();
        if (normsq(getBlock<0, Ny_att, 0, 1>(x) - xref) < xrefnormsqthres)
            return i;
    }
    return max_sim_steps + normsq(getBlock<0, Ny_att, 0, 1>(x) - xref);
}

int main(int argc, char const *argv[]) {
    filesystem::path loadPath = Config::Tuner::loadPath;
    filesystem::path outPath  = "";

    cout << "argc = " << argc << endl;
    for (int i = 0; i < argc; ++i) {
        cout << i << ": " << argv[i] << endl;
        if (strcmp(argv[i], "--load") == 0) {
            ++i;
            if (i == argc) {
                cerr << "Error: `--load` expects the load path as an argument"
                     << endl;
                exit(-1);
            } else {
                cout << "Setting load path to: " << argv[i] << endl;
                loadPath = argv[i];
            }
        }
        if (strcmp(argv[i], "--out") == 0) {
            ++i;
            if (i == argc) {
                cerr << "Error: `--out` expects the output path as an argument"
                     << endl;
                exit(-1);
            } else {
                cout << "Setting output path to: " << argv[i] << endl;
                outPath = argv[i];
            }
        }
    }

    Drone drone = loadPath;

    Drone::AttitudeModel model = drone.getAttitudeModel();

    vector<Weights> populationWeights;
    populationWeights.resize(Config::Tuner::populationSize);

    populationWeights[0].Q_diag = Config::Tuner::Q_diag_initial;
    populationWeights[0].R_diag = Config::Tuner::R_diag_initial;

#pragma omp parallel for
    for (size_t i = 1; i < Config::Tuner::populationSize; ++i) {
        populationWeights[i] = populationWeights[0];
        populationWeights[i].mutate();
    }

    for (size_t g = 0; g < Config::Tuner::generations; ++g) {

#pragma omp parallel for
        for (size_t i = 0; i < Config::Tuner::populationSize; ++i) {
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
        static ColVector<10> x0           = {1};

        Drone::FixedClampAttitudeController ctrl =
            drone.getFixedClampAttitudeController(best.Q(), best.R());
        auto result = model.simulate(ctrl, reff, x0, Config::Tuner::odeopt);
        result.resultCode.verbose();

        /* ------ Plot the simulation result -------------------------------- */
        // Plot all results
        const double t_start = result.time[0];
        const double t_end   = result.time.back();

        plt::figure();
        std::string gstr = std::to_string(g);

        plt::subplot(4, 1, 1);
        plotDroneSignal(result.time, result.solution,
                        &DroneAttitudeState::getOrientationEuler,
                        {"z", "y'", "x\""}, {"b-", "g-", "r-"},
                        "Orientation of drone (Generation #" + gstr + ")");
        plt::xlim(t_start, t_end * 1.1);

        plt::subplot(4, 1, 2);
        plotDroneSignal(result.time, result.solution,
                        &DroneAttitudeState::getAngularVelocity,
                        {"x", "y", "z"}, {"r-", "g-", "b-"},
                        "Angular velocity of drone");
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

        plt::tight_layout();

        std::stringstream filename;
        filename << "generation" << std::setw(3) << std::setfill('0') << g
                 << ".png";
        std::filesystem::path outputfile = outPath / filename.str();
        plt::save(outputfile);

#endif

        //

        // mutate
        std::default_random_engine generator;
        std::uniform_int_distribution<size_t> distribution(
            0, Config::Tuner::survivors - 1);

#pragma omp parallel for
        for (size_t i = Config::Tuner::survivors;
             i < Config::Tuner::populationSize; ++i) {
            size_t idx1 = distribution(generator);
            size_t idx2 = distribution(generator);
            populationWeights[i].crossOver(populationWeights[idx1],
                                           populationWeights[idx2]);
            populationWeights[i].mutate();
        }
    }
    return EXIT_SUCCESS;
}