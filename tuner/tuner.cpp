#include <Matrix/Randn.hpp>
#include <Util/AlmostEqual.hpp>

#include <Util/PerfTimer.hpp>
#include <cstdlib>  // strtoul

#include <iostream>
#include <parallel/algorithm>

#include <Plot.hpp>
#include <Util/ANSIColors.hpp>

#include <ArgParser/ArgParser.hpp>

#include <Config.hpp>

using namespace std;

constexpr static size_t Nq = ::Attitude::Nx - 1;
constexpr static size_t Nr = ::Attitude::Nu;

struct Weights {
    ColVector<Nq> Q_diag;
    ColVector<Nr> R_diag;
    auto Q() const { return diag(Q_diag); }
    auto R() const { return diag(R_diag); }
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

double getRiseTimeCost(Drone::FixedClampAttitudeController &ctrl,
                       ContinuousModel<Nx_att, Nu_att, Ny_att> &model,
                       ColVector<Ny_att> xref, double xrefnormsqthres,
                       ColVector<10> x0 = {1}) {
    static size_t max_sim_steps = numberOfSamplesInTimeRange(
        Config::Tuner::odeopt.t_start, ctrl.Ts, Config::Tuner::odeopt.t_end);

    AdaptiveODEOptions opt   = Config::Tuner::odeopt;
    auto x                   = x0;
    ODEResultCode resultCode = {};
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

double getCost(Drone::FixedClampAttitudeController &ctrl,
               ContinuousModel<Nx_att, Nu_att, Ny_att> &model) {
    double totalCost = 0;
    {
        static ColVector<Ny_att> ref =
            vcat(eul2quat({M_PI / 16, M_PI / 16, M_PI / 16}), zeros<3, 1>());
        static double refnormsqthres = 0.001 * normsq(ref);
        totalCost += getRiseTimeCost(ctrl, model, ref, refnormsqthres);
    }
    {
        static ColVector<Ny_att> ref =
            vcat(eul2quat({0, M_PI / 16, M_PI / 16}), zeros<3, 1>());
        static double refnormsqthres = 0.001 * normsq(ref);
        totalCost += getRiseTimeCost(ctrl, model, ref, refnormsqthres);
    }
    {
        static ColVector<Ny_att> ref =
            vcat(eul2quat({M_PI / 16, 0, 0}), zeros<3, 1>());
        static double refnormsqthres = 0.001 * normsq(ref);
        totalCost += getRiseTimeCost(ctrl, model, ref, refnormsqthres);
    }
    {
        static ColVector<Ny_att> ref =
            vcat(eul2quat({0, M_PI / 16, 0}), zeros<3, 1>());
        static double refnormsqthres = 0.001 * normsq(ref);
        // totalCost += getRiseTimeCost(ctrl, model, ref, refnormsqthres);
    }
    {
        static ColVector<Ny_att> ref =
            vcat(eul2quat({0, 0, M_PI / 16}), zeros<3, 1>());
        static double refnormsqthres = 0.001 * normsq(ref);
        // totalCost += getRiseTimeCost(ctrl, model, ref, refnormsqthres);
    }
    return totalCost;
}

int main(int argc, char const *argv[]) {
    filesystem::path loadPath = Config::Tuner::loadPath;
    filesystem::path outPath  = "";

    size_t population  = Config::Tuner::population;
    size_t generations = Config::Tuner::generations;
    size_t survivors   = Config::Tuner::survivors;

    size_t px_x = Config::Tuner::px_x;
    size_t px_y = Config::Tuner::px_y;

    ArgParser parser;
    parser.add("--out", "-o", [&outPath](const char *argv[]) {
        outPath = argv[1];
        cout << "Setting output path to: " << argv[1] << endl;
    });
    parser.add("--load", "-l", [&loadPath](const char *argv[]) {
        loadPath = argv[1];
        cout << "Setting load path to: " << argv[1] << endl;
    });
    parser.add("--population", "-p", [&population](const char *argv[]) {
        population = strtoul(argv[1], nullptr, 10);
        cout << "Setting population size to: " << population << endl;
    });
    parser.add("--generations", "-g", [&generations](const char *argv[]) {
        generations = strtoul(argv[1], nullptr, 10);
        cout << "Setting number of generations to: " << generations << endl;
    });
    parser.add("--survivors", "-s", [&survivors](const char *argv[]) {
        survivors = strtoul(argv[1], nullptr, 10);
        cout << "Setting number of survivors to: " << survivors << endl;
    });
    parser.add("-x", [&px_x](const char *argv[]) {
        px_x = strtoul(argv[1], nullptr, 10);
        cout << "Setting the image width to: " << px_x << endl;
    });
    parser.add("-y", [&px_y](const char *argv[]) {
        px_y = strtoul(argv[1], nullptr, 10);
        cout << "Setting the image height to: " << px_y << endl;
    });
    cout << ANSIColors::blueb;
    parser.parse(argc, argv);
    cout << ANSIColors::reset << endl;

    assert(survivors > 0);
    assert(population > survivors);

    cout << "Loading Drone ..." << endl;
    Drone drone = loadPath;
    cout << ANSIColors::greenb << "Successfully loaded Drone!"
         << ANSIColors::reset << endl
         << endl;

    Drone::AttitudeModel model = drone.getAttitudeModel();

    vector<Weights> populationWeights;
    populationWeights.resize(population);

    populationWeights[0].Q_diag = Config::Tuner::Q_diag_initial;
    populationWeights[0].R_diag = Config::Tuner::R_diag_initial;

    // Random engine for mutations
    static std::default_random_engine generator;
    static std::uniform_int_distribution<size_t> distribution(0, survivors - 1);

    cout << ANSIColors::blueb << "Starting Genetic Algorithm ..."
         << ANSIColors::reset << endl;

    for (size_t g = 0; g < generations; ++g) {

        PerfTimer mutateTimer;
        if (g == 0)
#pragma omp parallel for
            for (size_t i = 1; i < population; ++i) {
                populationWeights[i] = populationWeights[0];
                populationWeights[i].mutate();
            }
        else
#pragma omp parallel for
            for (size_t i = survivors; i < population; ++i) {
                size_t idx1 = distribution(generator);
                size_t idx2 = distribution(generator);
                populationWeights[i].crossOver(populationWeights[idx1],
                                               populationWeights[idx2]);
                populationWeights[i].mutate();
            }
        auto mutateTime = mutateTimer.getDuration<chrono::microseconds>();
        cout << endl
             << "Mutated " << population << " controllers in " << mutateTime
             << " µs." << endl;

        PerfTimer simTimer;
#pragma omp parallel for
        for (size_t i = 0; i < population; ++i) {
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
        auto simTime = simTimer.getDuration<chrono::milliseconds>();
        cout << "Simulated " << population << " controllers in " << simTime
             << " ms." << endl;

        PerfTimer sortTimer;
        __gnu_parallel::sort(populationWeights.begin(),
                             populationWeights.end());
        auto sortTime = sortTimer.getDuration<chrono::microseconds>();
        cout << "Sorted " << population << " controllers in " << sortTime
             << " µs." << endl;

        auto &best = populationWeights[0];
        cout << ANSIColors::cyanb << endl;
        cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
        cout << ANSIColors::whiteb;
        cout << "Best of generation " << (g + 1) << ':' << endl;
        cout << ANSIColors::cyanb;
        cout << "=====================================================" << endl;
        cout << ANSIColors::whiteb;
        cout << "\tQ = diag(";
        printMATLAB(cout, transpose(best.Q_diag));
        cout << ");" << endl;
        cout << "\tR = diag(";
        printMATLAB(cout, transpose(best.R_diag));
        cout << ");" << endl;
        cout << "\tCost = " << best.cost << endl;
        cout << ANSIColors::cyanb;
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
        cout << ANSIColors::reset;

        //

#ifdef PLOT_CONTROLLER

        /* ------ Simulate the drone with the best controller so far -------- */

        class DisplayRef
            : public Drone::FixedClampAttitudeController::ReferenceFunction {
          public:
            Drone::FixedClampAttitudeController::VecR_t
            operator()(double t) override {
                Quaternion q = qu;
                if (t >= m * 0 && t < m * 1)
                    q = quatmultiply(q, qx);
                if (t >= m * 2 && t < m * 3)
                    q = quatmultiply(q, qy);
                if (t >= m * 4 && t < m * 5)
                    q = quatmultiply(q, qz);

                if (t >= m * 6 && t < m * 7)
                    q = quatmultiply(q, qx);
                if (t >= m * 6 && t < m * 7)
                    q = quatmultiply(q, qy);

                DroneAttitudeOutput rr;
                rr.setOrientation(q);
                return rr;
            }
            const Quaternion qz = eul2quat({M_PI / 8, 0, 0});
            const Quaternion qy = eul2quat({0, M_PI / 8, 0});
            const Quaternion qx = eul2quat({0, 0, M_PI / 8});
            const Quaternion qu = eul2quat({0, 0, 0});
            const double m      = 0.5;
        } reff;

        static ColVector<10> x0 = {1};

        Drone::FixedClampAttitudeController ctrl =
            drone.getFixedClampAttitudeController(best.Q(), best.R());
        auto result = model.simulate(ctrl, reff, x0, Config::Tuner::odeoptdisp);
        result.resultCode.verbose();

        /* ------ Plot the simulation result -------------------------------- */
        // Plot all results
        const double t_start = result.time[0];
        const double t_end   = result.time.back();

        std::string gstr = std::to_string(g + 1);
        plt::figure_size(px_x, px_y);

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
        filename << "generation" << std::setw(4) << std::setfill('0') << (g + 1)
                 << ".png";
        std::filesystem::path outputfile = outPath / filename.str();
        plt::save(outputfile);
        plt::clf();

#endif
    }
    cout << ANSIColors::greenb << endl << "Done. ✔" << endl;
    return EXIT_SUCCESS;
}