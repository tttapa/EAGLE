#include "Cost.hpp"
#include "DisplayReference.hpp"
#include "PrintBest.hpp"
#include "Weights.hpp"
#include <ArgParser/ArgParser.hpp>
#include <Config.hpp>
#include <Plot.hpp>
#include <PlotStepReponse.hpp>
#include <Util/ANSIColors.hpp>
#include <Util/AlmostEqual.hpp>
#include <Util/Degrees.hpp>
#include <Util/PerfTimer.hpp>
#include <cstdlib>  // strtoul
#include <iostream>
#include <parallel/algorithm>

using namespace std;

/*
┏━━━━━━━━━━━━━━━━━┓
┃ Generation #    ┃
┡━━━━━━━━━━━━━━━━━┩
│                 │
└─────────────────┘
*/

int main(int argc, char const *argv[]) {
    filesystem::path loadPath = Config::Tuner::loadPath;
    filesystem::path outPath  = "";
    size_t population         = Config::Tuner::population;
    size_t generations        = Config::Tuner::generations;
    size_t survivors          = Config::Tuner::survivors;
    size_t px_x               = Config::px_x;
    size_t px_y               = Config::px_y;
    bool show                 = true;

    double steperrorfactor      = Config::Tuner::steperrorfactor;
    CostWeights stepcostweights = Config::Tuner::stepcostweights;
    AdaptiveODEOptions odeopt   = Config::Tuner::odeopt;

    ArgParser parser;
    parser.add("--out", "-o", [&](const char *argv[]) {
        outPath = argv[1];
        cout << "Setting output path to: " << argv[1] << endl;
    });
    parser.add("--load", "-l", [&](const char *argv[]) {
        loadPath = argv[1];
        cout << "Setting load path to: " << argv[1] << endl;
    });
    parser.add("--population", "-p", [&](const char *argv[]) {
        population = strtoul(argv[1], nullptr, 10);
        cout << "Setting population size to: " << population << endl;
    });
    parser.add("--generations", "-g", [&](const char *argv[]) {
        generations = strtoul(argv[1], nullptr, 10);
        cout << "Setting number of generations to: " << generations << endl;
    });
    parser.add("--survivors", "-s", [&](const char *argv[]) {
        survivors = strtoul(argv[1], nullptr, 10);
        cout << "Setting number of survivors to: " << survivors << endl;
    });
    parser.add("--width", "-w", [&](const char *argv[]) {
        px_x = strtoul(argv[1], nullptr, 10);
        cout << "Setting the image width to: " << px_x << endl;
    });
    parser.add("--height", "-h", [&](const char *argv[]) {
        px_y = strtoul(argv[1], nullptr, 10);
        cout << "Setting the image height to: " << px_y << endl;
    });
    parser.add<0>("--no-show", [&](const char * /* argv */ []) {
        show = false;
        cout << "Not showing the resulting plots" << endl;
    });
    cout << ANSIColors::blue;
    parser.parse(argc, argv);
    cout << ANSIColors::reset << endl;

    assert(survivors > 0);
    assert(population >= survivors);  // TODO!
    if (population == survivors)
        cout << ANSIColors::yellow
             << "Warning: the entire population are survivors, so no mutations "
                "will take place."
             << ANSIColors::reset << endl
             << endl;

    cout << "Loading Drone ..." << endl;
    Drone drone = loadPath;
    cout << ANSIColors::greenb << "Successfully loaded Drone!"
         << ANSIColors::reset << endl
         << endl;

    Drone::AttitudeModel model = drone.getAttitudeModel();

    DroneState x0            = drone.getStableState();
    DroneAttitudeState attx0 = x0.getAttitude();

    vector<Weights> populationWeights;
    populationWeights.resize(population);

    populationWeights[0].Q_diag = Config::Tuner::Q_diag_initial;
    populationWeights[0].R_diag = Config::Tuner::R_diag_initial;
    populationWeights[0].renormalize();

    // Random engine for mutations
    static std::default_random_engine generator;
    static std::uniform_int_distribution<size_t> distribution(0, survivors - 1);

    cout << ANSIColors::blueb << "Starting Genetic Algorithm ..."
         << ANSIColors::reset << endl;

    for (size_t g = 0; g < generations; ++g) {

        PerfTimer mutateTimer;
        if (g == 0)
#ifndef DEBUG
#pragma omp parallel for
#endif
            for (size_t i = 1; i < population; ++i) {
                populationWeights[i] = populationWeights[0];
                populationWeights[i].mutate();
                populationWeights[i].renormalize();
            }
        else
#ifndef DEBUG
#pragma omp parallel for
#endif
            for (size_t i = survivors; i < population; ++i) {
                size_t idx1 = distribution(generator);
                size_t idx2 = distribution(generator);
                populationWeights[i].crossOver(populationWeights[idx1],
                                               populationWeights[idx2]);
                populationWeights[i].mutate();
                populationWeights[i].renormalize();
            }
        auto mutateTime = mutateTimer.getDuration<chrono::microseconds>();
        cout << endl
             << "Mutated " << population << " controllers in " << mutateTime
             << " µs." << endl;

        PerfTimer simTimer;
#ifndef DEBUG
#pragma omp parallel for
#endif
        for (size_t i = 0; i < population; ++i) {
            auto &w = populationWeights[i];
            try {
                Drone::FixedClampAttitudeController ctrl =
                    drone.getFixedClampAttitudeController(w.Q(), w.R());
                w.cost = getCost(ctrl, model, steperrorfactor, attx0, odeopt,
                                 stepcostweights);
            } catch (std::runtime_error &e) {
                // LAPACK sometimes fails for certain Q and R
                w.cost = std::numeric_limits<double>::infinity();
#ifdef DEBUG
                cerr << ANSIColors::redb << e.what() << ANSIColors::reset
                     << endl;
#endif
            }
        }
        auto simTime = simTimer.getDuration<chrono::milliseconds>();
        cout << "Simulated " << population << " controllers in " << simTime
             << " ms." << endl;

        PerfTimer sortTimer;
#ifndef DEBUG
        __gnu_parallel::sort(populationWeights.begin(),
                             populationWeights.end());
#else
        sort(populationWeights.begin(), populationWeights.end());
#endif
        auto sortTime = sortTimer.getDuration<chrono::microseconds>();
        cout << "Sorted " << population << " controllers in " << sortTime
             << " µs." << endl;

        auto &best = populationWeights[0];
        printBest(cout, g, best);
        appendBestToFile(outPath / "tuner.output", g, best);

        //

        /* ------ Simulate and plot the drone with the best controller so far */
        DisplayReference reff;

        Drone::FixedClampAttitudeController ctrl =
            drone.getFixedClampAttitudeController(best.Q(), best.R());
        auto result =
            model.simulate(ctrl, reff, attx0, Config::Tuner::odeoptdisp);
        result.resultCode.verbose();

        // plot and save
        std::stringstream filename;
        filename << "generation" << std::setw(4) << std::setfill('0') << (g + 1)
                 << ".png";
        std::filesystem::path outputfile = outPath / filename.str();
        std::string gstr                 = std::to_string(g + 1);

        plt::figure_size(px_x, px_y);
        plotAttitudeTunerResult(result);
        plt::suptitle("Generation #" + gstr);

        plt::save(outputfile);
        if (g + 1 < generations || !show)
            plt::close();
    }
    cout << ANSIColors::greenb << endl
         << "Done. ✔" << ANSIColors::reset << endl;

    if (show && !Config::Tuner::plotAllAtOnce &&
        Config::Tuner::plotSimulationResult)
        plt::show();

    if (Config::Tuner::plotStepResponse) {
        auto &best = populationWeights[0];
        size_t i   = 0;
        for (const Quaternion &ref : CostReferences::references) {
            plt::figure_size(px_x, px_y);
            plotStepResponseAttitude(drone, best.Q(), best.R(), steperrorfactor,
                                     ref, Config::Tuner::odeopt, "");
            std::stringstream filename;
            filename << "stepresponse" << std::setw(4) << std::setfill('0')
                     << (++i) << ".png";
            std::filesystem::path outputfile = outPath / filename.str();
            plt::tight_layout();
            plt::save(outputfile);
            if (!Config::Tuner::plotAllAtOnce && show)
                plt::show();
            else if (!show)
                plt::close();
        }
    }

    if (show && Config::Tuner::plotAllAtOnce &&
        (Config::Tuner::plotStepResponse ||
         Config::Tuner::plotSimulationResult))
        plt::show();

    return EXIT_SUCCESS;
}