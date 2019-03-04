#include <pybind11/embed.h>

#include "InputSignals.hpp"
#include <ArgParser.hpp>
#include <Config.hpp>
#include <Degrees.hpp>
#include <Plot.hpp>
#include <PlotStepResponse.hpp>
#include <iostream>

using namespace std;

int main(int argc, char const *argv[]) {

    /* ------ Parse command line arguments ---------------------------------- */

    filesystem::path loadPath = Config::loadPath;
    filesystem::path outPath  = "";
    size_t w                  = Config::px_x;
    size_t h                  = Config::px_y;
    bool plotResult           = true;

    ArgParser parser;
    parser.add("--out", "-o", [&](const char *argv[]) {
        outPath = argv[1];
        cout << "Setting output path to: " << argv[1] << endl;
    });
    parser.add<0>("--no-plot", "-n", [&](const char *[]) {
        plotResult = false;
        cout << "Not plotting the simulation result" << endl;
    });
    parser.add("--load", "-l", [&](const char *argv[]) {
        loadPath = argv[1];
        cout << "Setting load path to: " << argv[1] << endl;
    });
    parser.add("--width", "-w", [&](const char *argv[]) {
        w = strtoul(argv[1], nullptr, 10);
        cout << "Setting the image width to: " << w << endl;
    });
    parser.add("--height", "-h", [&](const char *argv[]) {
        h = strtoul(argv[1], nullptr, 10);
        cout << "Setting the image height to: " << h << endl;
    });
    cout << ANSIColors::blue;
    parser.parse(argc, argv);
    cout << ANSIColors::reset << endl;

    /* -------------------- Start the Python interpreter -------------------- */

    pybind11::scoped_interpreter guard{};

    /* ------ Load drone data and get models, controllers and observers ----- */

    Drone drone = {loadPath};

    Drone::Controller controller = drone.getController(
        Config::Attitude::Q, Config::Attitude::R, Config::Altitude::K_pi,
        Config::Altitude::maxIntegralInfluence);

    Drone::Observer observer = drone.getObserver(
        Config::Attitude::varDynamics, Config::Attitude::varSensors,
        Config::Altitude::varDynamics, Config::Altitude::varSensors);

    DroneState x0             = drone.getStableState();
    TestReferenceFunction ref = {};

    GaussianNoiseGenerator randFnW = hcat(Config::Attitude::varDynamics,
                                          Config::Altitude::varDynamics / 100);

    GaussianNoiseGenerator randFnV =
        hcat(Config::Attitude::varSensors, Config::Altitude::varSensors,
             zeros<1, Ny_nav>());

    // NoNoiseGenerator<Nu> randFnW;
    // NoNoiseGenerator<Ny> randFnV;

    /* ------ Simulate the drone with the controller ------------------------ */

    PerfTimer pt;
    Drone::ObserverControllerSimulationResult result;
    try {
        result = drone.simulate(controller, observer, randFnW, randFnV, ref, x0,
                                Config::odeopt);
    } catch (std::exception &e) {
        cerr << ANSIColors::red << e.what() << ANSIColors::reset << endl;
    }
    result.resultCode.verbose();
    cout << "Simulation took " << pt.getDuration() << " µs" << endl;

    /* ------ Plot the simulation result ------------------------------------ */

    if (plotResult || !outPath.empty()) {
        // Plot and/or save the simulation result
        auto fig = plot(result, w, h, 1, "Simulation");
        if (!outPath.empty())
            save(fig, outPath / "Simulation.svg");
        if (plotResult)
            show(fig);

        // Plot and/or save the step response
        EulerAngles e_ref = {0_deg, 30_deg, 30_deg};
        Quaternion q_ref  = eul2quat(e_ref);
        ostringstream title;
        title << "Step Response $" << asEulerAngles(e_ref, degreesTeX) << "$";
        auto ax = axes(w, h);
        plotStepResponseAttitude(drone, Config::Attitude::Q,
                                 Config::Attitude::R, 0.025, q_ref,
                                 Config::odeopt, ax, title.str());
        fig = ax.attr("figure");
        if (!outPath.empty())
            save(fig, outPath / "Step-Response.svg");
        if (plotResult)
            show(fig);
    }

    /* ------ Done ---------------------------------------------------------- */

    cout << "Done." << endl;

    return EXIT_SUCCESS;
}
