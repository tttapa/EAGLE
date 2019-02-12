#include "InputSignals.hpp"
#include <ArgParser.hpp>
#include <Config.hpp>
#include <Degrees.hpp>
#include <Plot.hpp>
#include <iostream>

using namespace std;

int main(int argc, char const *argv[]) {
    /* ------ Parse command line arguments ---------------------------------- */

    filesystem::path loadPath = Config::loadPath;
    filesystem::path outPath  = "";
    size_t px_x               = Config::px_x;
    size_t px_y               = Config::px_y;
    bool plotResult           = true;

    ArgParser parser;
    parser.add("--out", "-o", [&](const char *argv[]) {  // TODO
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
    parser.add("--width", "-w", [&](const char *argv[]) {  // TODO
        px_x = strtoul(argv[1], nullptr, 10);
        cout << "Setting the image width to: " << px_x << endl;
    });
    parser.add("--height", "-h", [&](const char *argv[]) {  // TODO
        px_y = strtoul(argv[1], nullptr, 10);
        cout << "Setting the image height to: " << px_y << endl;
    });
    cout << ANSIColors::blue;
    parser.parse(argc, argv);
    cout << ANSIColors::reset << endl;

    /* ------ Load drone data and get models, controllers and observers ----- */

    Drone drone = {loadPath};

    Drone::Controller controller = drone.getController(
        Config::Attitude::Q, Config::Attitude::R, Config::Altitude::K_pi,
        Config::Altitude::maxIntegralInfluence);

    // Drone::Observer observer = drone.getObserver( // TODO
    //     Config::Attitude::varDynamics, Config::Attitude::varSensors,
    //     Config::Altitude::varDynamics, Config::Altitude::varSensors);

    DroneState x0             = drone.getStableState();
    TestReferenceFunction ref = {};

    /* ------ Simulate the drone with the controller ------------------------ */

    PerfTimer pt;
    auto result = drone.simulate(controller, ref, x0, Config::odeopt);
    result.resultCode.verbose();
    cout << "Simulation took " << (pt.getDuration()) << " µs" << endl;

    /* ------ Plot the simulation result ------------------------------------ */

    // TODO
    // plt::figure_size(px_x, px_y);
    // plotDrone(result);
    // plt::axhline(drone.uh, "--", "r");
    // plt::save(outPath / "Simulation.svg");
    // plt::show();

    if (plotResult)
        plot(result);

    cout << "Done." << endl;

    return EXIT_SUCCESS;
}
