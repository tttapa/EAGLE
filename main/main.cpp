#include "InputSignals.hpp"
#include "PrintCSV.hpp"

#include "ANSIColors.hpp"
#include "Config.hpp"
#include "Plot.hpp"
#include <ODE/ODEEval.hpp>

#include "CodeGen/CodeGen.hpp"

#include <iostream>

using namespace std;
using namespace Config;

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;

    Drone drone = loadPath;

    Drone::Controller controller =
        drone.getController(Config::Attitude::Q, Config::Attitude::R,
                            Config::Altitude::K_p, Config::Altitude::K_i);

    Drone::Observer observer = drone.getObserver(
        Config::Attitude::varDynamics, Config::Attitude::varSensors,
        Config::Altitude::varDynamics, Config::Altitude::varSensors);

    Drone::VecX_t x0 = drone.getStableState();

    TestReferenceFunction ref = {};

    /* ------ Simulate the drone with the controller ------------------------ */
    auto result = drone.simulate(controller, ref, x0, odeopt);

    double t_end = odeopt.t_end;
    if (result.resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED) {
        std::cerr << ANSIColors::redb
                  << "Error: maximum number of iterations exceeded"
                  << ANSIColors::reset << endl;
        t_end = result.time.back();
    }
    if (result.resultCode & ODEResultCodes::MINIMUM_STEP_SIZE_REACHED)
        std::cerr << ANSIColors::yellow << "Warning: minimum step size reached"
                  << ANSIColors::reset << endl;


#ifdef PLOT_CONTROLLER

    /* ------ Plot the simulation result ------------------------------------ */
    plotDrone(result);
    plt::show();

#endif

    /* ------ Export the simulation result as CSV --------------------------- */

    if (exportCSV) {
        // Sample/interpolate the simulation result using a fixed time step
        auto sampled = sampleODEResult(result, odeopt.t_start, CSV_Ts, t_end);
        vector<Quaternion> sampledOrientation =
            Drone::extractState(sampled, &DroneState::getOrientation);
        vector<ColVector<3>> sampledLocation =
            Drone::extractState(sampled, &DroneState::getPosition);
        // Export to the given output file
        printCSV(rotationCSVFile, 0.0, CSV_Ts, sampledOrientation);
        printCSV(locationCSVFile, 0.0, CSV_Ts, sampledLocation);
    }

    return EXIT_SUCCESS;
}
