#include "InputSignals.hpp"
#include "PrintCSV.hpp"
#include <Matrix/DLQE.hpp>
#include <Matrix/DLQR.hpp>
#include <Matrix/LQR.hpp>
#include <Matrix/Randn.hpp>
#include <Model/System.hpp>
#include <ODE/ODEEval.hpp>
#include <iostream>

#include "ANSIColors.hpp"
#include "Config.hpp"
#include "Plot.hpp"

using namespace std;
using namespace Config;

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;

    /* ------ Drone full non-linear and linear models ----------------------- */
    auto model = drone.getNonLinearFullModel();
    // auto model = drone.getLinearFullContinuousModel();

    /* ------ Design controller --------------------------------------------- */
    // auto controller = drone.getContinuousController(Q, R);
    auto controller = clampController
                          ? drone.getClampedDiscreteController(
                                Q, R, Ts, DiscretizationMethod::Bilinear)
                          : drone.getDiscreteController(
                                Q, R, Ts, DiscretizationMethod::Bilinear);

    /* ------ Reference function -------------------------------------------- */
    TestReferenceFunction ref = {};

    /* ------ Simulate the drone with the controller ------------------------ */
    auto result = model.simulate(controller, ref, x0, odeopt);

    double t_end = odeopt.t_end;
    if (result.resultCode & ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED) {
        std::cerr << ANSIColors::red
                  << "Error: maximum number of iterations exceeded"
                  << ANSIColors::reset << endl;
        t_end = result.time.back();
    }
    if (result.resultCode & ODEResultCodes::MINIMUM_STEP_SIZE_REACHED)
        std::cerr << ANSIColors::yellow << "Warning: minimum step size reached"
                  << ANSIColors::reset << endl;

    /* ------ Export the simulation result as CSV --------------------------- */

    if (exportCSV) {
        // Sample/interpolate the simulation result using a fixed time step
        auto sampled = sampleODEResult(result, odeopt.t_start, CSV_Ts, t_end);
        vector<EulerAngles> sampledOrientation;
        sampledOrientation.resize(sampled.size());
        transform(sampled.begin(), sampled.end(), sampledOrientation.begin(),
                  NonLinearFullDroneModel::stateToEuler);
        // Export to the given output file
        printCSV(outputFile, 0.0, CSV_Ts, sampledOrientation);
    }

    /* ------ Plot the simulation result ------------------------------------ */
    // Time and state solutions
    auto t    = result.time;
    auto data = result.solution;

    // Calculate controller output
    auto u = controller.getControlSignal(t, data, ref);

    // Convert the quaternions of the state to euler angles
    vector<EulerAngles> orientation;
    orientation.resize(data.size());
    transform(data.begin(), data.end(), orientation.begin(),
              NonLinearFullDroneModel::stateToEuler);

    // Convert the quaternions of the reference to euler angles
    vector<EulerAngles> reference;
    reference.resize(t.size());
    transform(t.begin(), t.end(), reference.begin(), [&ref](double t) {
        return quat2eul(getBlock<0, 4, 0, 1>(ref(t)));
    });

    // Plot all results
    plt::subplot(5, 1, 1);
    plotResults(t, reference, {0, 3}, {"z", "y", "x"}, {"b-", "g-", "r-"},
                "Reference orientation");
    plt::xlim(odeopt.t_start, odeopt.t_end + 1);
    plt::subplot(5, 1, 2);
    plotResults(t, orientation, {0, 3}, {"z", "y", "x"}, {"b-", "g-", "r-"},
                "Orientation of drone");
    plt::xlim(odeopt.t_start, odeopt.t_end + 1);
    plt::subplot(5, 1, 3);
    plotResults(t, data, {4, 7}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Angular velocity of drone");
    plt::xlim(odeopt.t_start, odeopt.t_end + 1);
    plt::subplot(5, 1, 4);
    plotResults(t, data, {7, 10}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Angular velocity of motors");
    plt::xlim(odeopt.t_start, odeopt.t_end + 1);
    plt::subplot(5, 1, 5);
    plotResults(t, u, {0, 3}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Control signal");
    plt::xlim(odeopt.t_start, odeopt.t_end + 1);

    plt::tight_layout();
    plt::show();

    /* ---------------------------------------------------------------------- */

    const RowVector<drone.Nu> covarDynamics    = 1e-9 * ones<1, drone.Nu>();
    const RowVector<drone.Ny - 1> covarSensors = 1e-2 * ones<1, drone.Ny - 1>();

    auto kalman = drone.getDiscreteObserver(covarDynamics, covarSensors, Ts,
                                            DiscretizationMethod::Bilinear);

    FunctionalTimeFunctionT<ColVector<drone.Nu>> randFnW = {
        [&covarDynamics](double /* t */) {
            return randn(covarDynamics[0].data);
        }  //
    };
    FunctionalTimeFunctionT<ColVector<drone.Ny>> randFnV = {
        [&covarSensors](double /* t */) {
            return vcat(zeros<1, 1>(), randn(covarSensors[0].data));
        }  //
    };

    /** [1 0 0 0 0 0 0] */
    ConstantTimeFunctionT<ColVector<drone.Ny>> ref0 =
        vcat(ones<1, 1>(), zeros<drone.Ny - 1, 1>());

    // auto refObs = ref0;
    auto refObs = ref;

    auto obsRes = model.simulate(controller, kalman, randFnW, randFnV, refObs,
                                 x0, odeopt);

    // Convert the quaternions of the reference to euler angles
    vector<EulerAngles> referenceObs;
    referenceObs.resize(obsRes.sampledTime.size());
    transform(obsRes.sampledTime.begin(), obsRes.sampledTime.end(),
              referenceObs.begin(),
              [&refObs](double t) {
                  return quat2eul(getBlock<0, 4, 0, 1>(refObs(t)));
              }  //
    );

    // Convert the quaternions of the state to euler angles
    vector<EulerAngles> sampledOrientation;
    sampledOrientation.resize(obsRes.estimatedSolution.size());
    transform(obsRes.estimatedSolution.begin(), obsRes.estimatedSolution.end(),
              sampledOrientation.begin(),
              NonLinearFullDroneModel::stateToEuler);

    // Plot all results
    /* Reference */
    plt::subplot(5, 2, 1);
    plotResults(obsRes.sampledTime, referenceObs, {0, 3}, {"z", "y", "x"},
                {"b-", "g-", "r-"}, "Reference orientation");
    plt::xlim(odeopt.t_start, odeopt.t_end + 3);

    /* Output */
    plt::subplot(5, 2, 2);
    plotResults(obsRes.sampledTime, obsRes.output, {0, 7},
                {"q0", "q1", "q2", "q3", "wx", "wy", "wz"}, {},
                "Noisy sensor outputs");
    plt::xlim(odeopt.t_start, odeopt.t_end + 3);

    /* Kalman estimation */
    plt::subplot(5, 2, 3);
    plotResults(obsRes.sampledTime, sampledOrientation, {0, 3}, {"z", "y", "x"},
                {"b-", "g-", "r-"}, "Estimated orientation of drone");
    plt::xlim(odeopt.t_start, odeopt.t_end + 3);

    plt::subplot(5, 2, 5);
    plotResults(obsRes.sampledTime, obsRes.estimatedSolution, {4, 7},
                {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Estimated angular velocity of drone");
    plt::xlim(odeopt.t_start, odeopt.t_end + 3);

    plt::subplot(5, 2, 7);
    plotResults(obsRes.sampledTime, obsRes.estimatedSolution, {7, 10},
                {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Estimated angular velocity of motors");
    plt::xlim(odeopt.t_start, odeopt.t_end + 3);

    /* Control signal */
    plt::subplot(5, 2, 9);
    plotResults(obsRes.sampledTime, obsRes.control, {0, 3}, {"x", "y", "z"},
                {"r-", "g-", "b-"}, "Control signal");
    plt::xlim(odeopt.t_start, odeopt.t_end + 3);

    // Convert the quaternions of the state to euler angles
    vector<EulerAngles> realOrientation;
    realOrientation.resize(obsRes.solution.size());
    transform(obsRes.solution.begin(), obsRes.solution.end(),
              realOrientation.begin(), NonLinearFullDroneModel::stateToEuler);

    /* Real ODE result */
    plt::subplot(5, 2, 4);
    plotResults(obsRes.time, realOrientation, {0, 3}, {"z", "y", "x"},
                {"b-", "g-", "r-"}, "Actual orientation of drone");
    plt::xlim(odeopt.t_start, odeopt.t_end + 3);

    plt::subplot(5, 2, 6);
    plotResults(obsRes.time, obsRes.solution, {4, 7}, {"x", "y", "z"},
                {"r-", "g-", "b-"}, "Actual angular velocity of drone");
    plt::xlim(odeopt.t_start, odeopt.t_end + 3);

    plt::subplot(5, 2, 8);
    plotResults(obsRes.time, obsRes.solution, {7, 10}, {"x", "y", "z"},
                {"r-", "g-", "b-"}, "Actual angular velocity of motors");
    plt::xlim(odeopt.t_start, odeopt.t_end + 3);

    plt::tight_layout();
    plt::show();

    // ------------------------------- //

    plt::subplot(2, 1, 1);
    plotResults(obsRes.time, obsRes.solution, {0, 10},
                {"q0", "q1", "q2", "q3", "wx", "wy", "wz"}, {},
                "Actual states");
    plt::xlim(odeopt.t_start, odeopt.t_end + 3);

    plt::subplot(2, 1, 2);
    plotResults(obsRes.sampledTime, obsRes.estimatedSolution, {0, 10},
                {"q0", "q1", "q2", "q3", "wx", "wy", "wz"}, {},
                "Estimated states");
    plt::xlim(odeopt.t_start, odeopt.t_end + 3);

    plt::tight_layout();
    plt::show();

    // ------------------------------- //

    cout << endl;
    printMATLAB(cout, Q, "Q");
    printMATLAB(cout, R, "R");
    cout << endl;
    printC(cout, Q, "Q");
    printC(cout, R, "R");
    cout << endl;
    printCpp(cout, Q, "Q");
    printCpp(cout, R, "R");
    cout << endl;

    return EXIT_SUCCESS;
}
