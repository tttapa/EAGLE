#include "InputSignals.hpp"
#include "PrintCSV.hpp"
#include <Matrix/DLQE.hpp>
#include <Matrix/DLQR.hpp>
#include <Matrix/LQR.hpp>
#include <Matrix/Randn.hpp>
#include <Model/System.hpp>
#include <ODE/ODEEval.hpp>
#include <Util/AlmostEqual.hpp>
#include <iostream>

#include "ANSIColors.hpp"
#include "Config.hpp"
#include "Plot.hpp"

#include "Generated/GeneratedController.hpp"

#include "CodeGen/CodeGen.hpp"

#include <debug.hpp>

using namespace std;
using namespace Config;

int main(int argc, char const *argv[]) {
    (void) argc, (void) argv;

#if 1

    /* ------ Drone full non-linear and linear models ----------------------- */
    auto model = drone.getNonLinearFullModel();
    // auto model = drone.getLinearFullContinuousModel();

    /* ------ Design controller --------------------------------------------- */
    // auto controller = drone.getContinuousController(Q, R);
    auto clampedController = drone.getClampedDiscreteController(
        clampMin, clampMax, Q, R, Ts, DiscretizationMethod::Bilinear);
    auto discreteControler =
        drone.getDiscreteController(Q, R, Ts, DiscretizationMethod::Bilinear);
    DiscreteLQRController &controller =
        clampController ? clampedController : discreteControler;

    GeneratedLQRController generatedController = {};

    /* ------ Reference function -------------------------------------------- */
    TestReferenceFunction ref = {};

    /* ------ Simulate the drone with the controller ------------------------ */
    auto result = model.simulate(controller, ref, x0, odeopt);

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

    auto generatedResult = model.simulate(generatedController, ref, x0, odeopt);
    if (generatedResult.resultCode &
        ODEResultCodes::MAXIMUM_ITERATIONS_EXCEEDED) {
        std::cerr << ANSIColors::redb
                  << "Error: maximum number of iterations exceeded"
                  << ANSIColors::reset << endl;
        t_end = result.time.back();
    }
    if (generatedResult.resultCode & ODEResultCodes::MINIMUM_STEP_SIZE_REACHED)
        std::cerr << ANSIColors::yellow << "Warning: minimum step size reached"
                  << ANSIColors::reset << endl;

    /* ------ Plot the simulation result ------------------------------------ */
    // Time and state solutions
    auto t    = result.time;
    auto data = result.solution;

    auto generatedTime = generatedResult.time;
    auto generatedData = generatedResult.solution;

    // Calculate controller output
    auto u = controller.getControlSignal(t, data, ref);
    auto generatedU =
        generatedController.getControlSignal(generatedTime, generatedData, ref);

    // Convert the quaternions of the state to euler angles
    vector<EulerAngles> orientation =
        NonLinearFullDroneModel::statesToEuler(data);

    // Convert the quaternions of the state to euler angles
    vector<EulerAngles> generatedOrientation =
        NonLinearFullDroneModel::statesToEuler(generatedData);

    // Convert the quaternions of the reference to euler angles
    vector<EulerAngles> refOrientation;
    refOrientation.resize(t.size());
    transform(t.begin(), t.end(), refOrientation.begin(), [&ref](double t) {
        return quat2eul(getBlock<0, 4, 0, 1>(ref(t)));
    });

#ifdef PLOT_CONTROLLER

    // Plot all results
    plt::subplot(5, 1, 1);
    plotResults(t, refOrientation, {0, 3}, {"z", "y", "x"}, {"b-", "g-", "r-"},
                "Reference orientation");
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
    plt::subplot(5, 1, 2);
    plotResults(t, orientation, {0, 3}, {"z", "y", "x"}, {"b-", "g-", "r-"},
                "Orientation of drone");
    plotResults(generatedTime, generatedOrientation, {0, 3},
                {"generated z", "generated y", "generated x"},
                {"b--", "g--", "r--"});
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
    plt::subplot(5, 1, 3);
    plotResults(t, data, {4, 7}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Angular velocity of drone");
    plotResults(generatedTime, generatedData, {4, 7},
                {"generated x", "generated y", "generated z"},
                {"r--", "g--", "b--"});
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
    plt::subplot(5, 1, 4);
    plotResults(t, data, {7, 10}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Angular velocity of motors");
    plotResults(generatedTime, generatedData, {7, 10},
                {"generated x", "generated y", "generated z"},
                {"r--", "g--", "b--"});
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
    plt::subplot(5, 1, 5);
    plotResults(t, u, {0, 3}, {"x", "y", "z"}, {"r-", "g-", "b-"},
                "Control signal");
    plotResults(generatedTime, generatedU, {0, 3},
                {"generated x", "generated y", "generated z"},
                {"r--", "g--", "b--"});
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);

    plt::tight_layout();
    plt::show();

#endif

    /* ---------------------------------------------------------------------- */

#ifdef PLOT_OBSERVER

    auto kalman = drone.getDiscreteObserver(varDynamics, varSensors, Ts,
                                            DiscretizationMethod::Bilinear);

    FunctionalTimeFunctionT<ColVector<drone.Nu>> randFnW = {
        [](double /* t */) {
            return randn(varDynamics[0].data);  //
        }                                       //
    };
    FunctionalTimeFunctionT<ColVector<drone.Ny>> randFnV = {
        [](double /* t */) {
            // Gaussian noise + drift
            auto varOrientation = getBlock<0, 1, 0, 3>(varSensors);
            auto varAngVelocity = getBlock<0, 1, 3, 6>(varSensors);
            EulerAngles randOrientation =
                randn(varOrientation[0]
                          .data);  // + 2.0 * t * transpose(varOrientation);
            ColVector<3> randAngVelocity =
                randn(varAngVelocity[0]
                          .data);  // + 1.0 * t * transpose(varAngVelocity);
            return vcat(           //
                eul2quat(randOrientation),  //
                randAngVelocity             //
            );                              //
        }                                   //
    };

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
    vector<EulerAngles> sampledOrientation =
        NonLinearFullDroneModel::statesToEuler(obsRes.estimatedSolution);

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
    vector<EulerAngles> realOrientation =
        NonLinearFullDroneModel::statesToEuler(obsRes.solution);

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

#endif

    // ------------------------------- //

#if PLOT_ACTUAL_VS_ESTIMATED_STATES
    plt::subplot(3, 1, 1);
    plotResults(obsRes.time, obsRes.solution, {0, 4},
                {"Actual q0", "Actual q1", "Actual q2", "Actual q3"},
                {"b", "g", "r", "y"}, "Orientation Quaternion");
    plotResults(
        obsRes.sampledTime, obsRes.estimatedSolution, {0, 4},
        {"Estimated q0", "Estimated q1", "Estimated q2", "Estimated q3"},
        {"--b", "--g", "--r", "--y"});
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);

    plt::subplot(3, 1, 2);
    plotResults(obsRes.time, obsRes.solution, {4, 7},
                {"Actual wx", "Actual wy", "Actual wz"}, {"b", "g", "r"},
                "Angular velocity of drone");
    plotResults(obsRes.sampledTime, obsRes.estimatedSolution, {4, 7},
                {"Estimated wx", "Estimated wy", "Estimated wz"},
                {"--b", "--g", "--r"});
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);

    plt::subplot(3, 1, 3);
    plotResults(obsRes.time, obsRes.solution, {7, 10},
                {"Actual nx", "Actual ny", "Actual nz"}, {"b", "g", "r"},
                "Angular velocity of motors");
    plotResults(obsRes.sampledTime, obsRes.estimatedSolution, {7, 10},
                {"Estimated nx", "Estimated ny", "Estimated nz"},
                {"--b", "--g", "--r"});
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);

    plt::tight_layout();
    plt::show();

    plotResults(obsRes.time, obsRes.solution, {0, 1}, {"Actual q0"}, {"b"});
    plotResults(obsRes.sampledTime, obsRes.estimatedSolution, {0, 1},
                {"Estimated q0"}, {"r"});
    plt::xlim(odeopt.t_start, odeopt.t_end * 1.1);
    plt::show();

    cout << fixed << setprecision(17) << obsRes.quatnorms.back() << endl;
    plt::plot(obsRes.sampledTime, obsRes.quatnorms);
    plt::title("Norm of ode45 quaternion");
    plt::ylim(0.0, 1.1);
    plt::show();

#endif

    /* ------ Export the simulation result as CSV --------------------------- */

    if (exportCSV) {
        // Sample/interpolate the simulation result using a fixed time step
        auto sampled = sampleODEResult(obsRes, odeopt.t_start, CSV_Ts, t_end);
        vector<EulerAngles> sampledOrientation =
            NonLinearFullDroneModel::statesToEuler(sampled);
        // Export to the given output file
        printCSV(outputFile, 0.0, CSV_Ts, sampledOrientation);
    }

    /* ------ Test the generated result vs the expected result -------------- */

    const auto sampledExpected =
        sampleODEResult(result, odeopt.t_start, Ts, t_end);
    const auto sampledGenerated =
        sampleODEResult(generatedResult, odeopt.t_start, Ts, t_end);

    assert(sampledExpected.size() == sampledGenerated.size());
    bool equal = true;
    for (size_t i = 0; i < sampledExpected.size(); ++i)
        if (!isAlmostEqual(sampledExpected[i], sampledGenerated[i], 1e-8)) {
            equal = false;
            break;
        }

    if (equal)
        cout << ANSIColors::greenb
             << "Success: Generated controller matches full controller"
             << ANSIColors::reset << endl;
    else
        cout << ANSIColors::redb
             << "Error: Generated controller does not match full controller"
             << ANSIColors::reset << endl;

        // ---------------------------------------------------------------------- //

#endif

#if 0
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
#endif

#if 0
    auto method    = DiscretizationMethod::Bilinear;
    auto redsys    = drone.getLinearReducedDiscreteSystem(Ts, method);
    auto ATT_A_red = redsys.A;
    auto ATT_B_red = redsys.B;
    auto ATT_LQR_red =
        drone.getReducedDiscreteControllerMatrixK(Q, R, Ts, method);
    auto ATT_G_red = LQRController::calculateG(redsys.A, redsys.B, redsys.C,
                                               redsys.D, false);
    auto ATT_L_red = drone.getReducedDiscreteObserverMatrixL(
        varDynamics, varSensors, Ts, method);

    cout << "A_red = " << ATT_A_red << endl;
    cout << "B_red = " << ATT_B_red << endl;
    cout << "K_red = " << ATT_LQR_red << endl;
    cout << "Q = " << Q << endl;
    cout << "R = " << R << endl;
    cout << "G = " << controller.G << endl;
    cout << "L = " << ATT_L_red << endl;

    std::map<string, DynamicMatrix> matmap = {};
    matmap.insert(std::make_pair("ATT_A", ATT_A_red));
    matmap.insert(std::make_pair("ATT_B", ATT_B_red));
    matmap.insert(std::make_pair("ATT_G", ATT_G_red));
    matmap.insert(std::make_pair("ATT_L", ATT_L_red));
    matmap.insert(std::make_pair("ATT_LQR", ATT_LQR_red));

    replaceTagsInFile(home + "/tmp/testTags.txt",
                      home + "/tmp/testTags.out.txt", matmap);
#endif

    return equal ? EXIT_SUCCESS : EXIT_FAILURE;
}

void mydebug() { std::cerr << "ERROR" << std::endl; }