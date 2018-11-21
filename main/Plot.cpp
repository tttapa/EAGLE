#include "Plot.hpp"

std::vector<double> makeTimeVector(double t_start, double Ts, double t_end) {
    size_t N = floor((t_end - t_start) / Ts) + 1;
    std::vector<double> timevector;
    timevector.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        timevector.push_back(t_start + Ts * i);
    }
    return timevector;
}

void plotDrone(const Drone::ControllerSimulationResult &result) {
    constexpr size_t r = 5;
    constexpr size_t c = 2;

    const double t_start = result.time[0];
    const double t_end   = result.time.back();

    plt::figure();

    // Plot all results
    plt::subplot(r, c, 1);
    plt::tight_layout();
    plotDroneSignal(result.sampledTime, result.reference,
                    &DroneOutput::getOrientationEuler, {"z", "y'", "x\""},
                    {"b" DISCRETE_FMT, "g" DISCRETE_FMT, "r" DISCRETE_FMT},
                    "Reference orientation");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(r, c, 2);
    plotDroneSignal(result.sampledTime, result.reference,
                    &DroneOutput::getPosition, {"z", "y", "x"},
                    {"b" DISCRETE_FMT, "g" DISCRETE_FMT, "r" DISCRETE_FMT},
                    "Reference position");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(r, c, 3);
    plotDroneSignal(result.time, result.solution,
                    &DroneState::getOrientationEuler, {"z", "y'", "x\""},
                    {"b-", "g-", "r-"}, "Orientation of drone");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(r, c, 5);
    plotDroneSignal(result.time, result.solution,
                    &DroneState::getAngularVelocity, {"x", "y", "z"},
                    {"r-", "g-", "b-"}, "Angular velocity of drone");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(r, c, 7);
    plotDroneSignal(result.time, result.solution, &DroneState::getMotorSpeed,
                    {"x", "y", "z"}, {"r-", "g-", "b-"},
                    "Angular velocity of torque motors");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(r, c, 4);
    plotDroneSignal(result.time, result.solution, &DroneState::getPosition,
                    {"x", "y", "z"}, {"r-", "g-", "b-"}, "Position");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(r, c, 6);
    plotDroneSignal(result.time, result.solution, &DroneState::getVelocity,
                    {"x", "y", "z"}, {"r-", "g-", "b-"}, "Velocity");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(r, c, 8);
    plotDroneSignal(result.time, result.solution,
                    &DroneState::getThrustMotorSpeed, {"z'"}, {"r-"},
                    "Angular velocity of thrust motor");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(r, c, 9);
    plotDroneSignal(result.sampledTime, result.control,
                    &DroneControl::getAttitudeControl, {"x", "y", "z"},
                    {"r" DISCRETE_FMT, "g" DISCRETE_FMT, "b" DISCRETE_FMT},
                    "Torque motor control");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(r, c, 10);
    plotDroneSignal(result.sampledTime, result.control,
                    &DroneControl::getThrustControl, {"t"}, {"r" DISCRETE_FMT},
                    "Thrust motor control");
    plt::xlim(t_start, t_end * 1.1);
}