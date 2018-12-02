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

void plotDrone(const Drone::ControllerSimulationResult &result, int colorset) {
    constexpr size_t row = 5;
    constexpr size_t col = 2;

    const double t_start = result.time[0];
    const double t_end   = result.time.back();

    const auto c  = colorsets.at(colorset);
    const auto rc = rcolorsets.at(colorset);

    const std::string istr = " (" + std::to_string(colorset) + ")";

    // Plot all results
    plt::subplot(row, col, 1);
    plt::tight_layout();
    plotDroneSignal(
        result.sampledTime, result.reference, &DroneOutput::getOrientationEuler,
        {"z" + istr, "y'" + istr, "x\"" + istr}, rc, "Reference orientation");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(row, col, 2);
    plotDroneSignal(
        result.sampledTime, result.reference, &DroneOutput::getPosition,
        {"x" + istr, "y" + istr, "z" + istr}, c, "Reference position");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(row, col, 3);
    plotDroneSignal(
        result.time, result.solution, &DroneState::getOrientationEuler,
        {"z" + istr, "y'" + istr, "x\"" + istr}, rc, "Orientation of drone");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(row, col, 5);
    plotDroneSignal(
        result.time, result.solution, &DroneState::getAngularVelocity,
        {"x" + istr, "y" + istr, "z" + istr}, c, "Angular velocity of drone");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(row, col, 7);
    plotDroneSignal(result.time, result.solution, &DroneState::getMotorSpeed,
                    {"x" + istr, "y" + istr, "z" + istr}, c,
                    "Angular velocity of torque motors");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(row, col, 4);
    plotDroneSignal(result.time, result.solution, &DroneState::getPosition,
                    {"x" + istr, "y" + istr, "z" + istr}, c, "Position");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(row, col, 6);
    plotDroneSignal(result.time, result.solution, &DroneState::getVelocity,
                    {"x" + istr, "y" + istr, "z" + istr}, c, "Velocity");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(row, col, 8);
    plotDroneSignal(result.time, result.solution,
                    &DroneState::getThrustMotorSpeed, {"z'" + istr}, c,
                    "Angular velocity of thrust motor");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(row, col, 9);
    plotDroneSignal(
        result.sampledTime, result.control, &DroneControl::getAttitudeControl,
        {"x" + istr, "y" + istr, "z" + istr}, c, "Torque motor control");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(row, col, 10);
    plotDroneSignal(result.sampledTime, result.control,
                    &DroneControl::getThrustControl, {"t" + istr}, c,
                    "Thrust motor control");
    plt::xlim(t_start, t_end * 1.1);
}