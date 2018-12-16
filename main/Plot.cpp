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

const Array<Format, 5> Format::formats       = {{
    "-",
    "--",
    "-.",
    ".",
    ":",
}};
const Array<ColorSet, 2> ColorSet::colorsets = {{
    {{"r", "g", "b"}},
    {{"tomato", "limegreen", "cornflowerblue"}},
}};

void plotDrone(const DronePlottable &result, ColorSet c, Format format,
               const std::string &legendSuffix, bool plotReference) {
    constexpr size_t row = 5;
    constexpr size_t col = 2;

    constexpr double marginRight = 0.15;

    assert(!result.time.empty());
    assert(!result.sampledTime.empty());
    assert(!result.states.empty());
    assert(!result.control.empty());
    assert(!result.reference.empty());

    const double t_start = result.time[0];
    const double t_end   = result.time.back();

    const auto rc  = c.reverse();
    const auto fmt = format.repeat(c.size());

    const std::string &lstr = legendSuffix;

    // Plot all results
    if (plotReference) {
        plt::subplot(row, col, 1);
        plotDroneSignal(result.sampledTime, result.reference,
                        &DroneOutput::getOrientationEuler, {"z", "y'", "x\""},
                        fmt, rc, "Reference orientation");
        plt::ylabel("Euler angles [$\\mathrm{rad}$]");
        plt::xlim(t_start, t_end + (t_end - t_start) * marginRight);

        plt::subplot(row, col, 2);
        plotDroneSignal(result.sampledTime, result.reference,
                        &DroneOutput::getPosition, {"x", "y", "z"}, fmt, c,
                        "Reference position");
        plt::ylabel("Position [$m$]");
        plt::xlim(t_start, t_end + (t_end - t_start) * marginRight);
    }

    plt::subplot(row, col, 3);
    plotDroneSignal(result.time, result.states,
                    &DroneState::getOrientationEuler,
                    {"z" + lstr, "y'" + lstr, "x\"" + lstr}, fmt, rc,
                    "Orientation of drone");
    plt::ylabel("Euler angles [$\\mathrm{rad}$]");
    plt::xlim(t_start, t_end + (t_end - t_start) * marginRight);

    plt::subplot(row, col, 5);
    plotDroneSignal(result.time, result.states, &DroneState::getAngularVelocity,
                    {"x" + lstr, "y" + lstr, "z" + lstr}, fmt, c,
                    "Angular velocity of drone");
    plt::ylabel("Angular velocity [$\\mathrm{rad}/s$]");
    plt::xlim(t_start, t_end + (t_end - t_start) * marginRight);

    plt::subplot(row, col, 7);
    plotDroneSignal(result.time, result.states, &DroneState::getMotorSpeed,
                    {"x" + lstr, "y" + lstr, "z" + lstr}, fmt, c,
                    "Angular velocity of torque motors");
    plt::ylabel("Angular velocity [$\\mathrm{rps}$]");
    plt::xlim(t_start, t_end + (t_end - t_start) * marginRight);

    plt::subplot(row, col, 4);
    plotDroneSignal(result.time, result.states, &DroneState::getPosition,
                    {"x" + lstr, "y" + lstr, "z" + lstr}, fmt, c, "Position");
    plt::ylabel("Position [$m$]");
    plt::xlim(t_start, t_end + (t_end - t_start) * marginRight);

    plt::subplot(row, col, 6);
    plotDroneSignal(result.time, result.states, &DroneState::getVelocity,
                    {"x" + lstr, "y" + lstr, "z" + lstr}, fmt, c, "Velocity");
    plt::ylabel("Velocity [$m/s$]");
    plt::xlim(t_start, t_end + (t_end - t_start) * marginRight);

    plt::subplot(row, col, 8);
    plotDroneSignal(result.time, result.states,
                    &DroneState::getThrustMotorSpeed, {"z'" + lstr}, fmt, c,
                    "Angular velocity of thrust motor");
    plt::ylabel("Angular velocity [$\\mathrm{rps}$]");
    plt::xlim(t_start, t_end + (t_end - t_start) * marginRight);

    plt::subplot(row, col, 9);
    plotDroneSignal(result.sampledTime, result.control,
                    &DroneControl::getAttitudeControl,
                    {"x" + lstr, "y" + lstr, "z" + lstr}, fmt, c,
                    "Torque motor control signal");
    plt::xlim(t_start, t_end + (t_end - t_start) * marginRight);
    plt::ylabel("Control signal [-]");
    plt::xlabel("time [$s$]");

    plt::subplot(row, col, 10);
    plotDroneSignal(result.sampledTime, result.control,
                    &DroneControl::getThrustControl, {"t" + lstr}, fmt, c,
                    "Thrust motor control signal");
    plt::xlim(t_start, t_end + (t_end - t_start) * marginRight);
    plt::ylabel("Control signal [-]");
    plt::xlabel("time [$s$]");

    plt::tight_layout();
}