#pragma once

#include <Drone/Drone.hpp>
#include <Util/ANSIColors.hpp>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

std::vector<double> makeTimeVector(double t_start, double Ts, double t_end);

template <class T, size_t N>
std::vector<T> extractRow(const std::vector<TColVector<T, N>> &in, size_t row) {
    std::vector<T> out;
    out.reserve(in.size());
    for (const auto &el : in)
        out.push_back(el[row]);
    return out;
}

struct IndexRange {
    constexpr IndexRange(size_t start) : start(start), end(start + 1) {}
    constexpr IndexRange(size_t start, size_t end) : start(start), end(end) {}
    size_t start;
    size_t end;
};

template <size_t N>
void plotVectors(const std::vector<double> &t,
                 const std::vector<ColVector<N>> &vectors,
                 const IndexRange idx                    = {0, N},
                 const std::vector<std::string> &legends = {},
                 const std::vector<std::string> &formats = {},
                 const std::string &title                = "") {
    assert(idx.start < idx.end);
    assert(idx.end <= N);
    try {
        for (size_t i = 0; i < (idx.end - idx.start); ++i) {
            std::vector<double> plotdata = extractRow(vectors, i + idx.start);
            std::string fmt              = i < formats.size() ? formats[i] : "";
            if (i < legends.size())
                plt::named_plot(legends[i], t, plotdata, fmt);
            else
                plt::plot(t, plotdata, fmt);
        }
        if (!legends.empty())
            plt::legend();
        if (!title.empty())
            plt::title(title);
    } catch (std::runtime_error &e) {
        std::cerr << ANSIColors::red << "Error: " << e.what()
                  << ANSIColors::reset << std::endl;
    }
}

template <class T, size_t RS, size_t RP>
void plotDroneSignal(const std::vector<double> &t,
                     const std::vector<ColVector<RS>> &signal,
                     ColVector<RP> (T::*extractor)() const,
                     const std::vector<std::string> &legends = {},
                     const std::vector<std::string> &formats = {},
                     const std::string &title                = "") {
    try {
        for (size_t i = 0; i < RP; ++i) {
            std::vector<double> plotdata =
                Drone::extractSignal(signal, extractor, i);
            std::string fmt = i < formats.size() ? formats[i] : "";
            if (i < legends.size())
                plt::named_plot(legends[i], t, plotdata, fmt);
            else
                plt::plot(t, plotdata, fmt);
        }
        if (legends.size() > 0)
            plt::legend();
        if (!title.empty())
            plt::title(title);
    } catch (std::runtime_error &e) {
        std::cerr << ANSIColors::red << "Error: " << e.what()
                  << ANSIColors::reset << std::endl;
    }
}

#define DISCRETE_FMT "-"

void plotDrone(const Drone::ControllerSimulationResult &result);

inline void plotAttitudeTunerResult(
    Drone::AttitudeModel::ControllerSimulationResult &result) {
    const double t_start = result.time[0];
    const double t_end   = result.time.back();

    plt::subplot(4, 1, 1);
    plotDroneSignal(
        result.time, result.solution, &DroneAttitudeState::getOrientationEuler,
        {"z", "y'", "x\""}, {"b-", "g-", "r-"}, "Orientation of drone");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(4, 1, 2);
    plotDroneSignal(result.time, result.solution,
                    &DroneAttitudeState::getAngularVelocity, {"x", "y", "z"},
                    {"r-", "g-", "b-"}, "Angular velocity of drone");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(4, 1, 3);
    plotDroneSignal(result.time, result.solution,
                    &DroneAttitudeState::getMotorSpeed, {"x", "y", "z"},
                    {"r-", "g-", "b-"}, "Angular velocity of torque motors");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(4, 1, 4);
    plotDroneSignal(result.sampledTime, result.control,
                    &DroneControl::getAttitudeControl, {"x", "y", "z"},
                    {"r" DISCRETE_FMT, "g" DISCRETE_FMT, "b" DISCRETE_FMT},
                    "Torque motor control");
    plt::xlim(t_start, t_end * 1.1);
    plt::xlabel("time [s]");
}