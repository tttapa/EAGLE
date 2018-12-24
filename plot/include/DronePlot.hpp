#pragma once

#include <ANSIColors.hpp>
#include <Drone.hpp>
#include <DroneLogLoader.hpp>

struct DronePlottable {
    DronePlottable(const Drone::ControllerSimulationResult &d)
        : time{d.time}, sampledTime{d.sampledTime}, states{d.solution},
          control{d.control}, reference{d.reference} {}
    DronePlottable(const DroneLogLoader &d)
        : time{d.getTimeStamps()}, sampledTime{time}, states{d.getStates()},
          control{d.getControl()}, reference{d.getReference()} {}
    DronePlottable() = default;

    std::vector<double> time;
    std::vector<double> sampledTime;
    std::vector<Drone::VecX_t> states;
    std::vector<Drone::VecU_t> control;
    std::vector<Drone::VecR_t> reference;
};

void plot(const std::vector<double> x, const std::vector<double> &y,
          std::string_view color = "", std::string_view format = "",
          double linewidth = 1);

void plot(const std::vector<double> x, const std::vector<double> &y,
          std::string_view color = "", std::string_view format = "",
          double linewidth = 1);

template <class T, size_t N>
std::vector<T> extractRow(const std::vector<TColVector<T, N>> &in, size_t row) {
    std::vector<T> out;
    out.reserve(in.size());
    for (const auto &el : in)
        out.push_back(el[row]);
    return out;
}

template <class T, size_t RS, size_t RP>
void plotDroneSignal(const std::vector<double> &t,
                     const std::vector<ColVector<RS>> &signal,
                     ColVector<RP> (T::*extractor)() const,
                     const std::vector<std::string> &legends = {},
                     const std::vector<std::string> &formats = {},
                     const std::vector<std::string> &colors  = {},
                     const std::string &title                = "") {
    for (size_t i = 0; i < RP; ++i) {
        std::vector<double> plotdata =
            Drone::extractSignal(signal, extractor, i);
        std::string fmt = i < formats.size() ? formats[i] : "";
        std::string clr = i < colors.size() ? colors[i] : "";
        if (i < legends.size())
            named_plot(legends[i], t, plotdata, fmt, clr);
        else
            plt::plot(t, plotdata, fmt, clr);
    }
    if (legends.size() > 0)
        plt::legend();
    if (!title.empty())
        plt::title(title);
}

void plotDrone(const DronePlottable &result,
               const std::string &legendSuffix = "", bool plotReference = true);