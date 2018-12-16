#pragma once

#include "DroneLogLoader.hpp"
#include <Drone/Drone.hpp>
#include <Util/ANSIColors.hpp>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

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
                 const std::vector<std::string> &colors  = {},
                 const std::string &title                = "") {
    assert(idx.start < idx.end);
    assert(idx.end <= N);
    try {
        for (size_t i = 0; i < (idx.end - idx.start); ++i) {
            std::vector<double> plotdata = extractRow(vectors, i + idx.start);
            std::string fmt              = i < formats.size() ? formats[i] : "";
            std::string clr              = i < colors.size() ? colors[i] : "";
            if (i < legends.size())
                plt::named_plot(legends[i], t, plotdata, fmt, clr);
            else
                plt::plot(t, plotdata, fmt, clr);
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
std::pair<double, double> plotDroneSignal(
    const std::vector<double> &t, const std::vector<ColVector<RS>> &signal,
    ColVector<RP> (T::*extractor)() const,
    const std::vector<std::string> &legends = {},
    const std::vector<std::string> &formats = {},
    const std::vector<std::string> &colors = {}, const std::string &title = "",
    double rel_y_margin = 0.05, // TODO: use `matplotlib.pyplot.margins`
    double y_min = std::numeric_limits<double>::infinity(), double y_max = 0) {
    try {
        for (size_t i = 0; i < RP; ++i) {
            std::vector<double> plotdata =
                Drone::extractSignal(signal, extractor, i);
            double curr_y_max =
                *std::max_element(plotdata.begin(), plotdata.end());
            double curr_y_min =
                *std::min_element(plotdata.begin(), plotdata.end());
            if (curr_y_max > y_max)
                y_max = curr_y_max;
            if (curr_y_min < y_min)
                y_min = curr_y_min;
            std::string fmt = i < formats.size() ? formats[i] : "";
            std::string clr = i < colors.size() ? colors[i] : "";
            if (i < legends.size())
                plt::named_plot(legends[i], t, plotdata, fmt, clr);
            else
                plt::plot(t, plotdata, fmt, clr);
        }
        double y_margin = (y_max - y_min) * rel_y_margin;
        plt::ylim(y_min - y_margin, y_max + y_margin);
        if (legends.size() > 0)
            plt::legend();
        if (!title.empty())
            plt::title(title);
    } catch (std::runtime_error &e) {
        std::cerr << ANSIColors::red << "Error: " << e.what()
                  << ANSIColors::reset << std::endl;
    }
    return {y_min, y_max};
}

#define DISCRETE_FMT "-"

class ColorFormatSet {
  public:  // protected: // TODO
    ColorFormatSet(std::vector<std::string> v) : vec{std::move(v)} {}

  public:
    operator std::vector<std::string>() const { return vec; }
    std::string operator[](size_t i) const { return vec[i]; }
    size_t size() const { return vec.size(); }
    const auto begin() const { return vec.begin(); }
    const auto end() const { return vec.end(); }

  private:
    const std::vector<std::string> vec;
};

class Format {
  public:
    template <class T>
    Format(T &&fmt) : fmt{std::forward<T>(fmt)} {}
    Format(int i) : fmt{getFormat(i)} {}

    ColorFormatSet repeat(size_t times) const {
        std::vector<std::string> r(times);
        std::fill(r.begin(), r.end(), *this);
        return r;
    }

    operator std::string() const { return fmt; }
    static Format getFormat(size_t i) { return formats[i]; }

  private:
    const std::string fmt;
    static const Array<Format, 5> formats;
};

class ColorSet : public ColorFormatSet {
  public:
    ColorSet(std::vector<std::string> v) : ColorFormatSet{std::move(v)} {}
    ColorSet(int i) : ColorFormatSet{getColorSet(i)} {}

    ColorSet reverse() const {
        std::vector<std::string> cs = *this;
        std::reverse(cs.begin(), cs.end());
        return cs;
    }
    static ColorSet getColorSet(size_t i) { return colorsets[i]; }
    static ColorSet getReverseColorSet(size_t i) {
        return getColorSet(i).reverse();
    }

  private:
    static const Array<ColorSet, 2> colorsets;
};

void plotDrone(const DronePlottable &result, ColorSet c = 0, Format format = 0,
               const std::string &legendSuffix = "", bool plotReference = true);

inline void plotAttitudeTunerResult(
    Drone::AttitudeModel::ControllerSimulationResult &result) {
    const double t_start = result.time[0];
    const double t_end   = result.time.back();

    const ColorFormatSet fmt  = Format{"-"}.repeat(3);
    const ColorFormatSet dfmt = Format{".-"}.repeat(3);
    const ColorSet c          = ColorSet::getColorSet(0);
    const ColorSet rc         = ColorSet::getReverseColorSet(0);

    plt::subplot(4, 1, 1);
    plotDroneSignal(result.time, result.solution,
                    &DroneAttitudeState::getOrientationEuler,
                    {"z", "y'", "x\""}, fmt, rc, "Orientation of drone");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(4, 1, 2);
    plotDroneSignal(result.time, result.solution,
                    &DroneAttitudeState::getAngularVelocity, {"x", "y", "z"},
                    fmt, c, "Angular velocity of drone");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(4, 1, 3);
    plotDroneSignal(result.time, result.solution,
                    &DroneAttitudeState::getMotorSpeed, {"x", "y", "z"}, fmt, c,
                    "Angular velocity of torque motors");
    plt::xlim(t_start, t_end * 1.1);

    plt::subplot(4, 1, 4);
    plotDroneSignal(result.sampledTime, result.control,
                    &DroneControl::getAttitudeControl, {"x", "y", "z"}, dfmt, c,
                    "Torque motor control");
    plt::xlim(t_start, t_end * 1.1);
    plt::xlabel("time [s]");
}