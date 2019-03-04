#pragma once

#include <PlotHelpers.hpp>
#include <StepResponseAnalyzer.hpp>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <string>
#include <vector>

struct IndexRange {
    constexpr IndexRange(size_t start) : start(start), end(start + 1) {}
    constexpr IndexRange(size_t start, size_t end) : start(start), end(end) {}
    size_t start;
    size_t end;
};

template <size_t N>
class StepResponseAnalyzerPlotter : public StepResponseAnalyzer<N> {
  public:
    constexpr StepResponseAnalyzerPlotter(const ColVector<N> &x_ref,
                                          double factor,
                                          const ColVector<N> &x_0,
                                          bool continueToEnd = true)
        : StepResponseAnalyzer<N>{x_ref, factor, x_0},  //
          continueToEnd{continueToEnd} {}

    void plot(pybind11::object axes, IndexRange idx = {0, N},
              std::string title = "", std::vector<std::string> legends = {},
              int colorset = 0) const;

    void plot(pybind11::object axes, std::string title = "",
              std::vector<std::string> legends = {}, int colorset = 0) const {
        return plot({0, N}, title, legends, colorset);
    }

    bool operator()(double t, const ColVector<N> &x) override {
        t_v.push_back(t);
        x_v.push_back(x);
        return this->calculate(t, x) | continueToEnd;
    }

  private:
    bool continueToEnd;
    std::vector<double> t_v       = {};
    std::vector<ColVector<N>> x_v = {};
};

#include <iostream>

template <size_t N>
void StepResponseAnalyzerPlotter<N>::plot(pybind11::object axes, IndexRange idx,
                                          std::string title,
                                          std::vector<std::string> legends,
                                          int colorset) const {
    if (this->t_v.empty() || this->x_v.empty())
        return;

    const auto result = this->getResult();

    using namespace pybind11::literals;
    using pybind11::dict;
    using pybind11::list;

    std::vector<std::vector<double>> states;
    states.reserve(idx.end - idx.start);
    for (size_t i = idx.start; i < idx.end; ++i)
        states.push_back(Drone::extractSignal(this->x_v, i));

    auto data = dict{
        "times"_a      = this->t_v,
        "states"_a     = states,
        "references"_a = std::vector<double>(this->x_ref.begin() + idx.start,
                                             this->x_ref.begin() + idx.end),
        "thresholds"_a = std::vector<double>(this->x_thr.begin() + idx.start,
                                             this->x_thr.begin() + idx.end),
        "overshoots"_a =
            std::vector<double>(result.overshoot.begin() + idx.start,
                                result.overshoot.begin() + idx.end),

        "risetimes"_a = std::vector<double>(result.risetime.begin() + idx.start,
                                            result.risetime.begin() + idx.end),
        "settletimes"_a =
            std::vector<double>(result.settletime.begin() + idx.start,
                                result.settletime.begin() + idx.end),
    };

    auto pm                    = getPythonPlotModule();
    auto py_plot_step_analyzer = pm.attr("plot_step_analyzer");

    py_plot_step_analyzer(axes, data, title, legends, colorset);
}