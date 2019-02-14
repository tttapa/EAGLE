#include <pybind11/stl.h>

#include <DronePlot.hpp>
#include <Plot.hpp>
#include <PlotHelpers.hpp>

#include <filesystem>

pybind11::object plot(const DronePlottable &result, float w, float h,
                      int colors, std::string title) {
    auto pm     = getPythonPlotModule();
    auto pyplot = pm.attr("plot");

    auto time   = result.time;
    auto dtime  = result.sampledTime;
    auto states = dronePlottableToPythonDict(result);

    return pyplot(time, dtime, states, w, h, colors, title);
}

void show(pybind11::object fig) {
    auto pm     = getPythonPlotModule();
    auto pyshow = pm.attr("show");

    pyshow(fig);
}

void save(pybind11::object fig, std::filesystem::path filename) {
    auto pm     = getPythonPlotModule();
    auto pysave = pm.attr("save");

    pysave(fig, filename.c_str());
}

pybind11::object figure(float w, float h) {
    auto pm       = getPythonPlotModule();
    auto pyfigure = pm.attr("figure");

    return pyfigure(w, h);
}

pybind11::object axes(float w, float h) {
    auto pm     = getPythonPlotModule();
    auto pyaxes = pm.attr("axes");

    return pyaxes(w, h);
}