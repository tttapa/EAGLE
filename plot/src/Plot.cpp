#include <pybind11/stl.h>

#include <DronePlot.hpp>
#include <Plot.hpp>
#include <PlotHelpers.hpp>

#include <filesystem>

pybind11::object plot(const DronePlottable &result, float w, float h) {
    auto pm     = getPythonPlotModule();
    auto pyplot = pm.attr("plot");

    auto time   = result.time;
    auto dtime  = result.sampledTime;
    auto states = dronePlottableToPythonDict(result);

    return pyplot(time, dtime, states, w, h);
}

void show(pybind11::object fig) {
    auto pm     = getPythonPlotModule();
    auto pyshow = pm.attr("show");

    pyshow(fig);
} 

void save(pybind11::object fig, std::filesystem::path filename) {
    auto pm = getPythonPlotModule();
    auto pysave = pm.attr("save");

    pysave(fig, filename.c_str());
}
