#pragma once

/* TODO
#include <pybind11/embed.h>

class Plotter {
  protected:
    pybind11::module sys = pybind11::module::import("sys");
    pybind11::module plt = pybind11::module::import("matplotlib.pyplot");
    Plotter() { pybind11::print(sys.attr("version")); }

  public:
    Plotter(Plotter const &) = delete;
    void operator=(Plotter const &) = delete;
    
    static Plotter &getInstance() {
        static Plotter plt = {};
        return plt;
    }

    using attr_t     = pybind11::detail::str_attr_accessor;
    using function_t = attr_t;

    function_t plot       = plt.attr("plot");
    function_t named_plot = plt.attr("named_plot");
    function_t show       = plt.attr("show");

    function_t xlabel = plt.attr("xlabel");
    function_t ylabel = plt.attr("ylabel");

    attr_t operator[](const char *a) { return plt.attr(a); }
};
*/