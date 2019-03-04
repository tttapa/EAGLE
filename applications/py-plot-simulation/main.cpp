#include <PlotHelpers.hpp>
#include <PyDrone.hpp>

namespace py = pybind11;
using namespace std;

/**
 * @brief   Load the Python Plot module, and the Drone module.
 * 
 * @return  A dict containing the Plot module as `plot` and the Drone module as 
 *          `drone`.
 */
pybind11::dict getModules();

filesystem::path getSourceFolder() {
    std::filesystem::path filepath = __FILE__;
    return filepath.parent_path();
}

int main() {
    py::scoped_interpreter guard{};  // Start the Python interpreter
    auto locals  = getModules();
    auto main_py = getSourceFolder() / "main.py";
    pybind11::globals().attr("update")(locals);
    py::eval_file(main_py.c_str(), pybind11::globals());
}

pybind11::dict getModules() {
    using namespace py::literals;

    return pybind11::dict{
        // Plot
        "plot"_a = getPythonPlotModule(),

        // PyDrone
        "drone"_a = py::module::import("PyDrone"),
    };

    // // Main.py
    // std::filesystem::path filepath = __FILE__;
    // auto path                      = filepath.parent_path();
    // py::globals()["path"]          = path.c_str();
    // py::exec(R"(
    //     import sys
    //     sys.path.append(path)
    // )");
    // auto mainmodule = py::module::import("main");
}