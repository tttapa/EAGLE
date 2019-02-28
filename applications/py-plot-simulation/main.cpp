#include <PlotHelpers.hpp>
#include <PyDrone.hpp>

namespace py = pybind11;
using namespace std;

/**
 * @brief   Load the Python Plot module, and the main.py module in the current
 *          folder.  
 *          They will be available in Python as global variables `plot` and 
 *          `main`.  
 *          It also defines the global variable `path`, which is the folder of
 *          this file.
 */
void loadModules();

filesystem::path getSourceFolder() {
    std::filesystem::path filepath = __FILE__;
    return filepath.parent_path();
}

int main() {
    py::scoped_interpreter guard{};  // Start the Python interpreter
    loadModules();
    auto main_py = getSourceFolder() / "main.py";
    py::eval_file(main_py.c_str());
}

void loadModules() {
    using namespace py::literals;

    // Plot
    loadPythonPlotModule();

    // PyDrone
    py::globals()["drone"] = py::module::import("PyDrone");

    // // Main.py
    // std::filesystem::path filepath = __FILE__;
    // auto path                      = filepath.parent_path();
    // py::globals()["path"]          = path.c_str();
    // py::exec(R"(
    //     import sys
    //     sys.path.append(path)
    // )");
    // py::globals()["main"] = py::module::import("main");
}