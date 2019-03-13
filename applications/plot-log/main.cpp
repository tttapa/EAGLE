#include <pybind11/embed.h>

#include <ArgParser.hpp>
#include <Plot.hpp>
#include <iostream>

using namespace std;

struct Test {
    Test(const std::vector<int> &entries) : entries{entries} {}

    std::vector<int> entries;
    std::vector<int>::const_iterator getFirstFlyingEntry() const {
        return std::find_if(entries.begin(), entries.end(),
                            [](int l) { return l > 0; });
    }

    std::vector<int>::const_reverse_iterator getFinalFlyingEntry() const {
        return std::find_if(entries.rbegin(), entries.rend(),
                            [](int l) { return l > 0; });
    }

    size_t getFirstFlyingIndex() const {
        auto it = getFirstFlyingEntry();
        return std::distance(entries.begin(), it);
    }

    size_t getFinalFlyingIndex() const {
        auto it = getFinalFlyingEntry();
        return std::distance(it, entries.rend());
    }

    Test slice(size_t start_index, size_t end_index) const {
        return {std::vector<int>{
            entries.begin() + start_index,
            entries.begin() + end_index,
        }};
    }

    Test trim() const {
        return slice(getFirstFlyingIndex(), getFinalFlyingIndex());
    }
};

ostream &operator<<(ostream &os, const Test &t) {
    for (const auto &tt : t.entries)
        os << tt << ", ";
    return os << endl;
}

int main(int argc, char const *argv[]) {

    /* ------ Parse command line arguments ---------------------------------- */

    filesystem::path loadPath = "";
    filesystem::path outPath  = "";

    ArgParser parser;
    parser.add("--out", "-o", [&](const char *argv[]) {
        outPath = argv[1];
        cout << "Setting output path to: " << argv[1] << endl;
    });
    parser.add("--load", "-l", [&](const char *argv[]) {
        loadPath = argv[1];
        cout << "Setting load path to: " << argv[1] << endl;
    });
    cout << ANSIColors::blue;
    parser.parse(argc, argv);
    cout << ANSIColors::reset << endl;

    /* -------------------- Start the Python interpreter -------------------- */

    // pybind11::scoped_interpreter guard{};

    /* ------ Load the drone log -------------------------------------------- */

    DroneLogLoader dll = {loadPath};
    DronePlottable dp  = {dll};
    DronePlottable dpp = {dp};
    cout << dll[0].getFrametime() << endl;
    cout << dll[1].getFrametime() << endl;
    cout << dll[2].getFrametime() << endl;

    cout << dpp.time[0] << endl;
    cout << dpp.time[1] << endl;
    cout << dpp.time[2] << endl;

    /* ------ Plot the simulation result ------------------------------------ */

    // Plot and/or save the simulation result
    // auto fig = plot(dp, 1920, 1080, 1, "Simulation");
    // if (!outPath.empty())
    //     save(fig, outPath / "Simulation.svg");
    // show(fig);

    /* ------ Done ---------------------------------------------------------- */

    Test test = {{0, 0, 1, 2, 1, 0, 0}};
    cout << "First: " << test.getFirstFlyingIndex() << endl;
    cout << "Final: " << test.getFinalFlyingIndex() << endl;
    test = {{4, 3, 1, 2, 1, 3, 4}};
    cout << "First: " << test.getFirstFlyingIndex() << endl;
    cout << "Final: " << test.getFinalFlyingIndex() << endl;
    test = {{4, 0}};
    cout << "First: " << test.getFirstFlyingIndex() << endl;
    cout << "Final: " << test.getFinalFlyingIndex() << endl;

    test = {{0, 0, 1, 2, 1, 0, 0}};
    cout << test.trim() << endl;

    cout << "Done." << endl;

    return EXIT_SUCCESS;
}
