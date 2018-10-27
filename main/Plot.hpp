#include <Matrix/Matrix.hpp>
#include <cmath>  // floor
#include <matplotlibcpp.h>
#include <vector>

namespace plt = matplotlibcpp;

std::vector<double> makeTimeVector(double t_start, double Ts, double t_end) {
    size_t N = floor((t_end - t_start) / Ts) + 1;
    std::vector<double> timevector;
    timevector.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        timevector.push_back(t_start + Ts * i);
    }
    return timevector;
}

template <class T, size_t N>
std::vector<T> extractRow(const std::vector<TColVector<T, N>> &in, size_t row) {
    std::vector<T> out;
    out.reserve(in.size());
    for (const auto &el : in)
        out.push_back(el[row][0]);
    return out;
}

struct IndexRange {
    IndexRange(size_t start) : start(start), end(start + 1) {}
    IndexRange(size_t start, size_t end) : start(start), end(end) {}
    size_t start;
    size_t end;
};

template <size_t N>
void plotResults(const std::vector<double> &t,
                 const std::vector<TColVector<double, N>> &vectors,
                 const IndexRange idx                    = {0, N},
                 const std::vector<std::string> &legends = {},
                 const std::vector<std::string> &formats = {},
                 const std::string &title                = "") {
    assert(idx.start < idx.end);
    assert(idx.end <= N);
    for (size_t i = 0; i < (idx.end - idx.start); ++i) {
        std::vector<double> plotdata = extractRow(vectors, i + idx.start);
        std::string fmt              = i < formats.size() ? formats[i] : "";
        if (i < legends.size())
            plt::named_plot(legends[i], t, plotdata, fmt);
        else
            plt::plot(t, plotdata, fmt);
    }
    if (legends.size() > 0)
        try {
            plt::legend();
        } catch (std::runtime_error &e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    try {
        plt::title(title);
    } catch (std::runtime_error &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    plt::title(title);
}