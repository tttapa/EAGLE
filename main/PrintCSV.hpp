#include <fstream>
#include <iostream>

#include <ODE/ODEResult.hpp>

template <class T, size_t N>
void printCSV(std::ostream &out, const ODEResultX<TColVector<T, N>> &result) {
    for (size_t i = 0; i < result.time.size(); ++i) {
        out << result.time[i] << ',';
        for (size_t j = 0; j < N - 1; ++j)
            out << result.solution[i][j][0] << ',';
        out << result.solution[i][N - 1][0] << "\r\n";
    }
    out << std::endl;
}

template <class T, size_t N>
void printCSV(std::ostream &out, double t_start, double Ts,
              const std::vector<TColVector<T, N>> &samples) {
    double t = t_start;
    for (size_t i = 0; i < samples.size(); ++i) {
        out << t << ',';
        for (size_t j = 0; j < N - 1; ++j)
            out << samples[i][j][0] << ',';
        out << samples[i][N - 1][0] << "\r\n";
        t += Ts;
    }
    out << std::endl;
}

template <class... Args>
void printCSV(const std::string &filename, Args... args) {
    std::ofstream ofile;
    ofile.open(filename);
    if (!ofile)
        std::cerr << "Error opening file" << std::endl;
    else
        printCSV(ofile, args...);
    ofile.close();
}