#include "PrintBest.hpp"

#include <ANSIColors.hpp>

void printBest(std::ostream &os, size_t generation, const Weights &best) {
    using namespace std;
    os << ANSIColors::cyanb << endl
       << "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━┓\r\n"
       << "┃ " << ANSIColors::whiteb << "Best of Generation #" << setw(4)
       << setfill(' ') << (generation + 1) << "  " << ANSIColors::cyanb
       << "┃\r\n"
          "┡━━━━━━━━━━━━━━━━━━━━━━━━━━━┩\r\n";
    os << "│ " << ANSIColors::whiteb << "Cost = " << scientific
       << setprecision(2) << setw(2 + 7) << best.cost << ANSIColors::cyanb
       << "          │\r\n";
    os.unsetf(ios_base::floatfield);
    os << "│ " << ANSIColors::whiteb << "Qq = {{" << ANSIColors::cyanb
       << "                   │\r\n";
    for (double q : getBlock<0, 3, 0, 1>(best.Q_diag))
        os << "│   " << ANSIColors::whiteb << setprecision(16) << setw(16 + 6)
           << setfill(' ') << q << "," << ANSIColors::cyanb << " │\r\n";
    os << "│ " << ANSIColors::whiteb << "}};" << ANSIColors::cyanb
       << "                       │\r\n";
    os << "│ " << ANSIColors::whiteb << "Qomega = {{" << ANSIColors::cyanb
       << "               │\r\n";
    for (double w : getBlock<3, 6, 0, 1>(best.Q_diag))
        os << "│   " << ANSIColors::whiteb << setprecision(16) << setw(16 + 6)
           << setfill(' ') << w << "," << ANSIColors::cyanb << " │\r\n";
    os << "│ " << ANSIColors::whiteb << "}};" << ANSIColors::cyanb
       << "                       │\r\n";
    os << "│ " << ANSIColors::whiteb << "Qn = {{" << ANSIColors::cyanb
       << "                   │\r\n";
    for (double n : getBlock<6, 9, 0, 1>(best.Q_diag))
        os << "│   " << ANSIColors::whiteb << setprecision(16) << setw(16 + 6)
           << setfill(' ') << n << "," << ANSIColors::cyanb << " │\r\n";
    os << "│ " << ANSIColors::whiteb << "}};" << ANSIColors::cyanb
       << "                       │\r\n";
    os << "│ " << ANSIColors::whiteb << "Ru = {{" << ANSIColors::cyanb
       << "                   │\r\n";
    for (double r : best.R_diag)
        os << "│   " << ANSIColors::whiteb << setprecision(16) << setw(16 + 6)
           << setfill(' ') << r << "," << ANSIColors::cyanb << " │\r\n";
    os << "│ " << ANSIColors::whiteb << "}};" << ANSIColors::cyanb
       << "                       │\r\n";
    os << "└───────────────────────────┘\r\n" << ANSIColors::reset;
}

#include <fstream>
#include <iostream>

void appendBestToFile(const std::filesystem::path &filename, size_t generation,
                      const Weights &best) {
    std::ofstream ofile;
    ofile.open(filename, std::ios_base::app);
    if (!ofile)
        std::cerr << ANSIColors::red << "Error opening file: `" << filename
                  << "`" << ANSIColors::reset << std::endl;
    else
        ofile << (generation == 0 ? "\r\n---\r\n\r\n" : "") << (generation + 1)
              << ':' << asrowvector(best.Q_diag, ",", 16) << '\t'
              << asrowvector(best.R_diag, ",", 16) << std::endl;
    ofile.close();
}