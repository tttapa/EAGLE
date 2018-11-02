#include "CodeGen.hpp"
#include "../ANSIColors.hpp"
#include <fstream>
#include <iostream>

std::string doubleToString(double dbl, int precision) {
    std::ostringstream strs;
    strs << std::setprecision(precision) << dbl;
    return strs.str();
}

void replaceTagsInFile(const std::string &infilename,
                       const std::string &outfilename,
                       const std::map<std::string, DynamicMatrix> &matrixdict) {
    std::ifstream ifile;
    ifile.open(infilename);
    if (!ifile)
        throw std::runtime_error("Error opening file `" + infilename + "`");
    std::stringstream buffer;
    buffer << ifile.rdbuf();
    ifile.close();
    std::string text = buffer.str();

    const static std::regex tag{"<([a-zA-Z_]+)_(\\d+)_(\\d+)>"};

    std::string res = regex_replace(
        text, tag,  //
        [&matrixdict](const std::smatch &m) -> std::string {
            size_t r         = stoul(m.str(2)) - 1;  // arrays start at 1
            size_t c         = stoul(m.str(3)) - 1;
            std::string name = m.str(1);
            if (matrixdict.count(name)) {
                std::string valuestr =
                    doubleToString(matrixdict.at(name).at(r).at(c), 17);
                return valuestr + " /* " + m.str() + " */";
            } else {
                std::cerr << ANSIColors::red << "Error: Matrix `" << name
                          << "` was not found" << ANSIColors::reset
                          << std::endl;
                return "<SUBSTITUTION ERROR>";
            }
        }  //
    );

    std::ofstream ofile;
    ofile.open(outfilename);
    if (!ofile)
        throw std::runtime_error("Error opening file `" + outfilename + "`");

    ofile.seekp(std::ios::beg);
    ofile << res;
    ofile.close();
}