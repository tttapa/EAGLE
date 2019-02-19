#include "FileLoader.hpp"

double loadDouble(const std::string &name) {
    auto filename = name + ".double";
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    if (!in) {
        std::stringstream sstr;
        sstr << "Error: unable to open file: " << filename;
        throw std::runtime_error(sstr.str());
    }
    auto size = in.tellg();
    if (size != sizeof(double))
        throw std::runtime_error("Error: file size doesn't match expected "
                                 "size. Is double size correct?");

    double result;
    in.seekg(0, in.beg);
    in.read(reinterpret_cast<char *>(&result), sizeof(double));
    return result;
}
