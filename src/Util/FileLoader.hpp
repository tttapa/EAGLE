#pragma once

#include <Matrix/Matrix.hpp>
#include <filesystem>
#include <fstream>
#include <string>

template <size_t R, size_t C>
Matrix<R, C> loadMatrix(const std::string &name) {
    auto filename = name + ".matrix";
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    if (!in) {
        std::stringstream sstr;
        sstr << "Error: unable to open file: " << filename;
        throw std::runtime_error(sstr.str());
    }
    auto size = in.tellg();
    if (size < 2) {
        std::stringstream sstr;
        sstr << "Error: file size too small "
                "("
             << filename << ")";
        throw std::runtime_error(sstr.str());
    }
    uint8_t r, c;
    in.seekg(0, in.beg);
    in.read(reinterpret_cast<char *>(&r), 1);
    in.seekg(1, in.beg);
    in.read(reinterpret_cast<char *>(&c), 1);
    if (r != R) {
        std::stringstream sstr;
        sstr << "Error: number of rows in file doesn't match expected "
                "number of rows. "
                "("
             << filename
             << ") "
                "Expected: "
             << R << ", File: " << +r;
        throw std::runtime_error(sstr.str());
    }
    if (c != C) {
        std::stringstream sstr;
        sstr << "Error: number of columns in file doesn't match expected "
                "number of columns. "
                "("
             << filename
             << ") "
                "Expected: "
             << C << ", File: " << +c;
        throw std::runtime_error(sstr.str());
    }
    if (size != R * C * sizeof(double) + 2)
        throw std::runtime_error("Error: file size doesn't match expected "
                                 "size. Is double size correct?");

    Matrix<R, C> result;
    in.seekg(2, in.beg);
    in.read(reinterpret_cast<char *>(toArrayPointer(result)),
            R * C * sizeof(double));
    return result;
}

double loadDouble(const std::string &name);