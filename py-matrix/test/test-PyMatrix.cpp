#include <gtest/gtest.h>
#include <pybind11/embed.h>

#include <PyMatrix.hpp>

TEST(PyMatrix, simple) {
    Matrix<3, 3> m                      = {{
        {11, 12, 13},
        {21, 22, 23},
        {31, 32, 33},
    }};
    pybind11::scoped_interpreter intrpr = {};

    using namespace pybind11::literals;

    pybind11::exec("import numpy\nprint(m)", pybind11::globals(),
                   pybind11::dict{"m"_a = m});

}