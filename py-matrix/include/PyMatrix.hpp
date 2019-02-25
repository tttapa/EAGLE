#pragma once

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <iostream>

#include <Matrix.hpp>

// type caster: Matrix <-> NumPy-array
namespace pybind11 {
namespace detail {
template <typename T, size_t R, size_t C>
struct type_caster<TMatrix<T, R, C>> {

  public:
    using MT = TMatrix<T, R, C>;
    PYBIND11_TYPE_CASTER(MT, _("TMatrix<T, R, C>"));

    // Conversion part 1 (Python -> C++)
    bool load(pybind11::handle src, bool convert) {
        namespace py = pybind11;

        if (!convert && !py::array_t<T>::check_(src))
            return false;

        auto buf =
            py::array_t<T, py::array::c_style | py::array::forcecast>::ensure(
                src);
        if (!buf)
            return false;

        auto dims = buf.ndim();
        if (dims != 2)
            return false;

        Array<ssize_t, 2> shape   = {{buf.shape()[0], buf.shape()[1]}};
        Array<ssize_t, 2> toShape = {{R, C}};
        if (shape != toShape) {
            std::cerr << "Shapes do not match: Python: (" << shape[0] << ", "
                      << shape[1] << "), C++: (" << toShape[0] << ", "
                      << toShape[1] << ")\r\n";
            return false;
        }

        std::copy(buf.data(), buf.data() + R * C, toArrayPointer(value));

        return true;
    }

    //Conversion part 2 (C++ -> Python)
    static pybind11::handle cast(const TMatrix<T, R, C> &src,
                                 pybind11::return_value_policy policy,
                                 pybind11::handle parent) {
        namespace py = pybind11;

        (void) policy;
        (void) parent;

        std::vector<size_t> dim     = {R, C};
        std::vector<size_t> strides = {sizeof(T) * C, sizeof(T)};
        py::array a                 = {dim, strides, toArrayPointer(src)};

        return a.release();
    }
};
}  // namespace detail
}  // namespace pybind11