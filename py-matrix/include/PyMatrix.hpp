#pragma once

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <iostream>  // cerr

#include <Matrix.hpp>

// type caster: TMatrix<T, R, C> <-> NumPy-array
namespace pybind11 {
namespace detail {

template <typename T, size_t R, size_t C>
struct type_caster<TMatrix<T, R, C>> {
  public:
    using MT = TMatrix<T, R, C>;

    PYBIND11_TYPE_CASTER(MT, _("TMatrix<T, R, C>"));

    /// Conversion Python → C++
    bool load(pybind11::handle src, bool convert) {
        namespace py = pybind11;

        if (!convert && !py::array_t<T>::check_(src))
            return false;

        // Ensure that the source object is a NumPy array of the correct type
        // (and if not, try to convert it)
        constexpr auto flags = py::array::c_style | py::array::forcecast;
        auto buf             = py::array_t<T, flags>::ensure(src);
        if (!buf)
            return false;

        // Check that it is a two-dimensional NumPy array
        auto dims = buf.ndim();
        if (dims != 2)
            return false;

        // Check that the number of rows and columns match
        Array<ssize_t, 2> shape   = {{buf.shape()[0], buf.shape()[1]}};
        Array<ssize_t, 2> toShape = {{R, C}};
        if (shape != toShape) {
            std::cerr << "Shapes do not match: Python: (" << shape[0] << ", "
                      << shape[1] << "), C++: (" << toShape[0] << ", "
                      << toShape[1] << ")\r\n";
            return false;
        }

        // Copy the buffer to the TMatrix
        std::copy(buf.data(), buf.data() + R * C, toArrayPointer(value));
        return true;
    }

    /// Conversion C++ → Python
    static pybind11::handle cast(const TMatrix<T, R, C> &src,
                                 pybind11::return_value_policy policy,
                                 pybind11::handle parent) {
        (void) policy;
        (void) parent;

        std::array<size_t, 2> shape   = {R, C};
        std::array<size_t, 2> strides = {sizeof(T) * C, sizeof(T)};
        pybind11::array a             = {shape, strides, toArrayPointer(src)};

        // Release the Python object from `a`, otherwise, the reference count
        // will be decreased when `a` goes out of scope, and the Python GC 
        // will destroy the new object
        return a.release();
    }
};

}  // namespace detail
}  // namespace pybind11