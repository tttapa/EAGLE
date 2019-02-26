#pragma once

#include <pybind11/numpy.h>

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
    bool load(handle src, bool convert) {

        if (!convert && !array_t<T>::check_(src))
            return false;

        // Ensure that the source object is a NumPy array of the correct type
        // (and if not, try to convert it)
        constexpr auto flags = array::c_style | array::forcecast;
        auto buf             = array_t<T, flags>::ensure(src);
        if (!buf)
            return false;

        auto dims = buf.ndim();
        if (dims != 1 && dims != 2) {
            std::cerr << "One- or two-dimensional array expected, not " << dims
                      << "\r\n";
            return false;
        }
        if ((R > 1 && C > 1) || dims == 2) {
            // Check that it is a two-dimensional NumPy array
            if (dims != 2) {
                std::cerr << "Two-dimensional array expected, not " << dims
                          << "\r\n";
                return false;
            }

            // Check that the number of rows and columns match
            std::array<ssize_t, 2> shape   = {{buf.shape()[0], buf.shape()[1]}};
            std::array<ssize_t, 2> toShape = {{R, C}};
            if (shape != toShape) {
                std::cerr << "Shapes do not match: Python: (" << shape[0]
                          << ", " << shape[1] << "), C++: (" << toShape[0]
                          << ", " << toShape[1] << ")\r\n";
                return false;
            }
        } else if (R == 1) {  // && dims == 1 → Row vector
            if (C != buf.shape()[0]) {
                std::cerr << "Dimensions do not match: Python ("
                          << buf.shape()[0] << "), C++: (" << C << ")\r\n";
                return false;
            }
        } else if (C == 1) {  // && dims == 1 → Column vector
            if (R != buf.shape()[0]) {
                std::cerr << "Dimensions do not match: Python ("
                          << buf.shape()[0] << "), C++: (" << R << ")\r\n";
                return false;
            }
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
};  // namespace detail

}  // namespace detail
}  // namespace pybind11