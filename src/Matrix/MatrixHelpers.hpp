#pragma once

#include "ArrayHelpers.hpp"
#include "Matrix.hpp"

template <class T, size_t R, size_t C>
constexpr TMatrix<T, R, C> filledTMatrix(const T &value) {
    TMatrix<T, R, C> result = {};
    std::fill(result.begin(), result.end(), filledArray<T, C>(value));
    return result;
}

template <size_t R, size_t C>
constexpr Matrix<R, C> filledMatrix(const double value) {
    Matrix<R, C> result = {};
    std::fill(result.begin(), result.end(), filledArray<double, C>(value));
    return result;
}