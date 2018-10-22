#pragma once

#include "Array.hpp"
#include <iomanip>
#include <ostream>

template <class T, size_t R, size_t C>
using Matrix = Array<Array<T, C>, R>;

// Diagonal matrix
template <class T, size_t N>
Matrix<T, N, N> diag(const Array<T, N> &diagElements) {
    Matrix<T, N, N> matrix = {};
    for (size_t i = 0; i < N; ++i)
        matrix[i][i] = diagElements[i];
    return matrix;
}

// Matrix multiplication (naive approach, O(n³))
template <class T, size_t R, size_t M, size_t C>
Matrix<T, R, C> operator*(const Matrix<T, R, M> &lhs,
                          const Matrix<T, M, C> &rhs) {
    Matrix<T, R, C> result;
    for (size_t r = 0; r < R; ++r) {
        for (size_t c = 0; c < C; ++c) {
            T mac = {};
            for (size_t m = 0; m < M; ++m)
                mac += lhs[r][m] * rhs[m][c];
            result[r][c] = mac;
        }
    }
    return result;
}

// Scalar multiplication
template <class T, size_t R, size_t C>
Matrix<T, R, C> operator*(T scalar, const Matrix<T, R, C> &matrix) {
    Matrix<T, R, C> result;
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            result[r][c] = scalar * matrix[r][c];
    return result;
}

template <class T, size_t R, size_t C>
Matrix<T, R, C> operator*(const Matrix<T, R, C> &matrix, T scalar) {
    return scalar * matrix;
}

// Matrix addition
template <class T, size_t R, size_t C>
Matrix<T, R, C> operator+(const Matrix<T, R, C> &lhs,
                          const Matrix<T, R, C> &rhs) {
    Matrix<T, R, C> result;
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            result[r][c] = lhs[r][c] + rhs[r][c];
    return result;
}

// Matrix subtraction
template <class T, size_t R, size_t C>
Matrix<T, R, C> operator-(const Matrix<T, R, C> &lhs,
                          const Matrix<T, R, C> &rhs) {
    Matrix<T, R, C> result;
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            result[r][c] = lhs[r][c] - rhs[r][c];
    return result;
}

// Matrix transpose
template <class T, size_t R, size_t C>
Matrix<T, C, R> transpose(const Matrix<T, R, C> &matrix) {
    Matrix<T, C, R> result;
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            result[c][r] = matrix[r][c];
    return result;
}

// Printing
template <class T, size_t R, size_t C>
std::ostream &operator<<(std::ostream &os, const Matrix<T, R, C> &matrix) {
    auto colsep = ' ';
    auto rowsep = "\r\n";
    for (const auto &row : matrix) {
        for (const auto &el : row)
            os << std::setw(10) << el << colsep;
        os << rowsep;
    }
    return os;
}