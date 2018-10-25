#pragma once

#include "Array.hpp"
#include <cmath>  // sqrt
#include <iomanip>
#include <ostream>

template <class T, size_t R, size_t C>
using TMatrix = Array<Array<T, C>, R>;

template <size_t R, size_t C>
using Matrix = TMatrix<double, R, C>;

template <class T, size_t R>
using TColVector = TMatrix<T, R, 1>;

template <size_t R>
using ColVector = TColVector<double, R>;

template <class T, size_t C>
using TRowVector = TMatrix<T, 1, C>;

template <size_t C>
using RowVector = TRowVector<double, C>;

namespace Matrices {

struct TransposeStruct {
} extern T;

}  // namespace Matrices

// Diagonal matrix
template <class T, size_t N>
TMatrix<T, N, N> diag(const Array<T, N> &diagElements) {
    TMatrix<T, N, N> matrix = {};
    for (size_t i = 0; i < N; ++i)
        matrix[i][i] = diagElements[i];
    return matrix;
}

// Identity matrix
template <class T, size_t N>
TMatrix<T, N, N> eye(T unit = 1) {
    TMatrix<T, N, N> matrix = {};
    for (size_t i = 0; i < N; ++i)
        matrix[i][i] = unit;
    return matrix;
}

// All zeros
template <class T, size_t M, size_t N>
TMatrix<T, M, N> zeros() {
    return {};
}

// Matrix multiplication (naive approach, O(n³))
template <class T, class U, size_t R, size_t M, size_t C>
TMatrix<T, R, C> operator*(const TMatrix<T, R, M> &lhs,
                           const TMatrix<U, M, C> &rhs) {
    TMatrix<T, R, C> result;
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c) {
            T mac = {};
            for (size_t m = 0; m < M; ++m)
                mac += lhs[r][m] * rhs[m][c];
            result[r][c] = mac;
        }
    return result;
}

// Scalar multiplication
template <class T, size_t R, size_t C>
TMatrix<T, R, C> operator*(T scalar, const TMatrix<T, R, C> &matrix) {
    TMatrix<T, R, C> result;
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            result[r][c] = scalar * matrix[r][c];
    return result;
}

template <class T, size_t R, size_t C>
TMatrix<T, R, C> operator*(const TMatrix<T, R, C> &matrix, T scalar) {
    return scalar * matrix;
}

// Matrix addition
template <class T, class U, size_t R, size_t C>
TMatrix<T, R, C> operator+(const TMatrix<T, R, C> &lhs,
                           const TMatrix<U, R, C> &rhs) {
    TMatrix<T, R, C> result = lhs;
    return result += rhs;
}

template <class T, class U, size_t R, size_t C>
TMatrix<T, R, C> &operator+=(TMatrix<T, R, C> &lhs,
                             const TMatrix<U, R, C> &rhs) {
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            lhs[r][c] += rhs[r][c];
    return lhs;
}

// Matrix subtraction
template <class T, class U, size_t R, size_t C>
TMatrix<T, R, C> operator-(const TMatrix<T, R, C> &lhs,
                           const TMatrix<U, R, C> &rhs) {
    TMatrix<T, R, C> result;
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            result[r][c] = lhs[r][c] - rhs[r][c];
    return result;
}

// Matrix transpose
template <class T, size_t R, size_t C>
TMatrix<T, C, R> transpose(const TMatrix<T, R, C> &matrix) {
    TMatrix<T, C, R> result;
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            result[c][r] = matrix[r][c];
    return result;
}

template <class U, size_t R, size_t C>
TMatrix<U, C, R> operator^(const TMatrix<U, R, C> &matrix,
                           Matrices::TransposeStruct t) {
    (void) t;
    return transpose(matrix);
}

// Norm
template <class T, size_t R>
double norm(const TRowVector<T, R> &rowvector) {
    double sumsq = 0;
    for (size_t r = 0; r < R; ++r)
        sumsq += rowvector[r][0] * rowvector[r][0];
    return sqrt(sumsq);
}

template <class T, size_t C>
double norm(const TColVector<T, C> &colvector) {
    double sumsq = 0;
    for (size_t c = 0; c < C; ++c)
        sumsq += colvector[0][c] * colvector[0][c];
    return sqrt(sumsq);
}

template <class T, size_t N>
double norm(const Array<T, N> &vector) {
    double sumsq = 0;
    for (size_t i = 0; i < N; ++i)
        sumsq += vector[i] * vector[i];
    return sqrt(sumsq);
}

// Printing
template <class T, size_t R, size_t C>
std::ostream &operator<<(std::ostream &os, const TMatrix<T, R, C> &matrix) {
    auto colsep = ' ';
    auto rowsep = "\r\n";
    os << '(' << R << " × " << C << ')' << rowsep;
    for (const auto &row : matrix) {
        for (const auto &el : row)
            os << std::setw(10) << el << colsep;
        os << rowsep;
    }
    return os;
}

// -----------------------------------------------------------------------------

/**
 * @brief   Calculates u × v, where u, v ⊆ ℝ³.
 */
template <class T>
TColVector<T, 3> cross(const TColVector<T, 3> &u, const TColVector<T, 3> &v) {
    return {{
        {u[1][0] * v[2][0] - u[2][0] * v[1][0]},
        {u[2][0] * v[0][0] - u[0][0] * v[2][0]},
        {u[0][0] * v[1][0] - u[1][0] * v[0][0]},
    }};
}

// -----------------------------------------------------------------------------

template <class T, size_t R, size_t C, size_t RR_sz, size_t CC_sz,
          size_t RR_offset, size_t CC_offset>
struct MatrixAssignmentHelper {
    TMatrix<T, R, C> &m;
    MatrixAssignmentHelper<T, R, C, RR_sz, CC_sz, RR_offset, CC_offset> &
    operator=(const TMatrix<T, RR_sz, CC_sz> &rhs) {
        for (size_t r = 0; r < RR_sz; ++r)
            for (size_t c = 0; c < CC_sz; ++c)
                m[RR_offset + r][CC_offset + c] = rhs[r][c];
        return *this;
    }
};

template <size_t Rstart, size_t Rend, size_t Cstart, size_t Cend, class T,
          size_t R, size_t C>
inline MatrixAssignmentHelper<T, R, C, Rend - Rstart, Cend - Cstart, Rstart,
                              Cstart>
assignBlock(TMatrix<T, R, C> &matrix) {
    static_assert(Rstart < R && Rend <= R, "Error: Row indices out of bounds");
    static_assert(Cstart < C && Cend <= C,
                  "Error: Column indices out of bounds");
    return {matrix};
}

template <size_t Rstart, size_t Rend, size_t Cstart, size_t Cend, class T,
          size_t R, size_t C>
inline TMatrix<T, Rend - Rstart, Cend - Cstart>
getBlock(const TMatrix<T, R, C> &matrix) {
    static_assert(Rstart < R && Rend <= R, "Error: Row indices out of bounds");
    static_assert(Cstart < C && Cend <= C,
                  "Error: Column indices out of bounds");
    TMatrix<T, Rend - Rstart, Cend - Cstart> result = {};
    for (size_t r = 0; r < Rend - Rstart; ++r)
        for (size_t c = 0; c < Cend - Cstart; ++c)
            result[r][c] = matrix[r + Rstart][c + Cstart];
    return result;
}

template <class T, size_t R, size_t C1, size_t C2>
TMatrix<T, R, C1 + C2> hcat(const TMatrix<T, R, C1> &l,
                            const TMatrix<T, R, C2> &r) {
    TMatrix<T, R, C1 + C2> result;
    assignBlock<0, R, 0, C1>(result)       = l;
    assignBlock<0, R, C1, C1 + C2>(result) = r;
    return result;
}

template <class T, size_t R, size_t C, class... Args>
auto hcat(const TMatrix<T, R, C> &l, Args... args)
    -> decltype(hcat(l, hcat(args...))) {
    return hcat(l, hcat(args...));
}

template <class T, size_t R1, size_t R2, size_t C>
TMatrix<T, R1 + R2, C> vcat(const TMatrix<T, R1, C> &t,
                            const TMatrix<T, R2, C> &b) {
    TMatrix<T, R1 + R2, C> result;
    assignBlock<0, R1, 0, C>(result)       = t;
    assignBlock<R1, R1 + R2, 0, C>(result) = b;
    return result;
}

template <class T, size_t R, size_t C, class... Args>
auto vcat(const TMatrix<T, R, C> &t, Args... args)
    -> decltype(vcat(t, vcat(args...))) {
    return vcat(t, vcat(args...));
}