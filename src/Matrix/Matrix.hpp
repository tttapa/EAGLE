#pragma once

#include "Array.hpp"
#include <algorithm>  // all_of
#include <cmath>      // sqrt, isfinite
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

template <class T, size_t R, size_t C>
T *toArrayPointer(TMatrix<T, R, C> &matrix) {
    return &matrix[0][0];
}

template <class T, size_t R, size_t C>
const T *toArrayPointer(const TMatrix<T, R, C> &matrix) {
    return &matrix[0][0];
}

template <class T, size_t R, size_t C>
T (&toCppArray(TMatrix<T, R, C> &matrix))
[R * C] {
    return *reinterpret_cast<T(*)[R * C]>(&matrix.data[0].data);
}

template <class T, size_t R, size_t C>
const T (&toCppArray(const TMatrix<T, R, C> &matrix))[R * C] {
    return *reinterpret_cast<const T(*)[R * C]>(&matrix.data[0].data);
}

template <class T, size_t R, size_t C, class U>
void copyToCArray(U (&dst)[R * C], const TMatrix<T, R, C> &src) {
    size_t i = 0;
    for (const auto &row : src)
        for (const T &el : row)
            dst[i++] = el;
}

template <class T, size_t R, size_t C, class U>
void copyFromCArray(TMatrix<T, R, C> &dst, U (&src)[R * C]) {
    size_t i = 0;
    for (auto &row : dst)
        for (T &el : row)
            el = src[i++];
}

namespace Matrices {

struct TransposeStruct {
} constexpr T;

}  // namespace Matrices

// Diagonal matrix
template <class T, size_t N>
constexpr TMatrix<T, N, N> Tdiag(const TRowVector<T, N> &diagElements) {
    TMatrix<T, N, N> matrix = {};
    for (size_t i = 0; i < N; ++i)
        matrix[i][i] = diagElements[0][i];
    return matrix;
}

template <size_t N>
constexpr Matrix<N, N> diag(const RowVector<N> &diagElements) {
    return Tdiag<double, N>(diagElements);
}

template <class T, size_t N>
constexpr TMatrix<T, N, N> Tdiag(const TColVector<T, N> &diagElements) {
    TMatrix<T, N, N> matrix = {};
    for (size_t i = 0; i < N; ++i)
        matrix[i][i] = diagElements[i];
    return matrix;
}

template <class T>
constexpr TMatrix<T, 1, 1> Tdiag(const TMatrix<T, 1, 1> &diagElements) {
    return diagElements;
}

template <size_t N>
constexpr Matrix<N, N> diag(const ColVector<N> &diagElements) {
    return Tdiag<double, N>(diagElements);
}

constexpr Matrix<1, 1> diag(const Matrix<1, 1> &diagElements) {
    return diagElements;
}

// Identity matrix
template <class T, size_t N>
constexpr TMatrix<T, N, N> Teye() {
    TMatrix<T, N, N> matrix = {};
    for (size_t i = 0; i < N; ++i)
        matrix[i][i] = T{1};
    return matrix;
}

// Identity matrix
template <size_t N>
constexpr Matrix<N, N> eye() {
    Matrix<N, N> matrix = {};
    for (size_t i = 0; i < N; ++i)
        matrix[i][i] = 1.0;
    return matrix;
}

// All zeros
template <class T, size_t M, size_t N>
constexpr TMatrix<T, M, N> Tzeros() {
    return {};
}

// All zeros
template <size_t M, size_t N>
constexpr Matrix<M, N> zeros() {
    return {};
}

// All ones
template <class T, size_t M, size_t N>
constexpr TMatrix<T, M, N> Tones() {
    TMatrix<T, M, N> matrix = {};
    for (auto &row : matrix)
        for (auto &el : row)
            el = T{1};
    return matrix;
}

// All zeros
template <size_t M, size_t N>
constexpr Matrix<M, N> ones() {
    return Tones<double, M, N>();
}

// Matrix multiplication (naive approach, O(n³))
template <class T, class U, size_t R, size_t M, size_t C>
constexpr TMatrix<T, R, C> operator*(const TMatrix<T, R, M> &lhs,
                                     const TMatrix<U, M, C> &rhs) {
    TMatrix<T, R, C> result = {};
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c) {
            for (size_t m = 0; m < M; ++m)
                result[r][c] += lhs[r][m] * rhs[m][c];
        }
    return result;
}

// Inner product
template <class T, class U, size_t R>
constexpr auto operator*(const TColVector<T, R> &lhs,
                         const TColVector<U, R> &rhs) {
    using sum_t = std::remove_reference_t<decltype(lhs[0][0] * rhs[0][0])>;
    sum_t sum   = {};
    for (size_t i = 0; i < R; ++i)
        sum += lhs[i] * rhs[i];
    return sum;
}

// Scalar multiplication
// template <class T, size_t R, size_t C>
// constexpr TMatrix<T, R, C> operator*(T scalar, const TMatrix<T, R, C> &matrix) {
//     TMatrix<T, R, C> result = matrix;
//     for (auto &row : result)
//         for (auto &el : row)
//             el *= scalar;
//     return result;
// }

// template <class T, size_t R, size_t C>
// constexpr TMatrix<T, R, C> operator*(const TMatrix<T, R, C> &matrix, T scalar) {
//     return scalar * matrix;
// }

// Matrix addition
template <class T, class U, size_t R, size_t C>
constexpr TMatrix<T, R, C> operator+(const TMatrix<T, R, C> &lhs,
                                     const TMatrix<U, R, C> &rhs) {
    TMatrix<T, R, C> result = lhs;
    return result += rhs;
}

template <class T, class U, size_t R, size_t C>
constexpr TMatrix<T, R, C> &operator+=(TMatrix<T, R, C> &lhs,
                                       const TMatrix<U, R, C> &rhs) {
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            lhs[r][c] += rhs[r][c];
    return lhs;
}

// Matrix subtraction
template <class T, class U, size_t R, size_t C>
constexpr TMatrix<T, R, C> operator-(const TMatrix<T, R, C> &lhs,
                                     const TMatrix<U, R, C> &rhs) {
    TMatrix<T, R, C> result = lhs;
    result -= rhs;
    return result;
}

template <class T, class U, size_t R, size_t C>
constexpr TMatrix<T, R, C> &operator-=(TMatrix<T, R, C> &lhs,
                                       const TMatrix<U, R, C> &rhs) {
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            lhs[r][c] -= rhs[r][c];
    return lhs;
}

// Matrix transpose
template <class T, size_t R, size_t C>
constexpr TMatrix<T, C, R> transpose(const TMatrix<T, R, C> &matrix) {
    TMatrix<T, C, R> result = {};
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            result[c][r] = matrix[r][c];
    return result;
}

template <class U, size_t R, size_t C>
constexpr TMatrix<U, C, R> operator^(const TMatrix<U, R, C> &matrix,
                                     Matrices::TransposeStruct t) {
    (void) t;
    return transpose(matrix);
}

template <class T, size_t R, size_t C>
constexpr TMatrix<T, R, C> operator-(const TMatrix<T, R, C> &matrix) {
    TMatrix<T, R, C> result = matrix;
    for (auto &row : result)
        for (auto &el : row)
            el = -el;
    return result;
}

// Norm
template <class T, size_t R>
constexpr double normsq(const TRowVector<T, R> &rowvector) {
    double sumsq = 0;
    for (size_t r = 0; r < R; ++r)
        sumsq += rowvector[r] * rowvector[r];
    return sumsq;
}

template <class T, size_t R>
constexpr double norm(const TRowVector<T, R> &rowvector) {
    return sqrt(normsq(rowvector));
}

template <class T, size_t C>
constexpr double normsq(const TColVector<T, C> &colvector) {
    double sumsq = 0;
    for (size_t c = 0; c < C; ++c)
        sumsq += colvector[0][c] * colvector[0][c];
    return sumsq;
}

template <class T, size_t C>
constexpr double norm(const TColVector<T, C> &colvector) {
    return sqrt(normsq(colvector));
}

template <class T, size_t N>
constexpr double normsq(const Array<T, N> &vector) {
    double sumsq = 0;
    for (size_t i = 0; i < N; ++i)
        sumsq += vector[i] * vector[i];
    return sumsq;
}

template <class T, size_t N>
constexpr double norm(const Array<T, N> &vector) {
    return sqrt(normsq(vector));
}

namespace MatrixPrinting {
constexpr size_t precision = 17;
constexpr size_t width     = precision + 6;
}  // namespace MatrixPrinting

// Printing
template <class T, size_t R, size_t C>
std::ostream &operator<<(std::ostream &os, const TMatrix<T, R, C> &matrix) {
    auto colsep = ' ';
    auto rowsep = "\r\n";
    // os << '(' << R << " × " << C << ')' << rowsep;
    os << std::setprecision(MatrixPrinting::precision);
    for (const auto &row : matrix) {
        for (const auto &el : row)
            os << std::setw(MatrixPrinting::width) << el << colsep;
        os << rowsep;
    }
    return os;
}

template <class T, size_t R, size_t C>
void printMATLAB(std::ostream &os, const TMatrix<T, R, C> &matrix) {
    auto colsep = ", ";
    auto rowsep = ";\r\n";
    os << "[ " << std::setprecision(MatrixPrinting::precision);
    for (size_t r = 0; r < R; ++r) {
        if (r)
            os << rowsep << "  ";
        for (size_t c = 0; c < C; ++c) {
            if (c)
                os << colsep;
            os << std::setw(MatrixPrinting::width) << matrix[r][c];
        }
    }
    os << " ]";
}

template <class T, size_t R, size_t C>
void printMATLAB(std::ostream &os, const TMatrix<T, R, C> &matrix,
                 const std::string &name) {
    os << name << " = ...\r\n";
    printMATLAB(os, matrix);
}

template <class T, size_t R, size_t C>
void printC(std::ostream &os, const TMatrix<T, R, C> &matrix) {
    auto colsep = ", ";
    auto rowsep = "},\r\n    {";
    os << "{\r\n    {" << std::setprecision(MatrixPrinting::precision);
    for (size_t r = 0; r < R; ++r) {
        if (r)
            os << rowsep;
        for (size_t c = 0; c < C; ++c) {
            if (c)
                os << colsep;
            os << std::setw(MatrixPrinting::width) << matrix[r][c];
        }
    }
    os << "},\r\n};" << std::endl;
}

template <class T, size_t R, size_t C>
void printC(std::ostream &os, const TMatrix<T, R, C> &matrix,
            const std::string &name) {
    os << "const double[" << R << "][" << C << "] " << name << " = ";
    printC(os, matrix);
}

template <class T, size_t R, size_t C>
void printCpp(std::ostream &os, const TMatrix<T, R, C> &matrix) {
    auto colsep = ", ";
    auto rowsep = "},\r\n    {";
    os << "{{\r\n    {" << std::setprecision(MatrixPrinting::precision);
    for (size_t r = 0; r < R; ++r) {
        if (r)
            os << rowsep;
        for (size_t c = 0; c < C; ++c) {
            if (c)
                os << colsep;
            os << std::setw(MatrixPrinting::width) << matrix[r][c];
        }
    }
    os << "},\r\n}};" << std::endl;
}

template <class T, size_t R, size_t C>
void printCpp(std::ostream &os, const TMatrix<T, R, C> &matrix,
              const std::string &name) {
    os << "const Matrix<" << R << ", " << C << "> " << name << " = ";
    printCpp(os, matrix);
}

struct Printable {
    virtual void print(std::ostream &os) const = 0;
};

inline std::ostream &operator<<(std::ostream &os, const Printable &p) {
    p.print(os);
    return os;
}

template <size_t R, size_t C>
class PrintAsTeX_t : public Printable {
  public:
    PrintAsTeX_t(const Matrix<R, C> &m, size_t precision)
        : m{m}, precision{precision} {}

    void print(std::ostream &os) const override {
        auto colsep = " & ";
        auto rowsep = "\\\\ \r\n";
        os << "\\begin{pmatrix}\r\n  " << std::setprecision(precision);
        for (size_t r = 0; r < R; ++r) {
            if (r)
                os << rowsep << "  ";
            for (size_t c = 0; c < C; ++c) {
                if (c)
                    os << colsep;
                os << std::setw(precision + 6) << m[r][c];
            }
        }
        os << "\r\n\\end{pmatrix}";
    }

  private:
    const Matrix<R, C> m;
    const size_t precision;
};

template <size_t R, size_t C>
PrintAsTeX_t<R, C> asTeX(const Matrix<R, C> &m, size_t precision = 2) {
    return {m, precision};
}

template <size_t N>
class PrintAsRowVector_t : public Printable {
  public:
    PrintAsRowVector_t(const ColVector<N> &v, const char *sep, size_t precision)
        : v{v}, sep{sep}, precision{precision} {}

    void print(std::ostream &os) const override {
        os << std::setprecision(precision) << std::setw(precision + 6)
           << double(v[0]);
        for (size_t i = 1; i < N; ++i)
            os << sep << std::setw(precision + 6) << double(v[i]);
    }

  private:
    const ColVector<N> v;
    const char *const sep;
    const size_t precision;
};

template <size_t N>
PrintAsRowVector_t<N> asrowvector(const ColVector<N> &v, const char *sep = " ",
                                  size_t precision = 2) {
    return {v, sep, precision};
}

template <size_t N>
PrintAsRowVector_t<N> asrowvector(const RowVector<N> &v, const char *sep = " ",
                                  size_t precision = 2) {
    return {transpose(v), sep, precision};
}

// -----------------------------------------------------------------------------

/**
 * @brief   Calculates u × v, where u, v ⊆ ℝ³.
 */
template <class T>
constexpr TColVector<T, 3> cross(const TColVector<T, 3> &u,
                                 const TColVector<T, 3> &v) {
    return {{
        {u[1] * v[2] - u[2] * v[1]},
        {u[2] * v[0] - u[0] * v[2]},
        {u[0] * v[1] - u[1] * v[0]},
    }};
}

// -----------------------------------------------------------------------------

template <class T, size_t R, size_t C, size_t RR_sz, size_t CC_sz,
          size_t RR_offset, size_t CC_offset>
struct MatrixAssignmentHelper {
    TMatrix<T, R, C> &m;
    constexpr MatrixAssignmentHelper<T, R, C, RR_sz, CC_sz, RR_offset,
                                     CC_offset> &
    operator=(const TMatrix<T, RR_sz, CC_sz> &rhs) {
        for (size_t r = 0; r < RR_sz; ++r)
            for (size_t c = 0; c < CC_sz; ++c)
                m[RR_offset + r][CC_offset + c] = rhs[r][c];
        return *this;
    }
};

template <size_t Rstart, size_t Rend, size_t Cstart, size_t Cend, class T,
          size_t R, size_t C>
constexpr inline MatrixAssignmentHelper<T, R, C, Rend - Rstart, Cend - Cstart,
                                        Rstart, Cstart>
assignBlock(TMatrix<T, R, C> &matrix) {
    static_assert(Rstart < R && Rend <= R, "Error: Row indices out of bounds");
    static_assert(Cstart < C && Cend <= C,
                  "Error: Column indices out of bounds");
    return {matrix};
}

template <size_t Rstart, size_t Rend, size_t Cstart, size_t Cend, class T,
          size_t R, size_t C>
constexpr inline TMatrix<T, Rend - Rstart, Cend - Cstart>
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
constexpr TMatrix<T, R, C1 + C2> hcat(const TMatrix<T, R, C1> &l,
                                      const TMatrix<T, R, C2> &r) {
    TMatrix<T, R, C1 + C2> result          = {};
    assignBlock<0, R, 0, C1>(result)       = l;
    assignBlock<0, R, C1, C1 + C2>(result) = r;
    return result;
}

template <class T, size_t R, size_t C, class... Args>
constexpr auto hcat(const TMatrix<T, R, C> &l, Args... args)
    -> decltype(hcat(l, hcat(args...))) {
    return hcat(l, hcat(args...));
}

template <class T, size_t R1, size_t R2, size_t C>
constexpr TMatrix<T, R1 + R2, C> vcat(const TMatrix<T, R1, C> &t,
                                      const TMatrix<T, R2, C> &b) {
    TMatrix<T, R1 + R2, C> result          = {};
    assignBlock<0, R1, 0, C>(result)       = t;
    assignBlock<R1, R1 + R2, 0, C>(result) = b;
    return result;
}

template <class T, size_t R, size_t C, class... Args>
constexpr auto vcat(const TMatrix<T, R, C> &t, Args... args)
    -> decltype(vcat(t, vcat(args...))) {
    return vcat(t, vcat(args...));
}

// Rounding
template <size_t R, size_t C>
Matrix<R, C> round(const Matrix<R, C> &m, size_t digits) {
    auto result = m;
    for (auto &row : result)
        for (auto &el : row)
            el = round(el * pow(10, digits)) / pow(10, digits);
    return result;
}

// Clamping
template <class T>
static T clamp(T v, T lo, T hi) {
    return (v < lo) ? lo : ((hi < v) ? hi : v);
}
template <size_t R, size_t C>
void clamp(Matrix<R, C> &m, const Matrix<R, C> &min, const Matrix<R, C> &max) {
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            m[r][c] = clamp(m[r][c], min[r][c], max[r][c]);
}