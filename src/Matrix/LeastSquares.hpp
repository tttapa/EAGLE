#pragma once

#include "HouseholderQR.hpp"

template <class T, size_t M, size_t N, size_t P>

TMatrix<T, N, P> solveLeastSquares(const TMatrix<T, M, N> &a,
                                  const TMatrix<T, M, P> &b) {
    static_assert(M >= N, "Error: A matrix should be a square or rectangular "
                          "matrix with more rows than columns");

    QR<T, M, N> qr = householderQR(a);
    auto &r = qr.R;
    auto qtb = qr.applyTranspose(b);
    TMatrix<T, N, P> x = {};
    // Back substitution
    for (size_t p = 0; p < P; ++p) {
        for (size_t n = N; n-- > 0;) {
            auto sum = qtb[n][p];
            for (size_t nn = n + 1; nn < N; ++nn)
                sum -= r[n][nn] * x[nn][p];
            x[n][p] = sum / r[n][n];
        }
    }
    return x;
}

template <size_t N>
Matrix<N, N> inv(const Matrix<N, N> &matrix) {
    return solveLeastSquares(matrix, eye<N>());
}