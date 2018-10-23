#pragma once

#include "HouseholderQR.hpp"

template <class T, size_t M, size_t N, size_t P>

Matrix<T, N, P> solveLeastSquares(const Matrix<T, M, N> &a,
                                  const Matrix<T, M, P> &b) {
    static_assert(M >= N, "Error: A matrix should be a square or rectangular "
                          "matrix with more rows than columns");

    QR<T, M, N> qr = householderQR(a);
    auto &r = qr.R;
    auto qtb = qr.applyTranspose(b);
    Matrix<T, N, P> x = {};
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