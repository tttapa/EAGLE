#pragma once

#include "Matrix.hpp"

#include <type_traits>

template <class T, size_t Rm, size_t Cn>
struct QR {
    TMatrix<T, Rm, Cn> U;
    TMatrix<T, Rm, Cn> R;

    template <size_t C>
    constexpr TMatrix<T, Rm, C> applyTranspose(const TMatrix<T, Rm, C> &b) {
        TMatrix<T, Rm, C> result = b;
        typedef typename std::remove_reference<decltype(result[0])>::type row_t;
        for (size_t c = 0; c < Cn; ++c) {
            row_t uTb = {};
            for (size_t cc = 0; cc < C; ++cc)
                for (size_t r = c; r < Rm; ++r)
                    uTb[cc] += U[r][c] * result[r][cc];
            for (size_t r = c; r < Rm; ++r)
                for (size_t cc = 0; cc < C; ++cc)
                    result[r][cc] += -U[r][c] * uTb[cc];
        }
        return result;
    }
};

template <typename T>
constexpr double sign(T x) {
    return double(x >= T{0}) - double(x < T{0});
}

template <class T, size_t Rm, size_t Cn>
constexpr QR<T, Rm, Cn> householderQR(const TMatrix<T, Rm, Cn> &a) {
    QR<T, Rm, Cn> qtr = {{}, {a}};

    for (size_t c = 0; c < Cn; ++c) {
        // For each column of A
        double norm_sq = 0;
        // Calculate the norm from column (from diagonal down)
        for (size_t r = c; r < Rm; ++r)
            norm_sq += qtr.R[r][c] * qtr.R[r][c];
        double norm = sqrt(norm_sq);
        double dcc  = sign(qtr.R[c][c]) * norm;  // TODO
        if (norm != 0) {
            // Normalize
            for (size_t r = c; r < Rm; ++r)
                qtr.R[r][c] /= norm;
            // Update first element
            qtr.R[c][c]         = qtr.R[c][c] + sign(qtr.R[c][c]);
            double newnormsqrt2 = sqrt(fabs(qtr.R[c][c]));
            // Normalize again to a factor of sqrt2
            for (size_t r = c; r < Rm; ++r) {
                qtr.R[r][c] /= newnormsqrt2;
                qtr.U[r][c] = qtr.R[r][c];
            }
        } else {
            qtr.R[c][c] = sqrt(2);
            qtr.U[c][c] = qtr.R[c][c];
        }

        for (size_t i = c + 1; i < Cn; ++i) {
            double sum = 0;
            for (size_t k = c; k < Rm; ++k)
                sum += qtr.R[k][c] * qtr.R[k][i];
            for (size_t k = c; k < Rm; ++k)
                qtr.R[k][i] -= sum * qtr.R[k][c];
        }
        for (size_t r = c + 1; r < Rm; ++r)
            qtr.R[r][c] = 0;
        qtr.R[c][c] = -dcc;
    }
    return qtr;
}