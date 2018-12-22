#pragma once

#include "HouseholderQR.hpp"
#include "LeastSquares.hpp"
#include "Matrix.hpp"
#include <lapacke.h>

template <size_t N>
struct Balance_result_GEP {
    Matrix<N, N> balancing_matrix;
    Matrix<N, N> balancing_matrix2;
    Matrix<N, N> balanced_matrix;
    Matrix<N, N> balanced_matrix2;
};

template <size_t N>
Balance_result_GEP<N> balance(const Matrix<N, N> &A, const Matrix<N, N> &B) {
    Balance_result_GEP<N> result = {eye<N>(), eye<N>(), A, B};

    double *p_balanced_mat  = toArrayPointer(result.balanced_matrix);
    double *p_balanced_mat2 = toArrayPointer(result.balanced_matrix2);

    char job = 'B';

    lapack_int n = N;
    lapack_int info;
    lapack_int ilo;
    lapack_int ihi;
    double plscale[N];
    double prscale[N];
    double pwork[N * 6];

    info = LAPACKE_dggbal_work(LAPACK_ROW_MAJOR, job, n, p_balanced_mat, n,
                               p_balanced_mat2, n, &ilo, &ihi, plscale, prscale,
                               pwork);
    if (info != 0)
        throw std::runtime_error(std::string("LAPACKE_dggbal_work: info = ") +
                                 std::to_string(info));

    double *p_balancing_mat  = toArrayPointer(result.balancing_matrix);
    double *p_balancing_mat2 = toArrayPointer(result.balancing_matrix2);

    info = LAPACKE_dggbak(LAPACK_ROW_MAJOR, job, 'L', n, ilo, ihi, plscale,
                          prscale, n, p_balancing_mat, n);
    if (info != 0)
        throw std::runtime_error(std::string("LAPACKE_dggbak: info = ") + std::to_string(info));

    info = LAPACKE_dggbak(LAPACK_ROW_MAJOR, job, 'R', n, ilo, ihi, plscale,
                          prscale, n, p_balancing_mat2, n);
    if (info != 0)
        throw std::runtime_error(std::string("LAPACKE_dggbak: info = ") + std::to_string(info));
    return result;
}

template <size_t Nx>
Matrix<Nx, Nx> qz_Z(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nx> &B) {
    lapack_int nn = Nx;

    Matrix<Nx, Nx> aa = A;
    double *paa       = toArrayPointer(aa);

    Matrix<Nx, Nx> bb = B;
    double *pbb       = toArrayPointer(bb);

    Matrix<Nx, Nx> QQ = eye<Nx>();
    double *pqq       = toArrayPointer(QQ);
    Matrix<Nx, Nx> ZZ = eye<Nx>();
    double *pzz       = toArrayPointer(ZZ);
    double alphar[Nx];
    double alphai[Nx];
    double betar[Nx];
    lapack_int ilo, ihi, info;
    char comp_q = 'V';
    char comp_z = 'V';

    // Permutation balancing
    const char bal_job = 'P';
    double lscale[Nx];
    double rscale[Nx];
    double work[6 * Nx];

    info = LAPACKE_dggbal_work(LAPACK_ROW_MAJOR, bal_job, nn, paa, nn, pbb, nn,
                               &ilo, &ihi, lscale, rscale, work);
    if (info != 0)
        throw std::runtime_error(std::string("LAPACKE_dggbal_work: info = ") +
                                 std::to_string(info));

    // QR factorization of bb
    QR<double, Nx, Nx> qrRes = householderQR(B);
    bb                       = qrRes.R;
    aa                       = qrRes.applyTranspose(aa);
    QQ                       = QQ * qrRes.Q();

    info = LAPACKE_dgghrd(LAPACK_ROW_MAJOR, comp_q, comp_z, nn, ilo, ihi, paa,
                          nn, pbb, nn, pqq, nn, pzz, nn);
    if (info != 0)
        throw std::runtime_error(std::string("LAPACKE_dgghrd: info = ") + std::to_string(info));

    // Hessenberg
    info = LAPACKE_dgghrd(LAPACK_ROW_MAJOR, comp_q, comp_z, nn, ilo, ihi, paa,
                          nn, pbb, nn, pqq, nn, pzz, nn);
    if (info != 0)
        throw std::runtime_error(std::string("LAPACKE_dgghrd: info = ") + std::to_string(info));

    constexpr char qz_job = 'S';
    // Schur
    info = LAPACKE_dhgeqz_work(LAPACK_ROW_MAJOR, qz_job, comp_q, comp_z, nn,
                               ilo, ihi, paa, nn, pbb, nn, alphar, alphai,
                               betar, pqq, nn, pzz, nn, work, nn);
    if (info != 0)
        throw std::runtime_error(std::string("LAPACKE_dhgeqz_work: info = ") +
                                 std::to_string(info));

    info = LAPACKE_dggbak(LAPACK_ROW_MAJOR, bal_job, 'L', nn, ilo, ihi, lscale,
                          rscale, nn, pqq, nn);
    if (info != 0)
        throw std::runtime_error(std::string("LAPACKE_dggbak: info = ") + std::to_string(info));

    info = LAPACKE_dggbak(LAPACK_ROW_MAJOR, bal_job, 'R', nn, ilo, ihi, lscale,
                          rscale, nn, pzz, nn);
    if (info != 0)
        throw std::runtime_error(std::string("LAPACKE_dggbak: info = ") + std::to_string(info));

    lapack_logical select[Nx];

    for (size_t j = 0; j < Nx; ++j)
        select[j] =
            alphar[j] * alphar[j] + alphai[j] * alphai[j] < betar[j] * betar[j];

    lapack_logical wantq = 0, wantz = 1;
    lapack_int ijob = 0;
    lapack_int mm;
    constexpr lapack_int lrwork3 = 4 * Nx + 16;
    constexpr lapack_int liwork  = Nx;
    double pl, pr;
    double rwork3[lrwork3];
    lapack_int iwork[liwork];

    info = LAPACKE_dtgsen_work(LAPACK_ROW_MAJOR, ijob, wantq, wantz, select, nn,
                               paa, nn, pbb, nn, alphar, alphai, betar, nullptr,
                               nn, pzz, nn, &mm, &pl, &pr, nullptr, rwork3,
                               lrwork3, iwork, liwork);
    if (info != 0)
        throw std::runtime_error(std::string("LAPACKE_dtgsen_work: info = ") +
                                 std::to_string(info));

    return ZZ;
}

template <size_t Nx, size_t Nu>
Matrix<Nx, Nx> dare(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                    const Matrix<Nx, Nx> &C, const Matrix<Nu, Nu> &R) {
    using Matrices::T;

    auto S1 = vcat(                //
        hcat(A, zeros<Nx, Nx>()),  //
        hcat(-C, eye<Nx>())        //
    );

    auto S2 = vcat(                             //
        hcat(eye<Nx>(), B * inv(R) * (B ^ T)),  //
        hcat(zeros<Nx, Nx>(), A ^ T)            //
    );

    Balance_result_GEP<2 *Nx> balRes = balance(S1, S2);
    auto U  = qz_Z(balRes.balanced_matrix, balRes.balanced_matrix2);
    auto D  = balRes.balancing_matrix2;
    U       = D * U;
    auto U1 = getBlock<Nx, 2 * Nx, 0, Nx>(U);
    auto U2 = getBlock<0, Nx, 0, Nx>(U);
    auto X  = U1 * inv(U2);
    return X;
}

template <size_t Nx, size_t Nu>
struct DLQR_result {
    Matrix<Nx, Nx> P;
    Matrix<Nu, Nx> K;
};

template <size_t Nx, size_t Nu>
DLQR_result<Nx, Nu> dlqr(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                         const Matrix<Nx, Nx> &Q, const Matrix<Nu, Nu> &R) {
    using Matrices::T;

    Matrix<Nx, Nu> S = zeros<Nx, Nu>();
    auto P           = dare(A, B, Q, R);
    auto K           = solveLeastSquares(    //
        R + (B ^ T) * P * B,       //
        (B ^ T) * P * A + (S ^ T)  //
    );
    return {P, K};
}