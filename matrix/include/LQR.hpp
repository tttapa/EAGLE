#pragma once

#include <LeastSquares.hpp>
#include <lapacke.h>
#include <string>

// Lapacke includes complex, so it defines I, which causes problems with other
// dependencies
#ifdef I
#undef I
#endif

template <size_t N>
struct Schur_result {
    Matrix<N, N> S;
    Matrix<N, N> U;
};

static lapack_logical select_ana(const double *a, const double *) {
    return (*a < 0.0);
}

static lapack_logical select_dig(const double *a, const double *b) {
    return (hypot(*a, *b) < 1.0);
}

template <size_t N>
Schur_result<N> schur(const Matrix<N, N> &A, const std::string &ord,
                      bool calc_unitary = true) {
    Matrix<N, N> S;
    Matrix<N, N> U;

    constexpr lapack_int n = N;

    if (n == 0) {
        S = {};
        U = {};
        return {S, U};
    }

    char jobvs;
    char sense = 'N';
    char sort  = 'N';

    if (calc_unitary)
        jobvs = 'V';
    else
        jobvs = 'N';

    char ord_char = (ord.empty() ? 'U' : ord[0]);

    if (ord_char == 'A' || ord_char == 'D' || ord_char == 'a' ||
        ord_char == 'd')
        sort = 'S';

    volatile LAPACK_D_SELECT2 selector = nullptr;
    if (ord_char == 'A' || ord_char == 'a')
        selector = select_ana;
    else if (ord_char == 'D' || ord_char == 'd')
        selector = select_dig;

    constexpr lapack_int lwork  = 8 * n;
    constexpr lapack_int liwork = 1;
    // lapack_int info;
    lapack_int sdim;
    double rconde;
    double rcondv;

    if (calc_unitary)
        U = {};
    S = A;

    double *s = toArrayPointer(S);
    double *q = toArrayPointer(U);

    double pwr[N];
    double pwi[N];

    double pwork[lwork];

    // BWORK is not referenced for the non-ordered Schur routine.
    /* lapack_int ntmp = (ord_char == 'N' || ord_char == 'n') ? 0 : n;
    Array<lapack_int> bwork(dim_vector(ntmp, 1));
    lapack_int *pbwork = bwork.fortran_vec(); */
    lapack_int pbwork[N];

    lapack_int piwork[liwork];

    lapack_int info = LAPACKE_dgeesx_work(  //
        LAPACK_ROW_MAJOR,                   //
        jobvs, sort,                        //
        selector, sense,                    //
        n, s, n,                            //
        &sdim, pwr, pwi,                    //
        q, n, &rconde,                      //
        &rcondv, pwork, lwork,              //
        piwork, liwork,                     //
        pbwork);

    if (info != 0)
        throw std::runtime_error("info != 0");
    return {S, U};
}

template <size_t N>
struct Balance_result {
    Matrix<N, N> balanced_mat;
    Matrix<N, N> balancing_mat;
};

// octave-4.4.1/liboctave/numeric/aepbalance.cc
// octave-4.4.1/liboctave/numeric/aepbalance.h
// octave-4.4.1/liboctave/numeric/schur.cc
// octave-4.4.1/liboctave/numeric/schur.h
template <size_t N>
Balance_result<N> balance(const Matrix<N, N> &A, bool noperm = false,
                          bool noscal = false) {
    Balance_result<N> result;
    result.balanced_mat = A;
    double *a           = toArrayPointer(result.balanced_mat);
    char job            = noperm ? (noscal ? 'N' : 'S') : (noscal ? 'P' : 'B');
    double scale[N];
    lapack_int ilo, ihi;
    lapack_int n = N;
    lapack_int info =
        LAPACKE_dgebal(LAPACK_ROW_MAJOR, job, n, a, n, &ilo, &ihi, scale);
    if (info != 0)
        throw std::runtime_error("info != 0");

    result.balancing_mat = eye<N>();
    char side            = 'R';
    double *b            = toArrayPointer(result.balancing_mat);
    info = LAPACKE_dgebak(LAPACK_ROW_MAJOR, job, side, n, ilo, ihi, scale, n, b,
                          n);
    if (info != 0)
        throw std::runtime_error("info != 0");
    return result;
}

// http://lasp.colorado.edu/cism/CISM_DX/code/CISM_DX-0.50/required_packages/octave/share/octave/2.1.50/m/control/base/are.m
template <size_t Nx, size_t Nu>
Matrix<Nx, Nx> are(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                   const Matrix<Nx, Nx> &C) {
    using Matrices::T;
    // assert(is_controllable(a,b));  // TODO
    // assert(is_observable(a, c));   // TODO
    auto temp = vcat(     //
        hcat(A, -B),      //
        hcat(-C, -A ^ T)  //
    );

    auto balRes = balance(temp);
    auto D      = balRes.balancing_mat;
    auto H      = balRes.balanced_mat;

    auto schurRes = schur(H, "A");

    auto U = schurRes.U;
    // auto S  = schurRes.S;

    U       = D * U;
    auto U1 = getBlock<Nx, 2 * Nx, 0, Nx>(U);
    auto U2 = getBlock<0, Nx, 0, Nx>(U);
    auto X  = U1 * inv(U2);
    return X;
}

// http://lasp.colorado.edu/cism/CISM_DX/code/CISM_DX-0.50/required_packages/octave/share/octave/2.1.50/m/control/base/lqr.m
template <size_t Nx, size_t Nu>
struct LQR_result {
    Matrix<Nx, Nx> P;
    Matrix<Nu, Nx> K;
};

template <size_t Nx, size_t Nu>
LQR_result<Nx, Nu> lqr(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                       const Matrix<Nx, Nx> &Q, const Matrix<Nu, Nu> &R) {
    using Matrices::T;

    // assert(issymmetric(Q) && issymmetric(R));      // TODO
    // assert(all(eig(q) >= 0) && all(eig(r) > 0)));  // TODO
    Matrix<Nx, Nu> S = zeros<Nx, Nu>();
    auto P           = are(A, B * inv(R) * (B ^ T), Q);
    auto K           = solveLeastSquares(R, (B ^ T) * P + (S ^ T));
    return {P, K};
}