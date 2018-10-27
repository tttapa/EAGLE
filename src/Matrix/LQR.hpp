#pragma once

#include <Matrix/LeastSquares.hpp>
#include <Matrix/Matrix.hpp>
#include <string>

extern "C" {
#include <lapacke.h>
};

// http://lasp.colorado.edu/cism/CISM_DX/code/CISM_DX-0.50/required_packages/octave/share/octave/2.1.50/m/control/base/lqr.m
template <size_t Nx, size_t Nu>
struct LQR_result {
    Matrix<Nu, Nx> K;
    // TODO
};

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

    // Workspace requirements may need to be fixed if any of the
    // following change.

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

    double *s = &S[0][0];
    double *q = &U[0][0];

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

    std::cout << info << std::endl;
    return {S, U};
}

template <size_t Nx, size_t Nu>
Matrix<Nx, Nx> are(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                   const Matrix<Nx, Nx> &C) {
    using Matrices::T;
    // assert(is_controllable(a,b));  // TODO
    // assert(is_observable(a, c));   // TODO
    auto temp     = vcat(     //
        hcat(A, -B),      //
        hcat(-C, -A ^ T)  //
    );
    auto balRes   = balance(temp);
    auto D        = balRes.D;
    auto H        = balRes.H;
    auto schurRes = schur(H, "A");

    auto U  = schurRes.U;
    auto S  = schurRes.S;
    U       = D * U;
    auto U1 = getBlock<Nx, 2 * Nx, 0, Nx>(U);
    auto U2 = getBlock<0, Nx, 0, Nx>(U);
    auto X  = U1 * inv(U2);
    return X;
}

template <size_t Nx, size_t Nu>
LQR_result<Nx, Nu> lqr(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                       const Matrix<Nx, Nx> &Q, const Matrix<Nu, Nu> &R) {
    using Matrices::T;
    // assert(issymmetric(Q) && issymmetric(R));      // TODO
    // assert(all(eig(q) >= 0) && all(eig(r) > 0)));  // TODO
    Matrix<Nx, Nu> S = zeros<Nx, Nu>();
    auto P           = are(A, B * inv(R) * (B ^ T), Q);
    auto K           = solveLeastSquares(R, (B ^ T) * P + (S ^ T));
    return K;
}