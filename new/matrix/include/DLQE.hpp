#pragma once

#include "DLQR.hpp"

template <size_t Nx, size_t Ny>
struct DLQE_result {    
    Matrix<Nx, Nx> P;
    Matrix<Nx, Ny> L;
};

template <size_t Nx, size_t Nu, size_t Ny>
DLQE_result<Nx, Ny> dlqe(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &G,
                         const Matrix<Ny, Nx> &C, const Matrix<Nu, Nu> &W,
                         const Matrix<Ny, Ny> &V) {
    using Matrices::T;
    auto dlqrRes     = dlqr(A ^ T, C ^ T, G * W * (G ^ T), V);
    // auto K           = dlqrRes.K;
    auto P           = dlqrRes.P;
    auto D           = P * (C ^ T);
    auto F           = inv(C * D + V);
    Matrix<Nx, Ny> L = D * F;
    return {P, L};
}