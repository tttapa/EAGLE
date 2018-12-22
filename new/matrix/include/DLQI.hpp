#pragma once

#include "DLQR.hpp"

template <size_t Nx, size_t Nu, size_t Ny>
DLQR_result<Nx + Ny, Nu> dlqi(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                              const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D,
                              const Matrix<Nx + Ny, Nx + Ny> &Q,
                              const Matrix<Nu, Nu> &R, double Ts) {
    auto Aaug = vcat(hcat(A, zeros<Nx, Ny>()), hcat(-C * Ts, eye<Ny>()));
    auto Baug = vcat(B, -D * Ts);
    return dlqr(Aaug, Baug, Q, R);
}