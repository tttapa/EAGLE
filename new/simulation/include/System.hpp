#pragma once

#include <LeastSquares.hpp>  // inv

template <size_t Nx, size_t Nu, size_t Ny>
class LTISystem {
  public:
    typedef ColVector<Nx> VecX_t;
    typedef ColVector<Nu> VecU_t;
    typedef ColVector<Ny> VecY_t;

    LTISystem(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
              const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D)
        : A(A), B(B), C(C), D(D) {}

    VecX_t getStateChange(const VecX_t &x, const VecU_t &u) const {
        return A * x + B * u;
    }

    VecY_t getSystemOutput(const VecX_t &x, const VecU_t &u) const {
        return C * x + D * u;
    }

    const Matrix<Nx, Nx> A;
    const Matrix<Nx, Nu> B;
    const Matrix<Ny, Nx> C;
    const Matrix<Ny, Nu> D;
};

template <size_t Nx, size_t Nu, size_t Ny>
class DTLTISystem : public LTISystem<Nx, Nu, Ny> {
  public:
    DTLTISystem(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D, double Ts)
        : LTISystem<Nx, Nu, Ny>{A, B, C, D}, Ts{Ts} {}

    const double Ts;
};

enum class DiscretizationMethod {
    Euler         = 0,
    BackwardEuler = 1,
    Bilinear      = 2,
};

template <size_t Nx, size_t Nu, size_t Ny>
class CTLTISystem : public LTISystem<Nx, Nu, Ny> {
  public:
    CTLTISystem(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
                const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D)
        : LTISystem<Nx, Nu, Ny>{A, B, C, D} {}

    DTLTISystem<Nx, Nu, Ny>
    discretize(double Ts,
               DiscretizationMethod method = DiscretizationMethod::Euler) {
        auto A  = this->A;
        auto B  = this->B;
        auto C  = this->C;
        auto D  = this->D;
        auto Ad = A;
        auto Bd = B;
        auto Cd = C;
        auto Dd = D;
        switch (method) {
            case DiscretizationMethod::Euler:
                Ad = eye<Nx>() + Ts * A;
                Bd = Ts * B;
                break;
            case DiscretizationMethod::BackwardEuler:
                Ad = inv(eye<Nx>() - Ts * A);
                Bd = Ts * Ad * B;
                Cd = C * Ad;
                Dd = D + C * Bd;
                break;
            case DiscretizationMethod::Bilinear:
                Ad = inv(eye<Nx>() - Ts * A / 2) * (eye<Nx>() + Ts * A / 2);
                Bd = inv(eye<Nx>() - Ts * A / 2) * B * Ts;
                Cd = C * inv(eye<Nx>() - Ts * A / 2);
                Dd = D + C * Bd / 2;
                break;
            default: break;
        }
        return {Ad, Bd, Cd, Dd, Ts};
    }
};
