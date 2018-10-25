#pragma once

#include <Model/Controller.hpp>

class ContinuousLQRController : public ContinuousController<double, 10, 3, 7> {
  public:
    static constexpr size_t nx = 10;
    static constexpr size_t nu = 3;
    static constexpr size_t ny = 7;

    ContinuousLQRController(const Matrix<double, nx, nx> &A,
                            const Matrix<double, nx, nu> &B,
                            const Matrix<double, ny, nx> &C,
                            const Matrix<double, ny, nu> &D,
                            const Matrix<double, nu, nx> &K)
        : K(K) {
        /* W =  [ A B ]
                [ C D ] */
        Matrix<double, nx + ny, nx + nu> W = vcat(hcat(A, B), hcat(C, D));
        Matrix<double, nx + ny, ny> OI =
            vcat(zeros<double, nx, ny>(), eye<double, ny>());
        G = solveLeastSquares(W, OI);
    }

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        // new equilibrium state
        ColVector<double, nx + nu> eq = G * r;
        ColVector<double, nx> xeq     = getBlock<0, 10, 0, 1>(eq);
        ColVector<double, nu> ueq     = getBlock<10, 13, 0, 1>(eq);

        // error
        ColVector<double, nx> xdiff    = x - xeq;
        Quaternion qx                  = getBlock<0, 4, 0, 1>(x);
        Quaternion qe                  = getBlock<0, 4, 0, 1>(xeq);
        assignBlock<0, 4, 0, 1>(xdiff) = quatDifference(qx, qe);

        // controller
        ColVector<double, nu> uc = K * xdiff;
        auto u                   = uc + ueq;
        return u;
    }

  private:
    Matrix<double, nx + nu, ny> G;
    Matrix<double, nu, nx> K;
};