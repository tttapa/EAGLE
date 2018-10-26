#pragma once

#include <Model/Controller.hpp>
#include <algorithm>

class ContinuousLQRController : public ContinuousController<10, 3, 7> {
  public:
    static constexpr size_t nx = 10;
    static constexpr size_t nu = 3;
    static constexpr size_t ny = 7;

    ContinuousLQRController(const Matrix<nx, nx> &A, const Matrix<nx, nu> &B,
                            const Matrix<ny, nx> &C, const Matrix<ny, nu> &D,
                            const Matrix<nu, nx> &K)
        : K(K) {
        /* W =  [ A B ]
                [ C D ] */
        Matrix<nx + ny, nx + nu> W = vcat(hcat(A, B), hcat(C, D));
        Matrix<nx + ny, ny> OI     = vcat(zeros<nx, ny>(), eye<ny>());
        G                          = solveLeastSquares(W, OI);
    }

    VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
        // new equilibrium state
        ColVector<nx + nu> eq = G * r;
        ColVector<nx> xeq     = getBlock<0, 10, 0, 1>(eq);
        ColVector<nu> ueq     = getBlock<10, 13, 0, 1>(eq);

        // error
        ColVector<nx> xdiff            = x - xeq;
        Quaternion qx                  = getBlock<0, 4, 0, 1>(x);
        Quaternion qe                  = getBlock<0, 4, 0, 1>(xeq);
        assignBlock<0, 4, 0, 1>(xdiff) = quatDifference(qx, qe);

        // controller
        ColVector<nu> uc = K * xdiff;
        auto u           = uc + ueq;
        return u;
    }

    std::vector<VecU_t> getControlSignal(const std::vector<double> &time,
                                         const std::vector<VecX_t> &states,
                                         ReferenceFunction &ref) {
        std::vector<ContinuousLQRController::VecU_t> u;
        u.resize(time.size());
        auto lambda = [this, &ref](double t, auto x) {
            return (*this)(x, ref(t));
        };
        std::transform(time.begin(), time.end(), states.begin(), u.begin(),
                       lambda);
        return u;
    }

  private:
    Matrix<nx + nu, ny> G;
    Matrix<nu, nx> K;
};