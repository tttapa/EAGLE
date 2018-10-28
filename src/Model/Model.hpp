#pragma once

#include "Params.hpp"
#include <ODE/DormandPrince.hpp>
#include <Util/TimeFunction.hpp>

template <size_t Nx, size_t Nu>
class Model {
  public:
    typedef ColVector<Nx> VecX_t;
    typedef ColVector<Nu> VecU_t;
    typedef TimeFunctionT<VecU_t> InputFunction;
    typedef ODEResultX<VecX_t> SimulationResult;

    virtual VecX_t operator()(const VecX_t &x, const VecU_t &u) = 0;
    virtual SimulationResult simulate(InputFunction &u,  // input to the model
                                      VecX_t x_start,    // initial state
                                      const AdaptiveODEOptions &opt  // options
                                      )                         = 0;
    virtual SimulationResult simulate(VecU_t u,        // input to the model
                                      VecX_t x_start,  // initial state
                                      const AdaptiveODEOptions &opt  // options
    ) {
        ConstantTimeFunctionT<VecU_t> fu = u;
        return simulate(fu, x_start, opt);
    }
};

template <size_t Nx, size_t Nu>
class ContinuousModel : public Model<Nx, Nu> {
  public:
    using VecX_t           = typename Model<Nx, Nu>::VecX_t;
    using VecU_t           = typename Model<Nx, Nu>::VecU_t;
    using InputFunction    = typename Model<Nx, Nu>::InputFunction;
    using SimulationResult = typename Model<Nx, Nu>::SimulationResult;

    SimulationResult simulate(InputFunction &u,  // input to the model
                              VecX_t x_start,    // initial state
                              const AdaptiveODEOptions &opt  // options
                              ) override {
        auto f = [this, &u](double t, const VecX_t &x) {
            ContinuousModel &model = *this;
            return model(x, u(t));
        };
        return dormandPrince(f, x_start, opt);
    }
};

template <size_t Nx, size_t Nu, size_t Ny>
class CTLTIModel : public ContinuousModel<Nx, Nu> {
  public:
    using VecX_t = typename Model<Nx, Nu>::VecX_t;
    using VecU_t = typename Model<Nx, Nu>::VecU_t;
    using VecY_t = ColVector<Ny>;

    CTLTIModel(const Matrix<Nx, Nx> &A, const Matrix<Nx, Nu> &B,
               const Matrix<Ny, Nx> &C, const Matrix<Ny, Nu> &D)
        : A(A), B(B), C(C), D(D) {}

    VecX_t operator()(const VecX_t &x, const VecU_t &u) override {
        return A * x + B * u;
    }

    VecY_t getOutput(const VecX_t &x, const VecU_t &u) const override {
        return C * x + D * u;
    }

    const Matrix<Nx, Nx> A;
    const Matrix<Nx, Nu> B;
    const Matrix<Ny, Nx> C;
    const Matrix<Ny, Nu> D;
};