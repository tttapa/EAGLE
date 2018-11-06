#pragma once

#include "FullKalman.hpp"
#include "LQRController.hpp"
#include "NonLinearFullDroneModel.hpp"
#include "Params.hpp"
#include <Matrix/DLQE.hpp>
#include <Matrix/DLQR.hpp>
#include <Matrix/LQR.hpp>

#include <iostream>

struct Drone {
    constexpr Drone() { compute(); }

    constexpr static size_t Nx = 10;
    constexpr static size_t Nu = 3;
    constexpr static size_t Ny = 7;

#pragma region Parameters.......................................................

    /** 
     * @brief   Update all computed parameters and system matrices. 
     *          Should be called each time one of the parameters is changed.
     */
    constexpr void compute() {
        p.compute();
        A = vcat(                                                //
            zeros<1, 10>(),                                      //
            hcat(zeros<3, 4>(), 0.5 * eye<3>(), zeros<3, 3>()),  //
            hcat(zeros<3, 7>(), p.gamma_n),                      //
            hcat(zeros<3, 7>(), -p.k2 * eye<3>())                //
        );
        B = vcat(                   //
            zeros<4, 3>(),          //
            p.gamma_u,              //
            p.k2 * p.k1 * eye<3>()  //
        );
        C = hcat(eye<Ny>(), zeros<Ny, Nu>());
        D = zeros<Ny, Nu>();
    }

    /**
     * @brief   A struct containing the drone's parameters. 
     * 
     * @note    You can edit some of the parameters, however, this invalidates
     *          some of the other derived/computed parameters, so you have to 
     *          call `Drone::compute()` afterwards. 
     */
    Params p = {};

#pragma region Controllers......................................................

    /**
     * @brief   TODO
     */
    ContinuousLQRController
    getContinuousController(const Matrix<Nx - 1, Nx - 1> &Q,
                            const Matrix<Nu, Nu> &R) const {
        auto sys   = getLinearReducedContinuousSystem();
        auto K_red = -lqr(sys.A, sys.B, Q, R).K;
        auto K     = hcat(zeros<Nu, 1>(), K_red);
        return {getLinearFullContinuousModel(), K};
    }

    /** 
     * @brief   TODO
     */
    Matrix<Nu, Nx - 1>
    getReducedDiscreteControllerMatrixK(const Matrix<Nx - 1, Nx - 1> &Q,
                                        const Matrix<Nu, Nu> &R, double Ts,
                                        DiscretizationMethod method) const {
        auto sys   = getLinearReducedDiscreteSystem(Ts, method);
        auto K_red = -dlqr(sys.A, sys.B, Q, R).K;
        return K_red;
    }

    /** 
     * @brief   TODO
     */
    DiscreteLQRController
    getDiscreteController(const Matrix<Nx - 1, Nx - 1> &Q,
                          const Matrix<Nu, Nu> &R, double Ts,
                          DiscretizationMethod method) const {
        auto K_red = getReducedDiscreteControllerMatrixK(Q, R, Ts, method);
        auto K     = hcat(zeros<Nu, 1>(), K_red);
        return {getLinearFullDiscreteSystem(Ts, method), K};
    }

    /** 
     * @brief   TODO
     */
    ClampedDiscreteLQRController
    getClampedDiscreteController(const Matrix<Nx - 1, Nx - 1> &Q,
                                 const Matrix<Nu, Nu> &R, double Ts,
                                 DiscretizationMethod method) const {
        return {getDiscreteController(Q, R, Ts, method), p.uh};
    }

#pragma region Observers........................................................

    Matrix<Nx - 1, Ny - 1> getReducedDiscreteObserverMatrixL(
        const RowVector<Nu> &varDynamics, const RowVector<Ny - 1> &varSensors,
        double Ts, DiscretizationMethod method) const {
        auto sys = getLinearReducedDiscreteSystem(Ts, method);
        auto L_red =
            dlqe(sys.A, sys.B, sys.C, diag(varDynamics), diag(varSensors)).L;
        return L_red;
    }

    /** 
     * @brief   TODO
     */
    FullKalman<Nx, Nu, Ny>
    getDiscreteObserver(const RowVector<Nu> &varDynamics,
                        const RowVector<Ny - 1> &varSensors, double Ts,
                        DiscretizationMethod method) const {
        auto sys_red = getLinearReducedDiscreteSystem(Ts, method);
        auto sys = getLinearFullDiscreteSystem(Ts, method);
        auto L_red =
            dlqe(sys_red.A, sys_red.B, sys_red.C, diag(varDynamics), diag(varSensors)).L;
        return {sys.A, sys.B, sys.C, L_red, Ts};
    }

#pragma region Models and Systems...............................................

    /** 
     * @brief   Get the true continuous-time, non-linear, full model of the 
     *          drone.
     */
    NonLinearFullDroneModel getNonLinearFullModel() const { return {p}; }

    /** 
     * @brief   Get a continuous-time system representing the drone,
     *          linearized around the equilibrium 
     *          @f$ x = (1, 0, 0, 0, 0, 0, 0, 0, 0, 0), u = (0, 0, 0) @f$.
     */
    CTLTISystem<Nx, Nu, Ny> getLinearFullContinuousSystem() const {
        return {A, B, C, D};
    }

    /** 
     * @brief   Get a reduced continuous-time system representing the drone,
     *          linearized around the equilibrium 
     *          @f$ x = (0, 0, 0, 0, 0, 0, 0, 0, 0), u = (0, 0, 0) @f$.
     *          The first state (the real part of the orienation quaternion)
     *          has been removed from the state-space representation.  
     *          It can be approximated by @f$ q_0 \approx \sqrt{1 - q_1^2 - 
     *          q_2^2 - q_3^2} @f$.
     */
    CTLTISystem<Nx - 1, Nu, Ny - 1> getLinearReducedContinuousSystem() const {
        auto A_red = getBlock<1, Nx, 1, Nx>(A);
        auto B_red = getBlock<1, Nx, 0, Nu>(B);
        auto C_red = getBlock<1, Ny, 1, Nx>(C);
        auto D_red = getBlock<1, Ny, 0, Nu>(D);
        return {A_red, B_red, C_red, D_red};
    }

    /** 
     * @brief   Get a continuous-time model of the drone that has been 
     *          linearized around the equilibrium 
     *          @f$ x = (1, 0, 0, 0, 0, 0, 0, 0, 0, 0), u = (0, 0, 0) @f$.
     */
    CTLTIModel<Nx, Nu, Ny> getLinearFullContinuousModel() const {
        return {getLinearFullContinuousSystem()};
    }

    /** 
     * @brief   Get a discrete-time system representing the drone, a discretized
     *          version of `getLinearFullContinuousSystem`, using the given 
     *          sample time and discretization method.
     */
    DTLTISystem<Nx, Nu, Ny>
    getLinearFullDiscreteSystem(double Ts, DiscretizationMethod method) const {
        auto ctsys = getLinearFullContinuousSystem();
        return ctsys.discretize(Ts, method);
    }

    /** 
     * @brief   Get a reduced discrete-time system representing the drone,
     *          linearized around the equilibrium 
     *          @f$ x = (0, 0, 0, 0, 0, 0, 0, 0, 0), u = (0, 0, 0) @f$, and 
     *          discretized using the given sample time and discretization 
     *          method.  
     *          The first state (the real part of the orienation quaternion)
     *          has been removed from the state-space representation.  
     *          It can be approximated by @f$ q_0 \approx \sqrt{1 - q_1^2 -
     *          q_2^2 - q_3^2} @f$.
     */
    DTLTISystem<Nx - 1, Nu, Ny - 1>
    getLinearReducedDiscreteSystem(double Ts,
                                   DiscretizationMethod method) const {
        auto discr  = getLinearFullDiscreteSystem(Ts, method);
        auto Ad_red = getBlock<1, Nx, 1, Nx>(discr.A);
        auto Bd_red = getBlock<1, Nx, 0, Nu>(discr.B);
        auto Cd_red = getBlock<1, Ny, 1, Nx>(discr.C);
        auto Dd_red = getBlock<1, Ny, 0, Nu>(discr.D);
        return {Ad_red, Bd_red, Cd_red, Dd_red, Ts};
    }

#pragma region System matrices..................................................

    /** 
     * ```
     *  A =  [  ·   ·   ·   ·   ·   ·   ·   ·   ·   ·  ]
     *       [  ·   ·   ·   ·  ┌─────────┐  ·   ·   ·  ]
     *       [  ·   ·   ·   ·  │  0.5 I3 │  ·   ·   ·  ]
     *       [  ·   ·   ·   ·  └─────────┘  ·   ·   ·  ]
     *       [  ·   ·   ·   ·   ·   ·   ·  ┌─────────┐ ]
     *       [  ·   ·   ·   ·   ·   ·   ·  │   Γ_n   │ ]
     *       [  ·   ·   ·   ·   ·   ·   ·  └─────────┘ ]
     *       [  ·   ·   ·   ·   ·   ·   ·  ┌─────────┐ ]
     *       [  ·   ·   ·   ·   ·   ·   ·  │ -k2 I3  │ ]
     *       [  ·   ·   ·   ·   ·   ·   ·  └─────────┘ ] 
     * ``` */
    Matrix<Nx, Nx> A = {};

    /** 
     * ```
     *  B =  [  ·   ·   ·  ]
     *       [  ·   ·   ·  ]
     *       [  ·   ·   ·  ]
     *       [  ·   ·   ·  ]
     *       [ ┌─────────┐ ]
     *       [ │   Γ_u   │ ]
     *       [ └─────────┘ ]
     *       [ ┌─────────┐ ]
     *       [ │ k1k2 I3 │ ]
     *       [ └─────────┘ ] 
     * ``` */
    Matrix<Nx, Nu> B = {};

    /**
     * ```
     *  C =  [ 1  ·  ·  ·  ·  ·  ·  ·  ·  · ]
     *       [ ·  1  ·  ·  ·  ·  ·  ·  ·  · ]
     *       [ ·  ·  1  ·  ·  ·  ·  ·  ·  · ]
     *       [ ·  ·  ·  1  ·  ·  ·  ·  ·  · ]
     *       [ ·  ·  ·  ·  1  ·  ·  ·  ·  · ]
     *       [ ·  ·  ·  ·  ·  1  ·  ·  ·  · ]
     *       [ ·  ·  ·  ·  ·  ·  1  ·  ·  · ] 
     * ``` */
    Matrix<Ny, Nx> C = {};

    /** 
     * ```
     *  D =  [ ·  ·  · ]
     *       [ ·  ·  · ]
     *       [ ·  ·  · ]
     *       [ ·  ·  · ]
     *       [ ·  ·  · ]
     *       [ ·  ·  · ]
     *       [ ·  ·  · ] 
     * ``` */
    Matrix<Ny, Nu> D = {};
};
