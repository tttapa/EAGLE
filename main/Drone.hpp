#pragma once

#include "KalmanObserver.hpp"
#include "LQRController.hpp"

#include <Model/Model.hpp>

#include <Matrix/DLQE.hpp>
#include <Matrix/DLQR.hpp>

#include <Util/AlmostEqual.hpp>
#include <Util/FileLoader.hpp>

#include <cassert>
#include <filesystem>
#include <iostream>

class DroneState {
    ColVector<17> x = {1};

  public:
    DroneState() = default;
    DroneState(const ColVector<17> &x) : x{x} {}
    ColVector<Nx_att> getAttitudeState() const {
        return getBlock<0, Nx_att, 0, 1>(x);
    }
    Quaternion getOrientation() const { return getBlock<0, 4, 0, 1>(x); }
    ColVector<3> getAngularVelocity() const { return getBlock<4, 7, 0, 1>(x); }
    ColVector<3> getMotorSpeed() const { return getBlock<7, 10, 0, 1>(x); }
    ColVector<3> getVelocity() const { return getBlock<10, 13, 0, 1>(x); }
    ColVector<3> getPosition() const { return getBlock<13, 16, 0, 1>(x); }
    double getThrustMotorSpeed() const { return x[16][0]; }
    ColVector<Nx_nav> getNavigationState() const {
        return getBlock<Nx_att, Nx, 0, 1>(x);
    }

    void setOrientation(const Quaternion &q) { assignBlock<0, 4, 0, 1>(x) = q; }
    void setAngularVelocity(const ColVector<3> &w) {
        assignBlock<4, 7, 0, 1>(x) = w;
    }
    void setMotorSpeed(const ColVector<3> &n) {
        assignBlock<7, 10, 0, 1>(x) = n;
    }
    void setVelocity(const ColVector<3> &v) {
        assignBlock<10, 13, 0, 1>(x) = v;
    }
    void setPosition(const ColVector<3> &z) {
        assignBlock<13, 16, 0, 1>(x) = z;
    }
    void setThrustMotorSpeed(double t) { x[16][0] = t; }
    operator ColVector<17>() const { return x; }
};

class DroneControl {
    ColVector<4> u = {};

  public:
    DroneControl() = default;
    DroneControl(const ColVector<4> &u) : u{u} {}
    ColVector<3> getAttitudeControl() const { return getBlock<0, 3, 0, 1>(u); }
    double getThrustControl() const { return u[3][0]; }
    operator ColVector<4>() const { return u; }
};

class DroneOutput {
    ColVector<10> y = {};

  public:
    DroneOutput() = default;
    DroneOutput(const ColVector<10> &y) : y{y} {}
    DroneOutput(const ColVector<Ny_att> &y_att, const ColVector<Ny_nav> &y_nav)
        : y{vcat(y_att, y_nav)} {}
    Quaternion getOrientation() const { return getBlock<0, 4, 0, 1>(y); }
    ColVector<3> getAngularVelocity() const { return getBlock<4, 7, 0, 1>(y); }
    ColVector<3> getPosition() const { return getBlock<7, 10, 0, 1>(y); }

    void setOrientation(const Quaternion &q) { assignBlock<0, 4, 0, 1>(y) = q; }
    void setAngularVelocity(const ColVector<3> &w) {
        assignBlock<4, 7, 0, 1>(y) = w;
    }
    void setAttitudeOutput(const ColVector<7> &yy) {
        assignBlock<0, 7, 0, 1>(y) = yy;
    }
    void setPosition(const ColVector<3> &z) { assignBlock<7, 10, 0, 1>(y) = z; }
    operator ColVector<10>() const { return y; }
};

struct Drone : public ContinuousModel<Nx, Nu, Ny> {

#pragma region Constructors.....................................................

    /**
     * @brief   TODO
     * 
     */
    Drone(const std::filesystem::path &loadPath) { load(loadPath); }

    /** 
     * @brief   Loads all system matrices and parameters from the respective 
     *          files in the given folder. 
     */
    void load(const std::filesystem::path &loadPath) {
        Aa = loadMatrix<Nx_att, Nx_att>(loadPath / "Aa");
        Ba = loadMatrix<Nx_att, Nu_att>(loadPath / "Ba");
        Ca = loadMatrix<Ny_att, Nx_att>(loadPath / "Ca");
        Da = loadMatrix<Ny_att, Nu_att>(loadPath / "Da");

        Ad = loadMatrix<Nx_att, Nx_att>(loadPath / "Ad");
        Bd = loadMatrix<Nx_att, Nu_att>(loadPath / "Bd");
        Cd = loadMatrix<Ny_att, Nx_att>(loadPath / "Cd");
        Dd = loadMatrix<Ny_att, Nu_att>(loadPath / "Dd");

        Ad_r = getBlock<1, Nx_att, 1, Nx_att>(Ad);
        Bd_r = getBlock<1, Nx_att, 0, Nu_att>(Bd);
        Cd_r = getBlock<1, Ny_att, 1, Nx_att>(Cd);

        Ts = loadDouble(loadPath / "Ts");

        gamma_n = loadMatrix<3, 3>(loadPath / "gamma_n");
        gamma_u = loadMatrix<3, 3>(loadPath / "gamma_u");
        I       = loadMatrix<3, 3>(loadPath / "I");
        I_inv   = loadMatrix<3, 3>(loadPath / "I_inv");
        k1      = loadDouble(loadPath / "k1");
        k2      = loadDouble(loadPath / "k2");

        assert(isAlmostEqual(I_inv, inv(I), 1e-12));
    }

#pragma region Continuous Model.................................................

    /** 
     * @brief   Calculate the derivative of the state vector, given the current
     *          state x and the current input u.
     * @param   x 
     *          The current state of the drone.
     * @param   u 
     *          The current control input u.
     * @return  The derivative of the state, x_dot.
     */
    VecX_t operator()(const VecX_t &x, const VecU_t &u) override {
        DroneState xx   = {x};
        DroneControl uu = {u};
        DroneState x_dot;

        // Attitude
        Quaternion q       = xx.getOrientation();
        ColVector<3> omega = xx.getAngularVelocity();
        ColVector<3> n     = xx.getMotorSpeed();
        ColVector<3> u_att = uu.getAttitudeControl();

        Quaternion q_omega = vcat(zeros<1, 1>(), omega);
        x_dot.setOrientation(0.5 * quatmultiply(q, q_omega));
        x_dot.setAngularVelocity(gamma_n * n + gamma_u * u_att -
                                 I_inv * cross(omega, I * omega));
        x_dot.setMotorSpeed(k2 * (k1 * u_att - n));

        // Navigation/Altitude
        ColVector<3> v  = xx.getVelocity();
        double n_thrust = xx.getThrustMotorSpeed();
        double u_thrust = uu.getThrustControl();

        double F_local_z     = ct * rho * pow(Dp, 4) * pow(n_thrust, 2);
        ColVector<3> F_local = {0, 0, F_local_z};
        ColVector<3> F_world = quatrotate(q, F_local);
        ColVector<3> F_grav  = {0, 0, -g * m};
        ColVector<3> a       = (F_world + F_grav) / m;

        x_dot.setVelocity(a);
        x_dot.setPosition(v);
        x_dot.setThrustMotorSpeed(k2 * (k1 * u_thrust - n_thrust));

        return x_dot;
    }

    VecY_t getOutput(const VecX_t &x, const VecU_t &u) override {
        DroneState xx   = {x};
        DroneControl uu = {u};
        ColVector<Ny_att> y_att =
            Ca * xx.getAttitudeState() + Da * uu.getAttitudeControl();
        ColVector<Ny_nav> y_nav = xx.getPosition();
        return DroneOutput{y_att, y_nav};
    }

#pragma region Controllers......................................................

    /** 
     * @brief   TODO
     */
    Matrix<Nu_att, Nx_att - 1>
    getAttitudeControllerMatrixK(const Matrix<Nx_att - 1, Nx_att - 1> &Q,
                                 const Matrix<Nu_att, Nu_att> &R) const {
        return -dlqr(Ad_r, Bd_r, Q, R).K;
    }

    /** 
     * @brief   TODO
     */
    Attitude::LQRController
    getAttitudeController(const Matrix<Nx_att - 1, Nx_att - 1> &Q,
                          const Matrix<Nu_att, Nu_att> &R) const {
        auto K_red = getAttitudeControllerMatrixK(Q, R);
        return {Ad, Bd, Cd, Dd, K_red, Ts};
    }

    /** 
     * @brief   TODO
     */
    Attitude::ClampedLQRController
    getClampedDiscreteController(const ColVector<Nu_att> &clampMin,
                                 const ColVector<Nu_att> &clampMax,
                                 const Matrix<Nx_att - 1, Nx_att - 1> &Q,
                                 const Matrix<Nu_att, Nu_att> &R) const {
        return {getAttitudeController(Q, R), clampMin, clampMax};
    }

#pragma region Observers........................................................

    /** 
     * @brief   TODO
     */
    Matrix<Nx_att - 1, Ny_att - 1>
    getAttitudeObserverMatrixL(const RowVector<Nu_att> &varDynamics,
                               const RowVector<Ny_att - 1> &varSensors) const {
        return dlqe(Ad_r, Bd_r, Cd_r, diag(varDynamics), diag(varSensors)).L;
    }

    /** 
     * @brief   TODO
     */
    Attitude::KalmanObserver
    getAttitudeObserver(const RowVector<Nu_att> &varDynamics,
                        const RowVector<Ny_att - 1> &varSensors) const {
        auto L_red = getAttitudeObserverMatrixL(varDynamics, varSensors);
        return {Ad_r, Bd_r, Cd, L_red, Ts};
    }

#pragma region System matrices..................................................

    /** 
     * ```
     *  Aa =  [  ·   ·   ·   ·   ·   ·   ·   ·   ·   ·  ]
     *        [  ·   ·   ·   ·  ┌─────────┐  ·   ·   ·  ]
     *        [  ·   ·   ·   ·  │  0.5 I3 │  ·   ·   ·  ]
     *        [  ·   ·   ·   ·  └─────────┘  ·   ·   ·  ]
     *        [  ·   ·   ·   ·   ·   ·   ·  ┌─────────┐ ]
     *        [  ·   ·   ·   ·   ·   ·   ·  │   Γ_n   │ ]
     *        [  ·   ·   ·   ·   ·   ·   ·  └─────────┘ ]
     *        [  ·   ·   ·   ·   ·   ·   ·  ┌─────────┐ ]
     *        [  ·   ·   ·   ·   ·   ·   ·  │ -k2 I3  │ ]
     *        [  ·   ·   ·   ·   ·   ·   ·  └─────────┘ ] 
     * ``` */
    Matrix<Nx_att, Nx_att> Aa = {};

    /** 
     * Discrete A 
     */
    Matrix<Nx_att, Nx_att> Ad = {};

    /** 
     * Discrete, reduced A 
     */
    Matrix<Nx_att - 1, Nx_att - 1> Ad_r = {};

    /** 
     * ```
     *  Ba =  [  ·   ·   ·  ]
     *        [  ·   ·   ·  ]
     *        [  ·   ·   ·  ]
     *        [  ·   ·   ·  ]
     *        [ ┌─────────┐ ]
     *        [ │   Γ_u   │ ]
     *        [ └─────────┘ ]
     *        [ ┌─────────┐ ]
     *        [ │ k1k2 I3 │ ]
     *        [ └─────────┘ ] 
     * ``` */
    Matrix<Nx_att, Nu_att> Ba = {};

    /** 
     * Discrete B
     */
    Matrix<Nx_att, Nu_att> Bd = {};

    /** 
     * Discrete, reduced B
     */
    Matrix<Nx_att - 1, Nu_att> Bd_r = {};

    /**
     * ```
     *  Ca =  [ 1  ·  ·  ·  ·  ·  ·  ·  ·  · ]
     *        [ ·  1  ·  ·  ·  ·  ·  ·  ·  · ]
     *        [ ·  ·  1  ·  ·  ·  ·  ·  ·  · ]
     *        [ ·  ·  ·  1  ·  ·  ·  ·  ·  · ]
     *        [ ·  ·  ·  ·  1  ·  ·  ·  ·  · ]
     *        [ ·  ·  ·  ·  ·  1  ·  ·  ·  · ]
     *        [ ·  ·  ·  ·  ·  ·  1  ·  ·  · ] 
     * ``` */
    Matrix<Ny_att, Nx_att> Ca = {};

    /**
     * Discrete C
     */
    Matrix<Ny_att, Nx_att> Cd = {};

    /**
     * Discrete, reduced C
     */
    Matrix<Ny_att - 1, Nx_att - 1> Cd_r = {};

    /** 
     * ```
     *  Da =  [ ·  ·  · ]
     *        [ ·  ·  · ]
     *        [ ·  ·  · ]
     *        [ ·  ·  · ]
     *        [ ·  ·  · ]
     *        [ ·  ·  · ]
     *        [ ·  ·  · ] 
     * ``` */
    Matrix<Ny_att, Nu_att> Da = {};

    /**
     * Discrete D
     */
    Matrix<Ny_att, Nu_att> Dd = {};

#pragma region System parameters................................................

    double Ts = 0;

    Matrix<3, 3> gamma_n = {};
    Matrix<3, 3> gamma_u = {};
    Matrix<3, 3> I       = {};
    Matrix<3, 3> I_inv   = {};
    double k1            = 0;
    double k2            = 0;

    const double m   = 1.850;  ///< kg           total mass
    const double ct  = 0.1;    ///< -            thrust coefficient
    const double g   = 9.81;   ///< m/s2         gravitational acceleration
    const double rho = 1.225;  ///< kg/m3        air density (nominal, at 15C,
                               ///               sea level)
    const double Dp = 12.0 * 0.0254;  ///< m       propeller diameter (12")
};
