#pragma once

#include "KalmanObserver.hpp"
#include "LQRController.hpp"

#include <Model/Model.hpp>

#include <Matrix/DLQE.hpp>
#include <Matrix/DLQR.hpp>

#include <Util/AlmostEqual.hpp>
#include <Util/FileLoader.hpp>
#include <Util/PerfTimer.hpp>

#include <cassert>
#include <filesystem>
#include <iostream>

class DroneState {
    ColVector<17> x = {1};

  public:
    DroneState() = default;
    DroneState(const ColVector<17> &x) : x{x} {}
    ColVector<Nx_att> getAttitude() const {
        return getBlock<0, Nx_att, 0, 1>(x);
    }
    Quaternion getOrientation() const { return getBlock<0, 4, 0, 1>(x); }
    ColVector<3> getAngularVelocity() const { return getBlock<4, 7, 0, 1>(x); }
    ColVector<3> getMotorSpeed() const { return getBlock<7, 10, 0, 1>(x); }
    ColVector<3> getVelocity() const { return getBlock<10, 13, 0, 1>(x); }
    ColVector<3> getPosition() const { return getBlock<13, 16, 0, 1>(x); }
    double getThrustMotorSpeed() const { return x[16]; }
    ColVector<3> getAltitude() const {
        return vcat(getBlock<16, 17, 0, 1>(x),  // n
                    getBlock<15, 16, 0, 1>(x),  // z
                    getBlock<12, 13, 0, 1>(x)   // v_z
        );
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
    void setThrustMotorSpeed(double t) { x[16] = {t}; }
    operator ColVector<17>() const { return x; }
};

class DroneControl {
    ColVector<4> u = {};

  public:
    DroneControl() = default;
    DroneControl(const ColVector<4> &u) : u{u} {}
    ColVector<3> getAttitudeControl() const { return getBlock<0, 3, 0, 1>(u); }
    double getThrustControl() const { return u[3]; }
    void setAttitudeControl(const ColVector<3> &u_att) {
        assignBlock<0, 3, 0, 1>(u) = u_att;
    }
    void setThrustControl(double u_thrust) { u[3] = {u_thrust}; }
    operator ColVector<4>() const { return u; }
};

class DroneOutput {
    ColVector<10> y = {};

  public:
    DroneOutput() = default;
    DroneOutput(const ColVector<10> &y) : y{y} {}
    DroneOutput(const ColVector<Ny_att> &y_att,
                const ColVector<Ny_nav + Ny_alt> &y_pos)
        : y{vcat(y_att, y_pos)} {}
    Quaternion getOrientation() const { return getBlock<0, 4, 0, 1>(y); }
    ColVector<3> getAngularVelocity() const { return getBlock<4, 7, 0, 1>(y); }
    ColVector<7> getAttitude() const { return getBlock<0, 7, 0, 1>(y); }
    ColVector<3> getPosition() const { return getBlock<7, 10, 0, 1>(y); }
    ColVector<1> getAltitude() const { return getBlock<9, 10, 0, 1>(y); }

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
    void load(const std::filesystem::path &loadPath);

#pragma region Continuous Model.................................................

    /** 
     * @brief   Calculate the derivative of the state vector, given the current
     *          state x and the current input u.
     * 
     * @f$
     *  \boldsymbol{\dot{q}} = \frac{1}{2} \boldsymbol{q} \otimes 
     *      \begin{pmatrix} 0 \\ \vec{\omega} \end{pmatrix}
     * @f$  
     * 
     * @f$
     *  \dot{\vec{\omega}} = \Gamma_n \vec{n} + \Gamma_u \vec{u}
     *      - I^{-1} \left(\vec{\omega} \times I \vec{\omega} \right)
     * @f$
     * 
     * @f$
     *  \dot{\vec{n}} = k_2 \left(k_1 \vec{u} - \vec{n}\right)
     * @f$ 
     * 
     * @f$
     *  \dot{\vec{v}} = \boldsymbol{q} \star 
     *      \begin{pmatrix} 0 \\ 0 \\ 1 \end{pmatrix} 
     *      \frac{F_{z'}}{m} - 
     *      \begin{pmatrix} 0 \\ 0 \\ g \end{pmatrix}
     * \quad @f$  where @f$ \star @f$ is the quaternion rotation of a vector and 
     *      @f$ F_{z'} = C_t \rho Dp^4 n_t^2 N_m @f$ is the thrust along the 
     *      local z-axis
     * 
     * @f$
     *  \dot{\vec{x}} = \vec{v}
     * @f$
     * 
     * @f$
     *  \dot{n_t} = k_2 \left(k_1 u_t - n_t\right)
     * @f$
     * 
     * @param   x 
     *          The current state of the drone.
     * @param   u 
     *          The current control input u.
     * @return  The derivative of the state, x_dot.
     */
    VecX_t operator()(const VecX_t &x, const VecU_t &u) override;

    /** 
     * @brief   Get the sensor output of the drone model.
     */
    VecY_t getOutput(const VecX_t &x, const VecU_t &u) override;

    /**
     * @brief   Get the initial state of the drone. (Upright orientation and 
     *          hovering thrust.)
     */
    VecX_t getInitialState() const;

    /** 
     * @brief   Get the rotation in Euler angles, given a state vector x.
     */
    template <size_t R>
    static EulerAngles stateToEuler(const ColVector<R> &x) {
        Quaternion q = getBlock<0, 4, 0, 1>(x);
        return quat2eul(q);
    }

    /** 
     * @brief   Convert the vector of states to a vector of Euler angles.
     */
    template <size_t R>
    static std::vector<EulerAngles>
    statesToEuler(const std::vector<ColVector<R>> &xs) {
        std::vector<EulerAngles> orientation;
        orientation.resize(xs.size());
        transform(xs.begin(), xs.end(), orientation.begin(),
                  Drone::stateToEuler<R>);
        return orientation;
    }

    /** 
     * @brief   Extract a certain part from the given vector of states.
     */
    template <size_t R>
    static std::vector<ColVector<R>>
    extractState(const std::vector<VecX_t> &xs,
                 ColVector<R> (DroneState::*f)() const) {
        std::vector<ColVector<R>> result;
        result.resize(xs.size());
        transform(xs.begin(), xs.end(), result.begin(),
                  [f](const VecX_t &x) { return (DroneState{x}.*f)(); });
        return result;
    }

#pragma region Controllers......................................................

    /** 
     * @brief   TODO
     */
    Matrix<Nu_att, Nx_att - 1>
    getAttitudeControllerMatrixK(const Matrix<Nx_att - 1, Nx_att - 1> &Q,
                                 const Matrix<Nu_att, Nu_att> &R) const {
        return -dlqr(Ad_att_r, Bd_att_r, Q, R).K;
    }

    /** 
     * @brief   TODO
     */
    Attitude::LQRController
    getAttitudeController(const Matrix<Nx_att - 1, Nx_att - 1> &Q,
                          const Matrix<Nu_att, Nu_att> &R) const {
        auto K_red = getAttitudeControllerMatrixK(Q, R);
        return {Ad_att, Bd_att, Cd_att, Dd_att, K_red, Ts_att};
    }

    /** 
     * @brief   TODO
     */
    Attitude::ClampedLQRController
    getClampedAttitudeController(const ColVector<Nu_att> &clampMin,
                                 const ColVector<Nu_att> &clampMax,
                                 const Matrix<Nx_att - 1, Nx_att - 1> &Q,
                                 const Matrix<Nu_att, Nu_att> &R) const {
        return {getAttitudeController(Q, R), clampMin, clampMax};
    }

    Altitude::LQRController getAltitudeController(const Matrix<1, 3> &K_p,
                                                  const Matrix<1, 3> &K_i) {
        return {Ad_alt, Bd_alt, Cd_alt, Dd_alt, K_p, K_i, {uh}, Ts_alt};
    }

    class Controller : public DiscreteController<Nx, Nu, Ny> {
      public:
        Controller(const Attitude::ClampedLQRController &attCtrl,
                   const Altitude::LQRController &altCtrl)
            : DiscreteController<Nx, Nu, Ny>{attCtrl.Ts}, attCtrl{attCtrl},
              altCtrl{altCtrl}, subsampleAlt{
                                    size_t(round(altCtrl.Ts / attCtrl.Ts))} {
            assert(attCtrl.Ts < altCtrl.Ts);
        }

        VecU_t operator()(const VecX_t &x, const VecR_t &r) override;

      private:
        Attitude::ClampedLQRController attCtrl;
        Altitude::LQRController altCtrl;
        const size_t subsampleAlt;
        size_t subsampleCounter = 0;
        Altitude::LQRController::VecU_t u_alt;
    };

#pragma region Observers........................................................

    /** 
     * @brief   TODO
     */
    Matrix<Nx_att - 1, Ny_att - 1>
    getAttitudeObserverMatrixL(const RowVector<Nu_att> &varDynamics,
                               const RowVector<Ny_att - 1> &varSensors) const {
        return dlqe(Ad_att_r, Bd_att_r, Cd_att_r, diag(varDynamics),
                    diag(varSensors))
            .L;
    }

    /** 
     * @brief   TODO
     */
    Attitude::KalmanObserver
    getAttitudeObserver(const RowVector<Nu_att> &varDynamics,
                        const RowVector<Ny_att - 1> &varSensors) const {
        auto L_red = getAttitudeObserverMatrixL(varDynamics, varSensors);
        return {Ad_att_r, Bd_att_r, Cd_att, L_red, Ts_att};
    }

#pragma region System matrices Attitude.........................................

    /** 
     * ```
     *  Aa_att =  [  ·   ·   ·   ·   ·   ·   ·   ·   ·   ·  ]
     *            [  ·   ·   ·   ·  ┌─────────┐  ·   ·   ·  ]
     *            [  ·   ·   ·   ·  │  0.5 I3 │  ·   ·   ·  ]
     *            [  ·   ·   ·   ·  └─────────┘  ·   ·   ·  ]
     *            [  ·   ·   ·   ·   ·   ·   ·  ┌─────────┐ ]
     *            [  ·   ·   ·   ·   ·   ·   ·  │   Γ_n   │ ]
     *            [  ·   ·   ·   ·   ·   ·   ·  └─────────┘ ]
     *            [  ·   ·   ·   ·   ·   ·   ·  ┌─────────┐ ]
     *            [  ·   ·   ·   ·   ·   ·   ·  │ -k2 I3  │ ]
     *            [  ·   ·   ·   ·   ·   ·   ·  └─────────┘ ] 
     * ``` */
    Matrix<Nx_att, Nx_att> Aa_att = {};

    /** 
     * Discrete A 
     */
    Matrix<Nx_att, Nx_att> Ad_att = {};

    /** 
     * Discrete, reduced A 
     */
    Matrix<Nx_att - 1, Nx_att - 1> Ad_att_r = {};

    /** 
     * ```
     *  Ba_att =  [  ·   ·   ·  ]
     *            [  ·   ·   ·  ]
     *            [  ·   ·   ·  ]
     *            [  ·   ·   ·  ]
     *            [ ┌─────────┐ ]
     *            [ │   Γ_u   │ ]
     *            [ └─────────┘ ]
     *            [ ┌─────────┐ ]
     *            [ │ k1k2 I3 │ ]
     *            [ └─────────┘ ] 
     * ``` */
    Matrix<Nx_att, Nu_att> Ba_att = {};

    /** 
     * Discrete B
     */
    Matrix<Nx_att, Nu_att> Bd_att = {};

    /** 
     * Discrete, reduced B
     */
    Matrix<Nx_att - 1, Nu_att> Bd_att_r = {};

    /**
     * ```
     *  Ca_att =  [ 1  ·  ·  ·  ·  ·  ·  ·  ·  · ]
     *            [ ·  1  ·  ·  ·  ·  ·  ·  ·  · ]
     *            [ ·  ·  1  ·  ·  ·  ·  ·  ·  · ]
     *            [ ·  ·  ·  1  ·  ·  ·  ·  ·  · ]
     *            [ ·  ·  ·  ·  1  ·  ·  ·  ·  · ]
     *            [ ·  ·  ·  ·  ·  1  ·  ·  ·  · ]
     *            [ ·  ·  ·  ·  ·  ·  1  ·  ·  · ] 
     * ``` */
    Matrix<Ny_att, Nx_att> Ca_att = {};

    /**
     * Discrete C
     */
    Matrix<Ny_att, Nx_att> Cd_att = {};

    /**
     * Discrete, reduced C
     */
    Matrix<Ny_att - 1, Nx_att - 1> Cd_att_r = {};

    /** 
     * ```
     *  Da_att =  [ ·  ·  · ]
     *            [ ·  ·  · ]
     *            [ ·  ·  · ]
     *            [ ·  ·  · ]
     *            [ ·  ·  · ]
     *            [ ·  ·  · ]
     *            [ ·  ·  · ] 
     * ``` */
    Matrix<Ny_att, Nu_att> Da_att = {};

    /**
     * Discrete D
     */
    Matrix<Ny_att, Nu_att> Dd_att = {};

#pragma region System matrices Altitude.........................................

    /** 
     * ```
     *  Aa_alt =  [ -k2  ·   ·   ]
     *            [  ·   ·   1   ]
     *            [  a   ·   ·   ]
     * ``` */
    Matrix<Nx_alt, Nx_alt> Aa_alt = {};

    /** 
     * Discrete A 
     */
    Matrix<Nx_alt, Nx_alt> Ad_alt = {};

    /** 
     * ```
     *  Ba_alt =  [ k1 k2 ]
     *            [   ·   ]
     *            [   ·   ]
     * ``` */
    Matrix<Nx_alt, Nu_alt> Ba_alt = {};

    /** 
     * Discrete B
     */
    Matrix<Nx_alt, Nu_alt> Bd_alt = {};

    /**
     * ```
     *  Ca_alt =  [ ·  1  ·  ]
     * ``` */
    Matrix<Ny_alt, Nx_alt> Ca_alt = {};

    /**
     * Discrete C
     */
    Matrix<Ny_alt, Nx_alt> Cd_alt = {};

    /** 
     * ```
     *  Da_alt =  [ · ]
     * ``` */
    Matrix<Ny_alt, Nu_alt> Da_alt = {};

    /**
     * Discrete D
     */
    Matrix<Ny_alt, Nu_alt> Dd_alt = {};

#pragma region System parameters................................................

    double Ts_att;
    double Ts_alt;

    Matrix<3, 3> gamma_n;
    Matrix<3, 3> gamma_u;
    Matrix<3, 3> Id;
    Matrix<3, 3> Id_inv;
    double k1;
    double k2;

    const int Nm     = 4;      ///< -            number of motors
    const double g   = 9.81;   ///< m/s2         gravitational acceleration
    const double rho = 1.225;  ///< kg/m3        air density (nominal, at 15C,
                               ///               sea level)
    double m;                  ///< kg           total mass
    double ct;                 ///< -            thrust coefficient
    double Dp;                 ///< m            propeller diameter
    double nh;                 ///< rps          hover propeller speed
    double uh;                 ///< -            hover control
};
