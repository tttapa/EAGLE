#pragma once

#include "DroneStateControlOutput.hpp"

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
     *      @f$ F_{z'} = C_t \rho D_p^4 n_t^2 N_m @f$ is the thrust along the 
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
    VecX_t getStableState() const;

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
     * @brief   Extract a specific part from the given vector of states.
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

    /** 
     * @brief   Extract a specific element from the given vector of signals 
     *          (states, inputs or outputs).
     */
    template <class S, size_t RS, size_t RP>
    static std::vector<typename ColVector<RP>::type::type>
    extractSignal(const std::vector<ColVector<RS>> &xs,
                  ColVector<RP> (S::*f)() const, size_t idx) {
        assert(idx < RP);
        std::vector<typename ColVector<RP>::type::type> result;
        result.resize(xs.size());
        transform(
            xs.begin(), xs.end(), result.begin(),
            [f, idx](const ColVector<RS> &x) { return (S(x).*f)()[idx]; });
        return result;
    }

    struct AttitudeModel : public ContinuousModel<Nx_att, Nu_att, Ny_att> {
        AttitudeModel(const Drone &drone);

        VecX_t operator()(const VecX_t &x, const VecU_t &u) override;
        VecY_t getOutput(const VecX_t &x, const VecU_t &u) override;

        const Matrix<3, 3> gamma_n;
        const Matrix<3, 3> gamma_u;
        const Matrix<3, 3> Id;
        const Matrix<3, 3> Id_inv;
        const double k1;
        const double k2;
        const ColVector<1> uh;
        const Matrix<Ny_att, Nx_att> Ca_att;
        const Matrix<Ny_att, Nu_att> Da_att;
    };

    AttitudeModel getAttitudeModel() const { return {*this}; }

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
    getAttitudeController(const Matrix<Nu_att, Nx_att - 1> &K_red) const {
        return {G_att, K_red, Ts_att};
    }

    /** 
     * @brief   TODO
     */
    Attitude::LQRController
    getAttitudeController(const Matrix<Nx_att - 1, Nx_att - 1> &Q,
                          const Matrix<Nu_att, Nu_att> &R) const {
        auto K_red = getAttitudeControllerMatrixK(Q, R);
        return {G_att, K_red, Ts_att};
    }

    /** 
     * A stand-alone attitude controller that clamps the control output using
     * the (fixed) hover thrust as the common thrust.
     */
    class FixedClampAttitudeController : public Attitude::LQRController {
      public:
        FixedClampAttitudeController(const Attitude::LQRController &ctrl,
                                     double uh)
            : Attitude::LQRController{ctrl}, uh{uh} {}
        VecU_t operator()(const VecX_t &x, const VecR_t &r) override {
            return Drone::Controller::clampAttitude(
                getRawControllerOutput(x, r), uh);
        }

      private:
        const ColVector<1> uh;
    };

    FixedClampAttitudeController
    getFixedClampAttitudeController(const Matrix<Nx_att - 1, Nx_att - 1> &Q,
                                    const Matrix<Nu_att, Nu_att> &R) const {
        return {getAttitudeController(Q, R), uh};
    }

    FixedClampAttitudeController getFixedClampAttitudeController(
        const Matrix<Nu_att, Nx_att - 1> &K_red) const {
        return {getAttitudeController(K_red), uh};
    }

    Altitude::LQRController getAltitudeController(const Matrix<1, 3> &K_p,
                                                  const Matrix<1, 3> &K_i) {
        return {G_alt, K_p, K_i, Ts_alt};
    }

    class Controller : public DiscreteController<Nx, Nu, Ny> {
      public:
        Controller(const Attitude::LQRController &attCtrl,
                   const Altitude::LQRController &altCtrl, double uh)
            : DiscreteController<Nx, Nu, Ny>{attCtrl.Ts}, attCtrl{attCtrl},
              altCtrl{altCtrl},
              subsampleAlt{size_t(round(altCtrl.Ts / attCtrl.Ts))}, uh{uh} {
            assert(attCtrl.Ts < altCtrl.Ts);
        }

        VecU_t operator()(const VecX_t &x, const VecR_t &r) override;

        /** Clamp the marginal thrust between 0.1 and -0.1 */
        static Altitude::LQRController::VecU_t
        clampThrust(Altitude::LQRController::VecU_t u_thrust);
        /** 2.2: Clamp [ux;uy;uz] such that all motor inputs ui <= 1. */
        static Attitude::LQRController::VecU_t
        clampAttitude(const Attitude::LQRController::VecU_t &u_raw,
                      const Altitude::LQRController::VecU_t &u_alt);

      private:
        Attitude::LQRController attCtrl;
        Altitude::LQRController altCtrl;
        const size_t subsampleAlt;
        size_t subsampleCounter = 0;
        Altitude::LQRController::VecU_t u_alt;
        const Altitude::LQRController::VecU_t uh;
    };

    Controller getController(const Matrix<Nx_att - 1, Nx_att - 1> &Q_att,
                             const Matrix<Nu_att, Nu_att> &R_att,
                             const Matrix<1, 3> &K_p_alt,
                             const Matrix<1, 3> &K_i_alt) {
        return {getAttitudeController(Q_att, R_att),
                getAltitudeController(K_p_alt, K_i_alt), uh};
    }

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

    /** 
     * @brief   TODO
     */
    Matrix<Nx_alt, Ny_alt>
    getAltitudeObserverMatrixL(const RowVector<Nu_alt> &varDynamics,
                               const RowVector<Ny_alt> &varSensors) const {
        return dlqe(Ad_alt, Bd_alt, Cd_alt, diag(varDynamics), diag(varSensors))
            .L;
    }

    /** 
     * @brief   TODO
     */
    Altitude::KalmanObserver
    getAltitudeObserver(const RowVector<Nu_alt> &varDynamics,
                        const RowVector<Ny_alt> &varSensors) const {
        auto L = getAltitudeObserverMatrixL(varDynamics, varSensors);
        return {Ad_alt, Bd_alt, Cd_alt, L, Ts_alt};
    }

    class Observer : public DiscreteObserver<Nx, Nu, Ny> {
      public:
        Observer(const Attitude::KalmanObserver &attObsv,
                 const Altitude::KalmanObserver &altObsv)
            : DiscreteObserver<Nx, Nu, Ny>{attObsv.Ts}, attObsv{attObsv},
              altObsv{altObsv}, subsampleAlt{
                                    size_t(round(altObsv.Ts / attObsv.Ts))} {
            assert(attObsv.Ts < altObsv.Ts);
        }

        VecX_t getStateChange(const VecX_t &x_hat, const VecY_t &y_sensor,
                              const VecU_t &u) override;

      private:
        Attitude::KalmanObserver attObsv;
        Altitude::KalmanObserver altObsv;
        const size_t subsampleAlt;
        size_t subsampleCounter = 0;
    };

    Observer getObserver(const RowVector<Nu_att> &varDynamics_att,
                         const RowVector<Ny_att - 1> &varSensors_att,
                         const RowVector<Nu_alt> &varDynamics_alt,
                         const RowVector<Ny_alt> &varSensors_alt) {
        return {getAttitudeObserver(varDynamics_att, varSensors_att),
                getAltitudeObserver(varDynamics_alt, varSensors_alt)};
    }

#pragma region System matrices Attitude.........................................
  private:
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

    /** 
     * Equilibrium G
     */
    Matrix<Nx_att + Nu_att, Ny_att> G_att;

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

    /** 
     * Equilibrium G
     */
    Matrix<Nx_alt + Nu_alt, Ny_alt> G_alt;

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
