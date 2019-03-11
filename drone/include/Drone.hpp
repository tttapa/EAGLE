#pragma once

#include "DroneStateControlOutput.hpp"

#include "KalmanObserver.hpp"
#include "LQRController.hpp"

#include <Model.hpp>

#include <DLQE.hpp>
#include <DLQR.hpp>

#include <AlmostEqual.hpp>
#include <PerfTimer.hpp>

#include <filesystem>
#include <iostream>

#include "C-code-wrappers/CKalmanObserver.hpp"
#include "C-code-wrappers/CLQRController.hpp"

#include "DroneParamsAndMatrices.hpp"

struct Drone : public ContinuousModel<Nx, Nu, Ny> {

#pragma region Constructors.....................................................

    /// Create a drone, loading the parameters from a given path.
    Drone(const std::filesystem::path &loadPath) { p.load(loadPath); }
    Drone(const DroneParamsAndMatrices &p) : p{p} {}
    Drone() = default;

#pragma region Continuous Model.................................................

    /** 
     * @brief   Calculate the derivative of the state vector, given the current
     *          state x and the current input u.
     * 
     * @f$
     *  \boldsymbol{\dot{q}} = \frac{1}{2} \boldsymbol{q} \otimes 
     *      \begin{pmatrix} 0 \\ \ve    c{\omega} \end{pmatrix}
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
     * 
     * @f$
     *   \vec{y} = C \vec{x} + D \vec{u}
     * @f$
     */
    VecY_t getOutput(const VecX_t &x, const VecU_t &u) override;

    /**
     * @brief   Get the initial state of the drone. (Upright orientation and 
     *          hovering thrust.)
     */
    DroneState getStableState() const;

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

    /** 
     * @brief   Extract a specific element from the given vector of signals 
     *          (states, inputs or outputs).
     */
    template <size_t RP>
    static std::vector<typename ColVector<RP>::type::type>
    extractSignal(const std::vector<ColVector<RP>> &xs, size_t idx) {
        assert(idx < RP);
        std::vector<typename ColVector<RP>::type::type> result;
        result.resize(xs.size());
        transform(xs.begin(), xs.end(), result.begin(),
                  [idx](const ColVector<RP> &x) { return x[idx]; });
        return result;
    }

    /// The continuous model of the attitude of the drone
    struct AttitudeModel : public ContinuousModel<Nx_att, Nu_att, Ny_att> {
        AttitudeModel(const DroneParamsAndMatrices &drone);

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

    /// Get the continuous model of the attitude of this drone
    AttitudeModel getAttitudeModel() const { return {p}; }

    /// The continuous model of the altitude controller of the drone
    /// @todo   Implement
    struct AltitudeModel : public ContinuousModel<Nx_alt, Nu_alt, Ny_alt> {};

#pragma region Controllers......................................................

    /** 
     * @brief   Calculate the proportional LQR matrix for the attitude 
     *          controller, given the weight matrices Q and R.
     */
    Matrix<Nu_att, Nx_att - 1>
    getAttitudeControllerMatrixK(const Matrix<Nx_att - 1, Nx_att - 1> &Q,
                                 const Matrix<Nu_att, Nu_att> &R) const {
        return -dlqr(p.Ad_att_r, p.Bd_att_r, Q, R).K;
    }

    /** 
     * @brief   Get the attitude controller given the proportional LQR matrix.
     * 
     * @note    This controller doesn't clamp the control output!  
     *          Use FixedClampAttitudeController to wrap it.
     */
    Attitude::LQRController
    getAttitudeController(const Matrix<Nu_att, Nx_att - 1> &K_red) const {
        return {p.G_att, K_red, p.Ts_att};
    }

    /** 
     * @brief   Get the attitude controller given the LQR weight matrices Q and 
     *          R.
     * 
     * @note    This controller doesn't clamp the control output!  
     *          Use FixedClampAttitudeController to wrap it.
     */
    Attitude::LQRController
    getAttitudeController(const Matrix<Nx_att - 1, Nx_att - 1> &Q,
                          const Matrix<Nu_att, Nu_att> &R) const {
        auto K_red = getAttitudeControllerMatrixK(Q, R);
        return {p.G_att, K_red, p.Ts_att};
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

    /** 
     * A stand-alone attitude controller that clamps the control output using
     * the (fixed) hover thrust as the common thrust.
     */
    class FixedClampCAttitudeController : public Attitude::CLQRController {
      public:
        FixedClampCAttitudeController(const Attitude::CLQRController &ctrl,
                                      double uh)
            : Attitude::CLQRController{ctrl}, uh{uh} {}
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
        return {getAttitudeController(Q, R), p.uh};
    }

    FixedClampAttitudeController getFixedClampAttitudeController(
        const Matrix<Nu_att, Nx_att - 1> &K_red) const {
        return {getAttitudeController(K_red), p.uh};
    }

    FixedClampCAttitudeController getFixedClampCAttitudeController() const {
        return {Attitude::CLQRController{p.Ts_att}, p.uh};
    }

    // Altitude

    Altitude::LQRController getAltitudeController(const Matrix<1, 4> &K_pi,
                                                  double maxIntegralInfluence) {
        return {p.G_alt, p.Cd_alt, K_pi, p.Ts_alt, maxIntegralInfluence};
    }

    Altitude::LQRController getAltitudeController(const Matrix<3, 3> &Q,
                                                  const Matrix<1, 1> &K_i,
                                                  double maxIntegralInfluence) {
        auto K_lqr = dlqr(p.Ad_alt, p.Bd_alt, Q, {{1}}).K;
        auto K_pi  = hcat(K_lqr, K_i);
        return {p.G_alt, p.Cd_alt, K_pi, p.Ts_alt, maxIntegralInfluence};
    }

    Altitude::CLQRController getCAltitudeController() { return {p.Ts_alt}; }

    class Controller : public DiscreteController<Nx, Nu, Ny> {
      public:
        using p_attitude_controller_t =
            std::unique_ptr<DiscreteController<Nx_att, Nu_att, Ny_att>>;
        using p_altitude_controller_t =
            std::unique_ptr<DiscreteController<Nx_alt, Nu_alt, Ny_alt>>;

        Controller(p_attitude_controller_t attitudeController,
                   p_altitude_controller_t altitudeController, double uh)
            : DiscreteController<Nx, Nu, Ny>{attitudeController->Ts},
              attitudeController{std::move(attitudeController)},
              altitudeController{std::move(altitudeController)},
              subsampleAlt{size_t(round(this->altitudeController->Ts /
                                        this->attitudeController->Ts))},
              uh{uh} {
            assert(this->attitudeController->Ts < this->altitudeController->Ts);
        }

        /** Get the controller output */
        VecU_t operator()(const VecX_t &x, const VecR_t &r) override;

        /** Reset the internal states of the controllers */
        void reset() override;

        /** Clamp the marginal thrust between 0.1 and -0.1 */
        static Altitude::LQRController::VecU_t
        clampThrust(Altitude::LQRController::VecU_t u_thrust);

        /** 2.2: Clamp [ux;uy;uz] such that all motor inputs ui <= 1. */
        static Attitude::LQRController::VecU_t
        clampAttitude(const Attitude::LQRController::VecU_t &u_raw,
                      const Altitude::LQRController::VecU_t &u_alt);

      private:
        p_attitude_controller_t attitudeController;
        p_altitude_controller_t altitudeController;
        const size_t subsampleAlt;
        size_t subsampleCounter = 0;
        Altitude::LQRController::VecU_t u_alt;
        const Altitude::LQRController::VecU_t uh;
    };

    Controller getController(const Matrix<Nx_att - 1, Nx_att - 1> &Q_att,
                             const Matrix<Nu_att, Nu_att> &R_att,
                             const Matrix<1, 4> &K_pi_alt,
                             double maxIntegralInfluence) {
        return {
            std::make_unique<Attitude::LQRController>(
                getAttitudeController(Q_att, R_att)),
            std::make_unique<Altitude::LQRController>(
                getAltitudeController(K_pi_alt, maxIntegralInfluence)),
            p.uh,
        };
    }

    Controller getController(const Matrix<Nx_att - 1, Nx_att - 1> &Q_att,
                             const Matrix<Nu_att, Nu_att> &R_att,
                             const Matrix<3, 3> &Q_alt,
                             const Matrix<1, 1> &K_i_alt,
                             double maxIntegralInfluence) {
        return {
            std::make_unique<Attitude::LQRController>(
                getAttitudeController(Q_att, R_att)),
            std::make_unique<Altitude::LQRController>(
                getAltitudeController(Q_alt, K_i_alt, maxIntegralInfluence)),
            p.uh,
        };
    }

    Controller getCController(int config = 1, bool enableIntegral = true) {
        return {
            std::make_unique<Attitude::CLQRController>(p.Ts_att, config),
            std::make_unique<Altitude::CLQRController>(p.Ts_alt, config,
                                                       enableIntegral),
            p.uh,
        };
    }

#pragma region Observers........................................................

    /** 
     * @brief   TODO
     */
    Matrix<Nx_att - 1, Ny_att - 1>
    getAttitudeObserverMatrixL(const RowVector<Nu_att> &varDynamics,
                               const RowVector<Ny_att> &varSensors) const {
        RowVector<Ny_att - 1> varSensors_r =
            getBlock<0, 1, 1, Ny_att>(varSensors);
        return dlqe(p.Ad_att_r, p.Bd_att_r, p.Cd_att_r, diag(varDynamics),
                    diag(varSensors_r))
            .L;
    }

    /** 
     * @brief   TODO
     */
    Attitude::KalmanObserver
    getAttitudeObserver(const RowVector<Nu_att> &varDynamics,
                        const RowVector<Ny_att> &varSensors) const {
        auto L_red = getAttitudeObserverMatrixL(varDynamics, varSensors);
        return {p.Ad_att_r, p.Bd_att_r, p.Cd_att, L_red, p.Ts_att};
    }

    /** 
     * @brief   TODO
     */
    Matrix<Nx_att - 1, Ny_att - 1> getAttitudeObserverMatrixL(
        const RowVector<Nu_att + Nx_att - 1> &varDynamics,
        const RowVector<Ny_att - 1> &varSensors) const {
        return dlqe(p.Ad_att_r, hcat(eye<9>(), p.Bd_att_r), p.Cd_att_r,
                    diag(varDynamics), diag(varSensors))
            .L;
    }

    /** 
     * @brief   TODO
     */
    Attitude::KalmanObserver
    getAttitudeObserver(const RowVector<Nu_att + Nx_att - 1> &varDynamics,
                        const RowVector<Ny_att - 1> &varSensors) const {
        auto L_red = getAttitudeObserverMatrixL(varDynamics, varSensors);
        return {p.Ad_att_r, p.Bd_att_r, p.Cd_att, L_red, p.Ts_att};
    }

    /** 
     * @brief   TODO
     */
    Attitude::KalmanObserver
    getAttitudeObserver(const Matrix<Nx_att - 1, Ny_att - 1> &L_red) const {
        return {p.Ad_att_r, p.Bd_att_r, p.Cd_att, L_red, p.Ts_att};
    }

    /** 
     * @brief   TODO
     */
    Matrix<Nx_alt, Ny_alt>
    getAltitudeObserverMatrixL(const RowVector<Nu_alt> &varDynamics,
                               const RowVector<Ny_alt> &varSensors) const {
        return dlqe(p.Ad_alt, p.Bd_alt, p.Cd_alt, diag(varDynamics),
                    diag(varSensors))
            .L;
    }

    /** 
     * @brief   TODO
     * 
     * @param   varDynamics
     *          [ varDynX, varDynU ]
     * @param   varSensors
     */
    Matrix<Nx_alt, Ny_alt>
    getAltitudeObserverMatrixL(const RowVector<Nx_alt + Nu_alt> &varDynamics,
                               const RowVector<Ny_alt> &varSensors) const {
        return dlqe(p.Ad_alt, hcat(eye<Nx_alt>(), p.Bd_alt), p.Cd_alt,
                    diag(varDynamics), diag(varSensors))
            .L;
    }

    /** 
     * @brief   TODO
     */
    Altitude::KalmanObserver
    getAltitudeObserver(const RowVector<Nu_alt> &varDynamics,
                        const RowVector<Ny_alt> &varSensors) const {
        auto L = getAltitudeObserverMatrixL(varDynamics, varSensors);
        return {p.Ad_alt, p.Bd_alt, p.Cd_alt, L, p.Ts_alt};
    }

    /** 
     * @brief   TODO
     */
    Altitude::KalmanObserver
    getAltitudeObserver(const RowVector<Nx_alt + Nu_alt> &varDynamics,
                        const RowVector<Ny_alt> &varSensors) const {
        auto L = getAltitudeObserverMatrixL(varDynamics, varSensors);
        return {p.Ad_alt, p.Bd_alt, p.Cd_alt, L, p.Ts_alt};
    }
    
    /** 
     * @brief   TODO
     */
    Altitude::KalmanObserver
    getAltitudeObserver(const Matrix<Nx_alt, Ny_alt> &L) const {
        return {p.Ad_alt, p.Bd_alt, p.Cd_alt, L, p.Ts_alt};
    }

    class Observer : public DiscreteObserver<Nx, Nu, Ny> {
      public:
        using p_attitude_observer_t =
            std::unique_ptr<DiscreteObserver<Nx_att, Nu_att, Ny_att>>;
        using p_altitude_observer_t =
            std::unique_ptr<DiscreteObserver<Nx_alt, Nu_alt, Ny_alt>>;

        Observer(p_attitude_observer_t attitudeObserver,
                 p_altitude_observer_t altitudeObserver)
            : DiscreteObserver<Nx, Nu, Ny>{attitudeObserver->Ts},
              attitudeObserver{std::move(attitudeObserver)},
              altitudeObserver{std::move(altitudeObserver)},
              subsampleAlt{size_t(round(this->altitudeObserver->Ts /
                                        this->attitudeObserver->Ts))} {
            assert(this->attitudeObserver->Ts < this->altitudeObserver->Ts);
        }

        void reset() override {
            subsampleCounter = 0;
            attitudeObserver->reset();
            altitudeObserver->reset();
        }

        VecX_t getStateChange(const VecX_t &x_hat, const VecY_t &y_sensor,
                              const VecU_t &u) override;

      private:
        p_attitude_observer_t attitudeObserver;
        p_altitude_observer_t altitudeObserver;
        const size_t subsampleAlt;
        size_t subsampleCounter = 0;
    };

    Observer getObserver(const RowVector<Nu_att> &varDynamics_att,
                         const RowVector<Ny_att> &varSensors_att,
                         const RowVector<Nu_alt> &varDynamics_alt,
                         const RowVector<Ny_alt> &varSensors_alt) {
        return {std::make_unique<Attitude::KalmanObserver>(
                    getAttitudeObserver(varDynamics_att, varSensors_att)),
                std::make_unique<Altitude::KalmanObserver>(
                    getAltitudeObserver(varDynamics_alt, varSensors_alt))};
    }

    Observer getCObserver() {
        return {std::make_unique<Attitude::CKalmanObserver>(p.Ts_att),
                std::make_unique<Altitude::CKalmanObserver>(p.Ts_alt)};
    }

    // private: TODO
    DroneParamsAndMatrices p;
};
