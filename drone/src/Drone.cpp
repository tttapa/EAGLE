#include "Drone.hpp"
#include "MotorControl.hpp"

using namespace std;

Drone::VecX_t Drone::operator()(const VecX_t &x, const VecU_t &u) {
    // Convert the state and input vectors to types with getters and setters
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
    x_dot.setAngularVelocity(p.gamma_n * n + p.gamma_u * u_att -
                             p.Id_inv * cross(omega, p.Id * omega));
    x_dot.setMotorSpeed(p.k2 * (p.k1 * u_att - n));

    // Navigation/Altitude
    ColVector<3> v  = xx.getVelocity();
    double n_thrust = xx.getThrustMotorSpeed();
    double u_thrust = uu.getThrustControl();

    double F_local_z = p.ct * p.rho * sq(sq(p.Dp)) * p.Nm *
                       (sq(n_thrust + p.nh) + sq(n[0]) + sq(n[1]) + sq(n[2]));
    ColVector<3> F_local = {0, 0, F_local_z};
    ColVector<3> F_world = quatrotate(quatconjugate(q), F_local);
    ColVector<3> a_world = F_world / p.m;
    ColVector<3> a_grav  = {0, 0, -p.g};
    ColVector<3> a       = a_world + a_grav;

    x_dot.setVelocity(a);
    x_dot.setPosition(v);
    x_dot.setThrustMotorSpeed(p.k2 * (p.k1 * (u_thrust - 0) - n_thrust));

    return x_dot;
}

Drone::VecY_t Drone::getOutput(const VecX_t &x, const VecU_t &u) {
    DroneState xx   = {x};
    DroneControl uu = {u};
    ColVector<Ny_att> y_att =
        p.Ca_att * xx.getAttitude() + p.Da_att * uu.getAttitudeControl();
    ColVector<Ny_nav + Ny_alt> y_nav = xx.getPosition();
    return DroneOutput{y_att, y_nav};
}

DroneState Drone::getStableState() const {
    DroneState xx = {};
    xx.setOrientation(eul2quat({0, 0, 0}));
    xx.setAngularVelocity({0, 0, 0});
    xx.setMotorSpeed({0, 0, 0});
    xx.setVelocity({0, 0, 0});
    xx.setPosition({0, 0, 0});
    xx.setThrustMotorSpeed(0);
    return xx;
}

Drone::AttitudeModel::AttitudeModel(const DroneParamsAndMatrices &drone)
    : gamma_n{drone.gamma_n}, gamma_u{drone.gamma_u}, Id{drone.Id},
      Id_inv{drone.Id_inv}, k1{drone.k1}, k2{drone.k2}, uh{drone.uh},
      Ca_att{drone.Ca_att}, Da_att{drone.Da_att} {}

// TODO: dry?
Drone::AttitudeModel::VecX_t Drone::AttitudeModel::operator()(const VecX_t &x,
                                                              const VecU_t &u) {
    // Convert the state and input vectors to types with getters and setters
    DroneAttitudeState xx = {x};
    DroneControl uu       = {u, {uh}};
    DroneAttitudeState x_dot;

    Quaternion q       = xx.getOrientation();
    ColVector<3> omega = xx.getAngularVelocity();
    ColVector<3> n     = xx.getMotorSpeed();
    ColVector<3> u_att = uu.getAttitudeControl();

    Quaternion q_omega = vcat(zeros<1, 1>(), omega);
    x_dot.setOrientation(0.5 * quatmultiply(q, q_omega));
    x_dot.setAngularVelocity(gamma_n * n + gamma_u * u_att -
                             Id_inv * cross(omega, Id * omega));
    x_dot.setMotorSpeed(k2 * (k1 * u_att - n));

    return x_dot;
}

// TODO: dry?
Drone::AttitudeModel::VecY_t Drone::AttitudeModel::getOutput(const VecX_t &x,
                                                             const VecU_t &u) {
    return Ca_att * x + Da_att * u;
}

#pragma region Controllers......................................................

Drone::Controller::VecU_t Drone::Controller::
operator()(const Drone::Controller::VecX_t &x,
           const Drone::Controller::VecR_t &r) {
    const DroneState xx     = {x};
    const DroneReference rr = {r};
    DroneControl uu;

    if (subsampleCounter == 0) {
        auto u_alt_raw =
            altitudeController->getOutput(xx.getAltitude(), rr.getAltitude());
        u_alt            = clampThrust(u_alt_raw);
        subsampleCounter = subsampleAlt;
    }
    --subsampleCounter;

    auto u_att =
        attitudeController->getOutput(xx.getAttitude(), rr.getAttitude());
    u_att = clampAttitude(u_att, u_alt + uh);
    uu.setAttitudeControl(u_att);
    uu.setThrustControl(u_alt);
    checkControlSignal(uu, uh);
    return uu;
}

void Drone::Controller::reset() {
    subsampleCounter = 0;
    attitudeController->reset();
    altitudeController->reset();
}

ColVector<1> Drone::Controller::clampThrust(ColVector<1> u_thrust) {
    clamp(u_thrust, {-0.1}, {0.1});
    return u_thrust;
}

Attitude::LQRController::VecU_t
Drone::Controller::clampAttitude(const Attitude::LQRController::VecU_t &u_raw,
                                 const Altitude::LQRController::VecU_t &u_alt) {
    ColVector<3> u      = u_raw;
    double other_max    = 1 - abs(u_alt);
    double other_actual = abs(u_raw[0]) + abs(u_raw[1]) + abs(u_raw[2]);
    if (other_actual > other_max) {
        u[0] = {other_max / other_actual * u_raw[0]};
        u[1] = {other_max / other_actual * u_raw[1]};
        u[2] = {other_max / other_actual * u_raw[2]};
    }
    other_actual = abs(u[0]) + abs(u[1]) + abs(u[2]);
    double e     = numeric_limits<double>::epsilon() + 1.0;
    if (other_actual > abs(u_alt)) {
        u[0] = {double(u_alt) / other_actual * u[0] / e};
        u[1] = {double(u_alt) / other_actual * u[1] / e};
        u[2] = {double(u_alt) / other_actual * u[2] / e};
    }
    return u;
}

#pragma region Observers........................................................

Drone::Observer::VecX_t Drone::Observer::getStateChange(const VecX_t &x_hat,
                                                        const VecY_t &y_sensor,
                                                        const VecU_t &u) {
    DroneState xx_hat = {x_hat};
    DroneOutput yy    = {y_sensor};
    DroneControl uu   = {u};

    if (subsampleCounter == 0) {
        auto new_x_alt = altitudeObserver->getStateChange(
            xx_hat.getAltitude(), yy.getAltitude(), uu.getThrustControl());
        xx_hat.setAltitude(new_x_alt);
        subsampleCounter = subsampleAlt;
    }
    --subsampleCounter;

    xx_hat.setAttitude(attitudeObserver->getStateChange(
        xx_hat.getAttitude(), yy.getAttitude(), uu.getAttitudeControl()));
    return xx_hat;
}