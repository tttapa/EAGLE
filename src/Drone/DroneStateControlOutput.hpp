#pragma once

#include "Def.hpp"
#include <Matrix/Matrix.hpp>
#include <Quaternions/Quaternion.hpp>

class DroneState {
    ColVector<17> x = {1};

  public:
    DroneState() = default;
    DroneState(const ColVector<17> &x) : x{x} {}
    ColVector<Nx_att> getAttitude() const {
        return getBlock<0, Nx_att, 0, 1>(x);
    }
    Quaternion getOrientation() const { return getBlock<0, 4, 0, 1>(x); }
    EulerAngles getOrientationEuler() const {
        return quat2eul(getOrientation());
    }
    ColVector<3> getAngularVelocity() const { return getBlock<4, 7, 0, 1>(x); }
    ColVector<3> getMotorSpeed() const { return getBlock<7, 10, 0, 1>(x); }
    ColVector<3> getVelocity() const { return getBlock<10, 13, 0, 1>(x); }
    ColVector<3> getPosition() const { return getBlock<13, 16, 0, 1>(x); }
    ColVector<1> getThrustMotorSpeed() const { return {x[16]}; }
    ColVector<3> getAltitude() const {
        return vcat(getBlock<16, 17, 0, 1>(x),  // n
                    getBlock<15, 16, 0, 1>(x),  // z
                    getBlock<12, 13, 0, 1>(x)   // v_z
        );
    }

    void setAttitude(const ColVector<Nx_att> &a) {
        assignBlock<0, Nx_att, 0, 1>(x) = a;
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
    void setAltitude(const ColVector<3> &a) {
        x[16] = a[0];  // n
        x[15] = a[1];  // z
        x[12] = a[3];  // v_z
    }
    operator ColVector<17>() const { return x; }
};

class DroneAttitudeState {
    ColVector<10> x = {1};

  public:
    DroneAttitudeState() = default;
    DroneAttitudeState(const ColVector<10> &x) : x{x} {}

    Quaternion getOrientation() const { return getBlock<0, 4, 0, 1>(x); }
    EulerAngles getOrientationEuler() const {
        return quat2eul(getOrientation());
    }
    ColVector<3> getAngularVelocity() const { return getBlock<4, 7, 0, 1>(x); }
    ColVector<3> getMotorSpeed() const { return getBlock<7, 10, 0, 1>(x); }

    void setOrientation(const Quaternion &q) { assignBlock<0, 4, 0, 1>(x) = q; }
    void setAngularVelocity(const ColVector<3> &w) {
        assignBlock<4, 7, 0, 1>(x) = w;
    }
    void setMotorSpeed(const ColVector<3> &n) {
        assignBlock<7, 10, 0, 1>(x) = n;
    }
    operator ColVector<10>() const { return x; }
};

class DroneControl {
    ColVector<4> u = {};

  public:
    DroneControl() = default;
    DroneControl(const ColVector<4> &u) : u{u} {}
    DroneControl(const ColVector<3> &u_att, const ColVector<1> &u_alt = {})
        : u{vcat(u_att, u_alt)} {}
    ColVector<3> getAttitudeControl() const { return getBlock<0, 3, 0, 1>(u); }
    ColVector<1> getThrustControl() const { return {u[3]}; }
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
                const ColVector<Ny_nav + Ny_alt> &y_pos = {})
        : y{vcat(y_att, y_pos)} {}
    Quaternion getOrientation() const { return getBlock<0, 4, 0, 1>(y); }
    EulerAngles getOrientationEuler() const {
        return quat2eul(getOrientation());
    }
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