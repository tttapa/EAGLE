#pragma once

#include "Def.hpp"
#include <Matrix.hpp>
#include <Quaternion.hpp>

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
    ColVector<2> getLocation() const { return getBlock<13, 15, 0, 1>(x); }
    ColVector<1> getThrustMotorSpeed() const { return {x[16]}; }
    ColVector<3> getAltitude() const {
        return vcat(getBlock<16, 17, 0, 1>(x),  // n
                    getBlock<15, 16, 0, 1>(x),  // z
                    getBlock<12, 13, 0, 1>(x)   // v_z
        );
    }
    ColVector<6> getNavigation() const {
        return vcat(getBlock<1, 3, 0, 1>(x),    // q1, q2
                    getBlock<13, 15, 0, 1>(x),  // x, y
                    getBlock<10, 12, 0, 1>(x)   // vx, vy
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
    void setPosition(const ColVector<3> &xyz) {
        assignBlock<13, 16, 0, 1>(x) = xyz;
    }
    void setLocation(const ColVector<2> &xy) {
        assignBlock<13, 15, 0, 1>(x) = xy;
    }
    void setThrustMotorSpeed(double t) { x[16] = {t}; }
    void setAltitude(const ColVector<3> &a) {
        x[16] = a[0];  // n
        x[15] = a[1];  // z
        x[12] = a[2];  // v_z
    }
    void setHeight(double h) {
        x[15] = h;
    }
    void setNavigation(const ColVector<6> &n) {
        // q1 and q2 ignored
        assignBlock<13, 15, 0, 1>(x) = getBlock<2, 4, 0, 1>(n);
        assignBlock<10, 12, 0, 1>(x) = getBlock<4, 6, 0, 1>(n);
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
    constexpr DroneControl() = default;
    constexpr DroneControl(const ColVector<4> &u) : u{u} {}
    constexpr DroneControl(const ColVector<3> &u_att,
                           const ColVector<1> &u_alt = {})
        : u{vcat(u_att, u_alt)} {}
    constexpr ColVector<3> getAttitudeControl() const {
        return getBlock<0, 3, 0, 1>(u);
    }
    constexpr ColVector<1> getThrustControl() const { return {u[3]}; }
    constexpr void setAttitudeControl(const ColVector<3> &u_att) {
        assignBlock<0, 3, 0, 1>(u) = u_att;
    }
    constexpr void setThrustControl(double u_thrust) { u[3] = {u_thrust}; }
    constexpr operator ColVector<4>() const { return u; }
};

class DroneOutput {
    ColVector<10> y = {};

  public:
    constexpr DroneOutput() = default;
    constexpr DroneOutput(const ColVector<10> &y) : y{y} {}
    constexpr DroneOutput(const ColVector<Ny_att> &y_att,
                          const ColVector<Ny_nav + Ny_alt> &y_pos = {})
        : y{vcat(y_att, y_pos)} {}
    constexpr Quaternion getOrientation() const {
        return getBlock<0, 4, 0, 1>(y);
    }
    constexpr EulerAngles getOrientationEuler() const {
        return quat2eul(getOrientation());
    }
    constexpr ColVector<3> getAngularVelocity() const {
        return getBlock<4, 7, 0, 1>(y);
    }
    constexpr ColVector<7> getAttitude() const {
        return getBlock<0, 7, 0, 1>(y);
    }
    constexpr ColVector<3> getPosition() const {
        return getBlock<7, 10, 0, 1>(y);
    }
    constexpr ColVector<1> getAltitude() const {
        return getBlock<9, 10, 0, 1>(y);
    }

    constexpr void setOrientation(const Quaternion &q) {
        assignBlock<0, 4, 0, 1>(y) = q;
    }
    constexpr void setAngularVelocity(const ColVector<3> &w) {
        assignBlock<4, 7, 0, 1>(y) = w;
    }
    constexpr void setAttitudeOutput(const ColVector<7> &yy) {
        assignBlock<0, 7, 0, 1>(y) = yy;
    }
    constexpr void setPosition(const ColVector<3> &z) {
        assignBlock<7, 10, 0, 1>(y) = z;
    }
    constexpr operator ColVector<10>() const { return y; }
};

using DroneReference = DroneOutput;

class DroneAttitudeOutput {
    ColVector<7> y = {};

  public:
    constexpr DroneAttitudeOutput() = default;
    constexpr DroneAttitudeOutput(const ColVector<7> &y) : y{y} {}
    constexpr Quaternion getOrientation() const {
        return getBlock<0, 4, 0, 1>(y);
    }
    constexpr EulerAngles getOrientationEuler() const {
        return quat2eul(getOrientation());
    }
    constexpr ColVector<3> getAngularVelocity() const {
        return getBlock<4, 7, 0, 1>(y);
    }

    constexpr void setOrientation(const Quaternion &q) {
        assignBlock<0, 4, 0, 1>(y) = q;
    }
    constexpr void setAngularVelocity(const ColVector<3> &w) {
        assignBlock<4, 7, 0, 1>(y) = w;
    }

    constexpr operator ColVector<7>() const { return y; }
};

class DroneAttitudeControl {
    ColVector<3> u = {};

  public:
    constexpr DroneAttitudeControl() = default;
    constexpr DroneAttitudeControl(const ColVector<3> &u) : u{u} {}

    constexpr ColVector<3> getAttitudeControl() const { return u; }
    constexpr void setAttitudeControl(const ColVector<3> &u_att) { u = u_att; }
    constexpr operator ColVector<3>() const { return u; }
};