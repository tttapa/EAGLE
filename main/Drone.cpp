#include "Drone.hpp"

void Drone::load(const std::filesystem::path &loadPath) {
    PerfTimer timer;

    /* Attitude */

    Aa_att = loadMatrix<Nx_att, Nx_att>(loadPath / "attitude" / "Aa");
    Ba_att = loadMatrix<Nx_att, Nu_att>(loadPath / "attitude" / "Ba");
    Ca_att = loadMatrix<Ny_att, Nx_att>(loadPath / "attitude" / "Ca");
    Da_att = loadMatrix<Ny_att, Nu_att>(loadPath / "attitude" / "Da");

    Ad_att = loadMatrix<Nx_att, Nx_att>(loadPath / "attitude" / "Ad");
    Bd_att = loadMatrix<Nx_att, Nu_att>(loadPath / "attitude" / "Bd");
    Cd_att = loadMatrix<Ny_att, Nx_att>(loadPath / "attitude" / "Cd");
    Dd_att = loadMatrix<Ny_att, Nu_att>(loadPath / "attitude" / "Dd");

    Ad_att_r = getBlock<1, Nx_att, 1, Nx_att>(Ad_att);
    Bd_att_r = getBlock<1, Nx_att, 0, Nu_att>(Bd_att);
    Cd_att_r = getBlock<1, Ny_att, 1, Nx_att>(Cd_att);

    Ts_att = loadDouble(loadPath / "attitude" / "Ts");

    gamma_n = loadMatrix<3, 3>(loadPath / "attitude" / "gamma_n");
    gamma_u = loadMatrix<3, 3>(loadPath / "attitude" / "gamma_u");

    /* Altitude */

    Aa_alt = loadMatrix<Nx_alt, Nx_alt>(loadPath / "altitude" / "Aa");
    Ba_alt = loadMatrix<Nx_alt, Nu_alt>(loadPath / "altitude" / "Ba");
    Ca_alt = loadMatrix<Ny_alt, Nx_alt>(loadPath / "altitude" / "Ca");
    Da_alt = loadMatrix<Ny_alt, Nu_alt>(loadPath / "altitude" / "Da");

    Ad_alt = loadMatrix<Nx_alt, Nx_alt>(loadPath / "altitude" / "Ad");
    Bd_alt = loadMatrix<Nx_alt, Nu_alt>(loadPath / "altitude" / "Bd");
    Cd_alt = loadMatrix<Ny_alt, Nx_alt>(loadPath / "altitude" / "Cd");
    Dd_alt = loadMatrix<Ny_alt, Nu_alt>(loadPath / "altitude" / "Dd");

    Ts_alt = loadDouble(loadPath / "altitude" / "Ts");

    /* General */

    Id      = loadMatrix<3, 3>(loadPath / "I");
    Id_inv  = loadMatrix<3, 3>(loadPath / "I_inv");
    k1      = loadDouble(loadPath / "k1");
    k2      = loadDouble(loadPath / "k2");

    m  = loadDouble(loadPath / "m");
    ct = loadDouble(loadPath / "ct");
    Dp = loadDouble(loadPath / "Dp");

    auto duration = timer.getDuration<std::chrono::microseconds>();
    std::cout << "Loaded Drone data in " << duration << " microseconds"
              << std::endl;

    nh = sqrt((m * g) / (ct * rho * pow(Dp, 4) * Nm));
    uh = nh / k1;

    // TODO

    double Fzh = ct * rho * pow(Dp, 4) * pow(nh, 2) * Nm;
    double Fg  = -g * m;

    std::cout << "nh  = " << nh << std::endl;
    std::cout << "uh  = " << uh << std::endl;
    std::cout << "Fzh = " << Fzh << std::endl;
    std::cout << "Fg  = " << Fg << std::endl;

    assert(isAlmostEqual(Id_inv, inv(Id), 1e-12));
}

Drone::VecX_t Drone::operator()(const Drone::VecX_t &x,
                                const Drone::VecU_t &u) {
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
                             Id_inv * cross(omega, Id * omega));
    x_dot.setMotorSpeed(k2 * (k1 * u_att - n));

    // Navigation/Altitude
    ColVector<3> v  = xx.getVelocity();
    double n_thrust = xx.getThrustMotorSpeed();
    double u_thrust = uu.getThrustControl();

    double F_local_z     = ct * rho * pow(Dp, 4) * pow(n_thrust, 2) * Nm;
    ColVector<3> F_local = {0, 0, F_local_z};
    ColVector<3> F_world = quatrotate(quatconjugate(q), F_local);  // TODO
    ColVector<3> F_grav  = {0, 0, -g * m};
    ColVector<3> a       = (F_world + F_grav) / m;

    x_dot.setVelocity(a);
    x_dot.setPosition(v);
    x_dot.setThrustMotorSpeed(k2 * (k1 * u_thrust - n_thrust));

    return x_dot;
}

Drone::VecY_t Drone::getOutput(const Drone::VecX_t &x, const Drone::VecU_t &u) {
    DroneState xx   = {x};
    DroneControl uu = {u};
    ColVector<Ny_att> y_att =
        Ca_att * xx.getAttitude() + Da_att * uu.getAttitudeControl();
    ColVector<Ny_nav + Ny_alt> y_nav = xx.getPosition();
    return DroneOutput{y_att, y_nav};
}

Drone::VecX_t Drone::getInitialState() const {
    DroneState xx = {};
    xx.setOrientation(eul2quat({0, 0, 0}));
    xx.setAngularVelocity({0, 0, 0});
    xx.setMotorSpeed({0, 0, 0});
    xx.setVelocity({0, 0, 0});
    xx.setPosition({0, 0, 0});
    xx.setThrustMotorSpeed(nh);
    return xx;
}

Drone::Controller::VecU_t Drone::Controller::
operator()(const Drone::Controller::VecX_t &x,
           const Drone::Controller::VecR_t &r) {
    DroneState xx  = {x};
    DroneOutput rr = {r};
    DroneControl uu;
    uu.setAttitudeControl(attCtrl(xx.getAttitude(), rr.getAttitude()));
    uu.setThrustControl(altCtrl(xx.getAltitude(), rr.getAltitude()));
    // TODO: clamp
    return uu;
}