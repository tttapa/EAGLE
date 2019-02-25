#pragma once

#include <Def.hpp>
#include <Matrix.hpp>
#include <filesystem>

struct DroneParamsAndMatrices {

    /** 
     * @brief   Loads all system matrices and parameters from the respective 
     *          files in the given folder. 
     */
    void load(const std::filesystem::path &loadPath);

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