from math import sqrt, pi
import numpy as np


def quat_params(**kwargs):
    # ---- Independent Quantities ----

    p = {
        # General parameters
        'Vmax': 11.1,          # V ....... maximum voltage
        'Vmin': 1.11,          # V ....... minimum voltage
        'Nm': 4,               # - ....... number of motors
        'm': 1.850,            # kg ...... total mass
        'L': 0.27,             # m ....... arm length
        'rho': 1.225,          # kg/m3 ... air density 
        'g': 9.81,             # m/s2 .... gravitational acceleration

        # Propellers
        'ct': 0.1,             # - ....... thrust coefficient
        'cp': 0.04,            # - ....... power coefficient
        'mp': 20e-3,           # kg ...... propeller mass (20g)
        'Dp': 12*2.54e-2,      # m ....... propeller diameter (12")

        # Motors
        'Kv': 700,             # rpm/V ... motor speed constant
        'tau_m': 35e-3,        # s ....... motor's time constant (35ms)
        'mr': 42e-3,           # kg ...... rotor mass (moving parts) (42g)
        'mm': 102e-3,          # kg ...... motor total mass (102g)
        'rr': 1.9e-2,          # m ....... rotor radius (1.9cm)

        # Moments of inertia
        'Ixx': 0.0321,         # kgm2 .... Ixx moment of inertia
        'Iyy': 0.0340,         # kgm2 .... Iyy moment of inertia
        'Izz': 0.0575,         # kgm2 .... Izz moment of inertia
    }

    # ---- Override the default values ----

    p.update(kwargs)

    # ---- Computed Quantities ----

    p['nh'] = sqrt((p['m'] * p['g']) /   # rps ... Hovering motor speed
                   (p['ct'] * p['rho'] * p['Dp']**4 * p['Nm']))

    # Very rough estimation of moments of inertia
    p['Ip'] = p['mp'] * p['Dp']**2 / 12  # kgm² ... Propeller moment of inertia
    p['Im'] = p['mr'] * p['rr']**2       # kgm² ... Rotor moment of inertia
    p['I'] = np.diag([p['Ixx'],          # kgm³ ... Inertia matrix
                      p['Iyy'],
                      p['Izz']])

    # Model constants
    p['k1'] = p['Kv']*(p['Vmax']-p['Vmin'])/60
    p['k2'] = 1/p['tau_m']
    p['k3'] = np.diag([2 * p['ct'] * p['rho'] * p['nh'] * p['Dp']**4 *
                       p['Nm'] * p['L'] / sqrt(2) / p['Ixx'],
                       2 * p['ct'] * p['rho'] * p['nh'] * p['Dp']**4 *
                       p['Nm'] * p['L'] / sqrt(2) / p['Iyy'],
                       2 * p['cp'] * p['rho'] * p['nh'] * p['Dp']**5 *
                       p['Nm'] / (2 * pi * p['Izz'])])
    p['k4'] = np.diag([0,
                       0,
                       2 * pi * p['Nm'] * (p['Im'] + p['Ip']) / p['Izz']])

    # Matrix Gamma_n
    p['gamma_n'] = p['k3'] - p['k2']*p['k4']

    # Matrix Gamma_u
    p['gamma_u'] = np.diag([0,
                            0,
                            p['k4'][2, 2] * p['k2'] * p['k1']])

    # Own constants
    p['kt'] = 4 * p['ct'] * p['rho'] * p['Dp']**4 / p['m']
    p['uh'] = p['nh'] / p['k1']

    return p
