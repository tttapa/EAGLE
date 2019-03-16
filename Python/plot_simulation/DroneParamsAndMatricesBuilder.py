from py_drone_module import DroneParamsAndMatrices
from numpy.linalg import inv
from numpy import array, zeros, eye, hstack, vstack
from scipy.signal import cont2discrete


def buildDroneParamsAndMatrices(quat_params):
    p = quat_params
    d = DroneParamsAndMatrices()
    # Parameters
    d.Ts_att = 1.0 / 238
    d.Ts_alt = d.Ts_att * 12
    d.gamma_n = p['gamma_n']
    d.gamma_u = p['gamma_u']
    d.Id = p['I']
    d.Id_inv = inv(p['I'])
    d.k1 = p['k1']
    d.k2 = p['k2']
    d.Nm = p['Nm']
    d.g = p['g']
    d.rho = p['rho']
    d.m = p['m']
    d.ct = p['ct']
    d.Dp = p['Dp']
    d.nh = p['nh']
    d.uh = p['uh']

    # Attitude model
    d.Aa_att = vstack((
        zeros((1, 10)),
        hstack((zeros((3, 4)), 0.5 * eye(3), zeros((3, 3)))),
        hstack((zeros((3, 7)), d.gamma_n)),
        hstack((zeros((3, 7)), -d.k2 * eye(3)))
    ))
    d.Ba_att = vstack((
        zeros((4, 3)),
        d.gamma_u,
        d.k1*d.k2*eye(3)
    ))
    d.Ca_att = hstack((eye(7), zeros((7, 3))))
    d.Da_att = zeros((7, 3))

    # Discretize the continuous attitude model
    d.Ad_att, d.Bd_att, d.Cd_att, d.Dd_att, _ = cont2discrete(
        (d.Aa_att, d.Ba_att, d.Ca_att, d.Da_att), d.Ts_att, method='zoh')

    # Altitude model
    a = 2 * d.nh * p['kt']

    d.Aa_alt = array(((-d.k2,  0,  0),
                    (0,      0,  1),
                    (a,      0,  0)))
    d.Ba_alt = array((d.k1 * d.k2,
                    0,
                    0))
    d.Ca_alt = array((0, 1, 0))
    d.Da_alt = zeros((1, 1))

    # Discretize the continuous altitude model
    d.Ad_alt, d.Bd_alt, d.Cd_alt, d.Dd_alt, _ = cont2discrete(
        (d.Aa_alt, d.Ba_alt, d.Ca_alt, d.Da_alt), d.Ts_alt, method='zoh')

    # Calculates reduced attitude system matrices and equilibrium matrix G for
    # attitude and altitude
    d.calculate()
    
    return d