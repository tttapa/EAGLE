from py_drone_module import Drone, DroneReference, DroneReferenceFunction, eul2quat, AdaptiveODEOptions, quatmultiply, quatconjugate
from DroneParamsAndMatricesBuilder import buildDroneParamsAndMatrices
from numpy import array, diag
from math import pi
import DronePlotter

d = Drone(buildDroneParamsAndMatrices(m=2))

x0 = d.getStableState()

Q_att = diag((139.6245112700232,
              139.6245112700232,
              15.2811761590895,
              1.1505204155597211,
              1.1505204155597211,
              0.1209919487616804,
              9.976475759487083e-08,
              9.976475759487083e-08,
              9.976475759487083e-09))
R_att = diag((8, 8, 8))
K_pi_alt = array((0.0, 0.9, 0.5, -0.1))
maxIntegralInfluence = 0.1

controller = d.getController(Q_att, R_att, K_pi_alt, maxIntegralInfluence)


def get_reference(t):
    m = 0.5  # time scale factor

    qz = eul2quat((10*pi/180, 0, 0))
    qy = eul2quat((0, 10*pi/180, 0))
    qx = eul2quat((0, 0, 10*pi/180))
    q = eul2quat((0, 0, 0))

    if (t >= m * 1 and t < m * 3):
        q = quatmultiply(q, qz)
    if (t >= m * 5 and t < m * 7):
        q = quatmultiply(q, qy)
    if (t >= m * 9 and t < m * 11):
        q = quatmultiply(q, qx)

    if (t >= m * 13 and t < m * 15):
        q = quatmultiply(q, quatconjugate(qy))
    if (t >= m * 17 and t < m * 19):
        q = quatmultiply(q, quatconjugate(qx))
    if (t >= m * 21 and t < m * 23):
        q = quatmultiply(q, quatconjugate(qz))

    if (t >= m * 27 and t < m * 30):
        q = quatmultiply(q, qy)
    if (t >= m * 28 and t < m * 31):
        q = quatmultiply(q, qx)

    if (t >= m * 33 and t < m * 36):
        q = quatmultiply(q, quatconjugate(qx))
    if (t >= m * 34 and t < m * 37):
        q = quatmultiply(q, quatconjugate(qy))

    rr = DroneReference()
    rr.setOrientation(q)
    rr.setPosition((0, 0, 1.0 * (t >= m * 24)))
    return rr.asColVector()


ref_function = DroneReferenceFunction(get_reference)

odeopt = AdaptiveODEOptions()
odeopt.t_start = 0
odeopt.t_end = 16
odeopt.epsilon = 1e-6
odeopt.h_start = 1e-6
odeopt.h_min = 1e-8
odeopt.maxiter = int(1e6)

if __name__ == '__main__':
    result = d.simulate(controller, ref_function, x0, odeopt)
    fig = DronePlotter.plot(result)
    DronePlotter.show(fig)
