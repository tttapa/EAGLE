from math import pi
from py_drone_module import eul2quat, quatmultiply, quatconjugate, DroneReference


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
