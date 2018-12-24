#pragma once

#include <Drone.hpp>
#include <Degrees.hpp>

/** 
 * @brief   Reference function for controller simulations.
 */
struct TestReferenceFunction : public TimeFunctionT<ColVector<Ny>> {
    ColVector<Ny> operator()(double t) override {
        constexpr double m = 0.4;

        Quaternion q = qu;
        if (t >= m * 1 && t < m * 3)
            q = quatmultiply(q, qz);
        if (t >= m * 5 && t < m * 7)
            q = quatmultiply(q, qy);
        if (t >= m * 9 && t < m * 11)
            q = quatmultiply(q, qx);

        if (t >= m * 13 && t < m * 15)
            q = quatmultiply(q, quatconjugate(qy));
        if (t >= m * 17 && t < m * 19)
            q = quatmultiply(q, quatconjugate(qx));
        if (t >= m * 21 && t < m * 23)
            q = quatmultiply(q, quatconjugate(qz));

        if (t >= m * 27 && t < m * 30)
            q = quatmultiply(q, qy);
        if (t >= m * 28 && t < m * 31)
            q = quatmultiply(q, qx);

        if (t >= m * 33 && t < m * 36)
            q = quatmultiply(q, quatconjugate(qx));
        if (t >= m * 34 && t < m * 37)
            q = quatmultiply(q, quatconjugate(qy));

        DroneOutput rr;
        rr.setOrientation(q);
        rr.setPosition({0, 0, 1.0 * (t >= m * 24)});
        return rr;
    }
    const Quaternion qz = eul2quat({10_deg, 0, 0});
    const Quaternion qy = eul2quat({0, 10_deg, 0});
    const Quaternion qx = eul2quat({0, 0, 10_deg});
    const Quaternion qu = eul2quat({0, 0, 0});
};

/** [1 0 0 0 0 0 0] */
ConstantTimeFunctionT<ColVector<7>> ref0 = vcat(ones<1, 1>(), zeros<6, 1>());