#pragma once

#include "Drone.hpp"

/** 
 * @brief   Reference function for controller simulations.
 */
struct TestReferenceFunction : public TimeFunctionT<ColVector<Ny>> {
    ColVector<Ny> operator()(double t) override {
        // ---------------------------------------------------------------------
        ColVector<Ny> ref = {};

        constexpr double m = 0.5;

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

        if (t >= m * 25 && t < m * 27)
            q = quatmultiply(q, qz);
        if (t >= m * 26 && t < m * 28)
            q = quatmultiply(q, qy);
        if (t >= m * 27 && t < m * 29)
            q = quatmultiply(q, qx);

        if (t >= m * 31 && t < m * 33)
            q = quatmultiply(q, quatconjugate(qx));
        if (t >= m * 32 && t < m * 34)
            q = quatmultiply(q, quatconjugate(qy));
        if (t >= m * 33 && t < m * 35)
            q = quatmultiply(q, quatconjugate(qz));

        assignBlock<0, 4, 0, 1>(ref) = q;
        return ref;
        // ---------------------------------------------------------------------
    }
    const Quaternion qz = eul2quat({M_PI / 32, 0, 0});
    const Quaternion qy = eul2quat({0, M_PI / 64, 0});
    const Quaternion qx = eul2quat({0, 0, M_PI / 64});
    const Quaternion qu = eul2quat({0, 0, 0});
};

/** [1 0 0 0 0 0 0] */
ConstantTimeFunctionT<ColVector<7>> ref0 = vcat(ones<1, 1>(), zeros<6, 1>());