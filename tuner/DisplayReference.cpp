#include "DisplayReference.hpp"
#include <Degrees.hpp>

DisplayReference::DisplayReference()
    : qz{eul2quat({10_deg, 0, 0})},  //
      qy{eul2quat({0, 10_deg, 0})},  //
      qx{eul2quat({0, 0, 10_deg})} {}

DisplayReference::VecR_t DisplayReference::operator()(double t) {
    Quaternion q = qu;
    if (t >= m * 0 && t < m * 1)
        q = quatmultiply(q, qx);
    if (t >= m * 2 && t < m * 3)
        q = quatmultiply(q, qy);
    if (t >= m * 4 && t < m * 5)
        q = quatmultiply(q, qz);

    if (t >= m * 6 && t < m * 7)
        q = quatmultiply(q, qx);
    if (t >= m * 6 && t < m * 7)
        q = quatmultiply(q, qy);

    DroneAttitudeOutput rr;
    rr.setOrientation(q);
    return rr;
}