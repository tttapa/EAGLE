#include <TiltCorrection.hpp>

ColVector<3> getCorrectedPosition(ColVector<2> rawPosition, double rawHeight,
                                  Quaternion orientation) {
    ColVector<3> v               = {{0, 0, -1}};
    ColVector<3> u               = quatrotate(quatconjugate(orientation), v);
    double height                = rawHeight * v * u;
    double horizontalDistanceAB  = sqrt(sq(rawHeight) - sq(height));
    ColVector<2> u_hat           = getBlock<0, 2, 0, 1>(u);
    ColVector<2> horizontalError = horizontalDistanceAB * normalize(u_hat);
    ColVector<3> position        = vcat(rawPosition - horizontalError, height);
    return position;
}
