#include <CLQRController.hpp>
#include <altitude-controller.h>
#include <attitude-controller.h>

namespace Attitude {
CLQRController::VecU_t
CLQRController::getRawControllerOutput(const VecX_t &x, const VecR_t &ref) {
    Quaternion rq = DroneAttitudeOutput{ref}.getOrientation();
    VecU_t u;
    getAttitudeControllerOutput(toCppArray(x), toCppArray(rq), toCppArray(u),
                                1);
    return u;
}
}  // namespace Attitude

namespace Altitude {
CLQRController::VecU_t
CLQRController::getRawControllerOutput(const VecX_t &x, const VecR_t &ref) {
    VecU_t u;
    getAltitudeControllerOutput(toCppArray(x), toCppArray(ref), toCppArray(u),
                                toCppArray(integral), 1);
    return u;
}
}  // namespace Altitude