#include <C-code-wrappers/CLQRController.hpp>
#include <DroneStateControlOutput.hpp>
#include <altitude-controller.h>
#include <attitude-controller.h>

namespace Attitude {
CLQRController::VecU_t
CLQRController::getRawControllerOutput(const VecX_t &x, const VecR_t &ref) {
    Quaternion rq = DroneAttitudeOutput{ref}.getOrientation();
    VecU_t u;
    getAttitudeControllerOutput(toCppArray(x), toCppArray(rq), toCppArray(u),
                                toCppArray(integral), config,
                                enableIntegral ? 0.5 : -0.5);
    return u;
}
}  // namespace Attitude

namespace Altitude {
CLQRController::VecU_t
CLQRController::getRawControllerOutput(const VecX_t &x, const VecR_t &ref) {
    VecU_t u;
    getAltitudeControllerOutput(toCppArray(x), toCppArray(ref), toCppArray(u),
                                toCppArray(integral), config,
                                enableIntegral ? 0.5 : -0.5);
    return u;
}
}  // namespace Altitude