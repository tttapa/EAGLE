#include <C-code-wrappers/CKalmanObserver.hpp>
#include <altitude-controller.h>
#include <attitude-controller.h>
#include <DroneStateControlOutput.hpp>

namespace Attitude {
CKalmanObserver::VecX_t CKalmanObserver::getStateChange(const VecX_t &x_hat,
                                                        const VecY_t &y_sensor,
                                                        const VecU_t &u) {
    VecX_t x_hat_cpy = x_hat;
    updateAttitudeKFEstimate(toCppArray(x_hat_cpy), toCppArray(u),
                             toCppArray(y_sensor), 1);
    return x_hat_cpy;
}
}  // namespace Attitude

namespace Altitude {
CKalmanObserver::VecX_t CKalmanObserver::getStateChange(const VecX_t &x_hat,
                                                        const VecY_t &y_sensor,
                                                        const VecU_t &u) {
    VecX_t x_hat_cpy = x_hat;
    Attitude::CKalmanObserver::VecX_t att = DroneAttitudeState{{1}}; // TODO
    updateAltitudeKFEstimate(toCppArray(x_hat_cpy), toCppArray(u),
                             toCppArray(y_sensor), toCppArray(att),
                             1);
    return x_hat_cpy;
}
}  // namespace Altitude
