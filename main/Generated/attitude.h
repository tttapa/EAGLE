#ifdef __cplusplus
extern "C" {
#endif

typedef double ControllerOutputU[3];
typedef double ReferenceQuaternion[4];
typedef double ReducedStateX[9];

/**
 * @brief   Given the current state estimation `x_hat` and the target reference
 *          `ref`, calculate the control signal `u`.
 *
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 *
 * @param   x_hat
 *          An array of 9 the estimates of the nine (reduced) states.
 * @param   ref
 *          The reference quaternion.
 * @param   u
 *          The output array where the control vector will be stored.
 */
void getControllerOutput(const ReducedStateX x_hat_r,
                         const ReferenceQuaternion ref, ControllerOutputU u);

#ifdef __cplusplus
}
#endif