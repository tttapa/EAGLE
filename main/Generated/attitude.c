/* Automatically generated, using 
Q = 
* 40 0 0 0 0 0 0 0 0
* 0 40 0 0 0 0 0 0 0
* 0 0 40 0 0 0 0 0 0
* 0 0 0 4.000000e-02 0 0 0 0 0
* 0 0 0 0 4.000000e-02 0 0 0 0
* 0 0 0 0 0 4.000000e-02 0 0 0
* 0 0 0 0 0 0 6.714924e-04 0 0
* 0 0 0 0 0 0 0 6.714924e-04 0
* 0 0 0 0 0 0 0 0 6.714924e-04
R = 
* 9.121488e+00 0 0
* 0 9.121488e+00 0
* 0 0 9.121488e+00
*/

#include "attitude.h"
#include <math.h> /* sqrt */

/*
 * @note    This is an automatically generated function. Do not edit it here,
 *          edit it in the template, or in the MATLAB code generator.
 */
void getControllerOutput(const ReducedStateX x_hat_r,
                         const ReferenceQuaternion ref, ControllerOutputU u) {
    /* Calculate the real part of the quaternion from the reduced state. */
    double xq0 = sqrt(1.0 - x_hat_r[0] * x_hat_r[0] - x_hat_r[1] * x_hat_r[1] -
                      x_hat_r[2] * x_hat_r[2]);
    /* Generated calculations for control signal. */
    u[0] = 1.9413226506703893 * ref[4 - 1] * x_hat_r[2 - 1] -
           0.0101899761375044 * x_hat_r[7 - 1] -
           1.9413226506703893 * ref[1 - 1] * x_hat_r[1 - 1] -
           1.9413226506703893 * ref[3 - 1] * x_hat_r[3 - 1] -
           0.13010590365305746 * x_hat_r[4 - 1] +
           1.9413226506703893 * ref[2 - 1] * xq0;
    u[1] = 1.9444854789794261 * ref[2 - 1] * x_hat_r[3 - 1] -
           0.0099736932638821458 * x_hat_r[8 - 1] -
           1.9444854789794261 * ref[1 - 1] * x_hat_r[2 - 1] -
           0.132591671802718 * x_hat_r[5 - 1] -
           1.9444854789794261 * ref[4 - 1] * x_hat_r[1 - 1] +
           1.9444854789794261 * ref[3 - 1] * xq0;
    u[2] = 0.0062499506019343268 * x_hat_r[9 - 1] -
           0.19086562455952086 * x_hat_r[6 - 1] -
           2.0036437402475 * ref[1 - 1] * x_hat_r[3 - 1] -
           2.0036437402475 * ref[2 - 1] * x_hat_r[2 - 1] +
           2.0036437402475 * ref[3 - 1] * x_hat_r[1 - 1] +
           2.0036437402475 * ref[4 - 1] * xq0;
}