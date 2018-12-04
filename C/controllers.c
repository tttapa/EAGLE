#include "controllers.h"
#include "attitude.h"
#include "quaternion.h"

float obsv_diff[6] = {};
float ref_diff[9]  = {};

void updateAttitudeObserver() {

    //
    // STEP 0: Variable Declaration
    //

    // 0.3: Temporary vector object, used to store the orientation adjustment.
    Vec32 temp_vect;

    // 0.4: Quaternion representation of state estimate, measurement and reference.
    Quat32 att_hat_quat;
    Quat32 q_quat;
    Quat32 r_quat;

    // 0.5: Quaternions to store mathimatical operations.
    Quat32 diff_quat;
    Quat32 mult_quat;
    Quat32 temp_quat;

    // 0.6: Create quaternions from att_hat, q, and r
    att_hat_quat.w = att_hat[0];
    att_hat_quat.x = att_hat[1];
    att_hat_quat.y = att_hat[2];
    att_hat_quat.z = att_hat[3];

    q_quat.w = q[0];
    q_quat.x = q[1];
    q_quat.y = q[2];
    q_quat.z = q[3];

    r_quat.w = r[0];
    r_quat.x = r[1];
    r_quat.y = r[2];
    r_quat.z = r[3];

    //
    // STEP 1: Update Kalman Filter
    //         att_hat = Ad*att_hat + Bd*u + L*(y-Cd*att_hat - Dd*u)
    //

    // 1.1: Store difference between y and att_hat in obsv_diff.
    quat32_difference(&diff_quat, &q_quat, &att_hat_quat);
    obsv_diff[0] = diff_quat.x;
    obsv_diff[1] = diff_quat.y;
    obsv_diff[2] = diff_quat.z;
    obsv_diff[3] = att_y[4] - att_hat[4];
    obsv_diff[4] = att_y[5] - att_hat[5];
    obsv_diff[5] = att_y[6] - att_hat[6];

    // 1.2: Update the Kalman Filter

    // 1.2.1: Create first orientation adjustment quaternion.
    temp_vect.x =
        0;  //0.0021008*att_hat[4];// + 1.1747e-05*att_hat[7] + 5.533e-05*att_u[0];
    temp_vect.y =
        0;  //0.0021008*att_hat[5];// + 1.1091e-05*att_hat[8] + 5.2238e-05*att_u[1];
    temp_vect.z =
        0;  //0.0021008*att_hat[6];// - 7.0849e-06*att_hat[9] + 0.00089983*att_u[2];
    quat32_from_vector(&temp_quat, &temp_vect);

    // 1.2.2: Calculate first part of orientation estimate.
    quat32_multiply(&mult_quat, &att_hat_quat, &temp_quat);

    // 1.2.3: Calculate second orientation adjustment quaternion.
    temp_vect.x = 0.9902 * obsv_diff[0];  // + 2.0588e-07*obsv_diff[3];
    temp_vect.y = 0.9902 * obsv_diff[1];  // + 2.0524e-07*obsv_diff[4];
    temp_vect.z = 0.9902 * obsv_diff[2];  // + 2.0214e-07*obsv_diff[5];
    quat32_from_vector(&temp_quat, &temp_vect);

    // 1.2.4: Calculate new orientation estimate.
    quat32_multiply(&att_hat_quat, &mult_quat, &temp_quat);
    quat32_normalize(&temp_quat, &att_hat_quat);

    // 1.2.5: Calculate next att_hat.
    att_hat[0] = temp_quat.w;
    att_hat[1] = temp_quat.x;
    att_hat[2] = temp_quat.y;
    att_hat[3] = temp_quat.z;
    att_hat[4] = att_hat[4] + 0.010964 * att_hat[7] + 0.078235 * att_u[0] +
                 0.0009904 * obsv_diff[0] + 0.9902 * obsv_diff[3];
    att_hat[5] = att_hat[5] + 0.010351 * att_hat[8] + 0.073863 * att_u[1] +
                 0.0009904 * obsv_diff[1] + 0.9902 * obsv_diff[4];
    att_hat[6] = att_hat[6] - 0.0066126 * att_hat[9] + 0.84122 * att_u[2] +
                 0.0009904 * obsv_diff[2] + 0.9902 * obsv_diff[5];
    att_hat[7] = 0.88688 * att_hat[7] + 13.1844 * att_u[0] +
                 4.8052e-05 * obsv_diff[0] + 0.044939 * obsv_diff[3];
    att_hat[8] = 0.88688 * att_hat[8] + 13.1844 * att_u[1] +
                 4.538e-05 * obsv_diff[1] + 0.04244 * obsv_diff[4];
    att_hat[9] = 0.88688 * att_hat[9] + 13.1844 * att_u[2] -
                 2.903e-05 * obsv_diff[2] - 0.027149 * obsv_diff[5];
    att_hat[10] =
        att_hat[10] - 0.00099019 * obsv_diff[0] + 1.0109e-06 * obsv_diff[3];
    att_hat[11] =
        att_hat[11] - 0.00099019 * obsv_diff[1] + 1.0109e-06 * obsv_diff[4];
    att_hat[12] =
        att_hat[12] - 0.00099019 * obsv_diff[2] + 1.0107e-06 * obsv_diff[5];
}
void updateAttitudeController() {

    /*

	//
	// STEP 2: Calculate Next Output
	//
    Quat32 r_quat;
    r_quat.w = att_ref[0];
	r_quat.x = att_ref[1];
	r_quat.y = att_ref[2];
	r_quat.z = att_ref[3];

	Quat32 att_hat_quat;
    att_hat_quat.w = att_hat[0];
    att_hat_quat.x = att_hat[1];
    att_hat_quat.y = att_hat[2];
    att_hat_quat.z = att_hat[3];

	// 2.1: Store difference between ref and att_hat in ref_diff.
    Quat32 diff_quat;
	quat32_difference(&diff_quat, &r_quat, &att_hat_quat);
	ref_diff[0] = diff_quat.x;
	ref_diff[1] = diff_quat.y;
	ref_diff[2] = diff_quat.z;
	ref_diff[3] = att_ref[4] - att_hat[4] - att_hat[10];
	ref_diff[4] = att_ref[5] - att_hat[5] - att_hat[11];
	ref_diff[5] = att_ref[6] - att_hat[6] - att_hat[12];
	ref_diff[6] = -att_hat[7];
	ref_diff[7] = -att_hat[8];
	ref_diff[8] = -att_hat[9];
	
	// 2.2: Calculate next output.
	att_u[0] = 0.075335*ref_diff[0] + 0.088048*ref_diff[3] + 0.067769*ref_diff[6];
	att_u[1] = 0.07536*ref_diff[1] + 0.088765*ref_diff[4] + 0.067744*ref_diff[7];
	att_u[2] = 0.07563*ref_diff[2] + 0.19748*ref_diff[5] + 0.054791*ref_diff[8];

    */

    float *ref     = att_ref;
    float *x_hat_r = att_hat + 1;

    /* Calculate the real part of the quaternion from the reduced state. */
    float xq0 = sqrt(1.0 - x_hat_r[0] * x_hat_r[0] - x_hat_r[1] * x_hat_r[1] -
                     x_hat_r[2] * x_hat_r[2]);
    /* Generated calculations for control signal. */
    att_u[0] = 9.9507803487420201 * ref[4 - 1] * x_hat_r[2 - 1] -
               0.022556336398881428 * x_hat_r[7 - 1] -
               9.9507803487420201 * ref[1 - 1] * x_hat_r[1 - 1] -
               9.9507803487420201 * ref[3 - 1] * x_hat_r[3 - 1] -
               0.44806396994101899 * x_hat_r[4 - 1] +
               9.9507803487420201 * ref[2 - 1] * xq0;
    att_u[1] = 10.003711277857807 * ref[2 - 1] * x_hat_r[3 - 1] -
               0.021916747614163647 * x_hat_r[8 - 1] -
               10.003711277857807 * ref[1 - 1] * x_hat_r[2 - 1] -
               0.45380397300300307 * x_hat_r[5 - 1] -
               10.003711277857807 * ref[4 - 1] * x_hat_r[1 - 1] +
               10.003711277857807 * ref[3 - 1] * xq0;
    att_u[2] = 0.0076583232871799811 * x_hat_r[9 - 1] -
               0.31551408720883622 * x_hat_r[6 - 1] -
               13.699906287174223 * ref[1 - 1] * x_hat_r[3 - 1] -
               13.699906287174223 * ref[2 - 1] * x_hat_r[2 - 1] +
               13.699906287174223 * ref[3 - 1] * x_hat_r[1 - 1] +
               13.699906287174223 * ref[4 - 1] * xq0;
}
