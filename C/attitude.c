/**********************************************************************
*   Attitude controller source file
*   this script contains functions used to generate inputs to the drone
*   that allow it to track the attitude reference from the RC or from the
*   navigation controller
*
*   author: p. coppens
***********************************************************************/
#include "attitude.h"
#include "controllers.h"
#include <math.h>

#define THRUST_MAX 0.85

// Global variables
// ====================================================================================================================
float rel_yaw = 0; 	/* relative yaw, used to build the reference */
/* controller inputs */
float r[4] = {}; 	/* reference signal */
float q[4] = {}; 	/* orientation measurement */
float w[3] = {}; 	/* angular velocity measurement */

/* attitude output */
float v[4] = {}; 	/* voltage control signals */

//********************************//
//*** EAGLE-1 ATTITUDE GLOBALS ***//
//********************************//

//*** Controller State: ux,uy,uz
float att_u[3] = {};

//*** Controller Reference: q4,w3
float att_ref[7] = {};

//*** Observer Input: q4,w3
float att_y[7] = {};

//*** Observer State: q4,w3,n3,b3
float att_hat[13] = {};

/**
 * TO IMPLEMENT:
 * This function should compute a vector of control signals (v[4]) based on 4 input signals
 * (4 parameters of the function) either coming from the remote control or from the
 * altitude/navigation controllers.
 * v[4] should allow the drone to follow the reference r[4].
 *
 * parameters:
 * 		thrust: 		provided by the RC or the altitude controller [0..1]
 * 		rot_x:			proportional to the pitch, provided by the RC or the navigation_controller [0..1]
 * 		rot_y			proportional to the roll, provided by the RC or the navigation controller [0..1]
 * 		rot_z			proportional to the yaw, provided by the RC (or the yaw controller) [0..1]
 */
void controller_flying(float thrust, float rot_x, float rot_y, float rot_z) {
	load_input(thrust, rot_x, rot_y, rot_z);
	load_measurements();

	//***************************//
	//*** Attitude Controller ***//
	//***************************//

	// Update the attitude controller
	updateAttitudeObserver();
	updateAttitudeController();


}

/**
 * TO IMPLEMENT:
 * This function should assign the initial state of the observer, the initial control
 * signal of the controller and any other initial values if needed.
 */
void controller_init() {
	// =================================================
	// Add reset of observer and controller
	// =================================================

	//***********************************//
	//*** Reset Observer & Controller ***//
	//***********************************//
	int counter;
    att_hat[0] = 1;
	for(counter=1; counter<10; counter++) {att_hat[counter] = 0;}
	att_u[0] = 0;
	att_u[1] = 0;
	att_u[2] = 0;

}

/**
 * This method turns of all motors by sending out PWM to all ESC's
 * with v[4] = {0, 0, 0, 0}
 */
void controller_idle() {
    //PWMOutput(0, 0, 0, 0);
}

/**
 * This function accesses the IMU's measurements and stores them in variables
 * to be used in the framework.
 * These measurements are q[4] and w[3].
 * q[4] --> measured quaternion
 * w[3] --> measured angular velocities
 */
void load_measurements() {
	att_y[0] = q[0];
	att_y[1] = q[1];
	att_y[2] = q[2];
	att_y[3] = q[3];
	att_y[4] = w[0];
	att_y[5] = w[1];
	att_y[6] = w[2];
	
	att_ref[0] = q[0];
	att_ref[1] = q[1];
	att_ref[2] = q[2];
	att_ref[3] = q[3];
	att_ref[4] = 0;
	att_ref[5] = 0;
	att_ref[6] = 0;
}

/**
 * This function reads the input from the RC or the altitude/navigation controllers
 * and builds the reference r[4] out of it, the details of this function are not important.
 *
 * parameters:
 * 		thrust: 		provided by the RC or the altitude controller [0..1]
 * 		rot_x:			proportional to the pitch, provided by the RC or the navigation_controller [0..1]
 * 		rot_y			proportional to the roll, provided by the RC or the navigation controller [0..1]
 * 		rot_z			proportional to the yaw, provided by the RC (or the yaw controller) [0..1]
 */
void load_input(float thrust, float rot_x, float rot_y, float rot_z) {
	float d;
	float w;
	float s;
	float rx;
	float ry;

	/* get the relative rotation input */
	if (thrust < 0.1)
		rel_yaw = 0.0;
	else if ((rot_z > 0.05) || (rot_z < -0.05))
		rel_yaw += 0.5 * rot_z * 2 * 3.1415 / 238.0;

	/* transform the reference input into radians */
	rx = rot_x * 45; 
        ry = rot_y * 45;
	CLAMP_INPLACE(rx, -10.0, 10.0); CLAMP_INPLACE(ry, -10.0, 10.0);

	rx = rx * 3.1415 / 180; ry = ry * 3.1415 / 180;

	/* build the quaternion */
	d = rx*rx + ry*ry + rel_yaw*rel_yaw;
	w = cos(sqrt(d)/2);
	if (d==0) s=0;
	else s = sin(sqrt(d)/2)/sqrt(d);

    r[0] = w;
    r[1] = rx*s;
    r[2] = ry*s;
    r[3] = rel_yaw*s;
}
