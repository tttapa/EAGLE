/**********************************************************************************************************************
*   Attitude controller header file
*   this script contains functions used to generate inputs to the drone
*   that allow it to track the attitude reference from the RC or from the 
*   navigation controller
*
*   author: p. coppens
***********************************************************************************************************************/
#ifndef ATTITUDE_H
#define ATTITUDE_H

// Header Files
// ====================================================================================================================
//#include "../main.h"

// Constant definitions
// ====================================================================================================================
#define CTR_OUT_LOW 40.0	/* Minimal PWM output signal */
#define CTR_OUT_HIGH 90.0	/* Maximal PWM output signal */

// Prototype definitions
// ====================================================================================================================
void controller_flying(float thrust, float rot_x, float rot_y, float rot_z);
void controller_init();
void controller_idle();
void load_measurements();
void load_input(float thrust, float rot_x, float rot_y, float rot_z);

// Macros
// ====================================================================================================================

/**
 * Used to restrict a variable to a certain interval.
 *
 * parameters:
 *  	x:      the variable to restrict to an interval
 *  	lo:     the lower boundary
 *  	hi:     the higher boundary
 */
#define CLAMP_INPLACE(x, lo, hi) { \
	if((x) < ((lo))) \
		(x) = (lo); \
	if((x) > ((hi))) \
		(x) = (hi); \
}

// Global variables
// ====================================================================================================================
extern float rel_yaw; 	/* relative yaw, used to build the reference */
/* controller inputs */
extern float r[4]; 	/* reference signal */
extern float q[4]; 	/* orientation measurement */
extern float w[3]; 	/* angular velocity measurement */

/* attitude output */
extern float v[4]; 	/* voltage control signals */

//********************************//
//*** EAGLE-1 ATTITUDE GLOBALS ***//
//********************************//

//*** Controller State: ux,uy,uz
extern float att_u[3];

//*** Controller Reference: q4,w3
extern float att_ref[7];

//*** Observer Input: q4,w3
extern float att_y[7];

//*** Observer State: q4,w3,n3,b3
extern float att_hat[13];


#endif /* ATTITUDE_H */
