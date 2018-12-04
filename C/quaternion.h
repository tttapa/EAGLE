/***********************************************************************
*   Quaternion header file
*   This script contains functions which can be used to create quaternions
*   and perform different operations on these quaternions
*
*   This file makes use of 'structures', which are defined in the header file.
*   In order to create a 'Vec32' structure, do the following:
*   Vec32 vector_name;
*   vector_name.x = value_x;
*   vector_name.y = value_y;
*   vector_name.z = value_z;
*   To call a function, do for example:
*   Quat32 quat_name;
*   quat32_from_vector(&quat_name, &vector_name);
*
*   See AHRS.c for some examples if necessary
************************************************************************/
#ifndef QUATERNION_H
#define QUATERNION_H

// Header Files
// ====================================================================================================================
#include <stdint.h>
#include "math.h"

// Structure definitions
// ====================================================================================================================

// General 32-bit vector (arbitrary units).
typedef struct {
    float x;
    float y;
    float z;
} Vec32;

// General 32-bit quaternion (fix2.30).
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quat32;

// Prototype definitions
// ====================================================================================================================

/**
 * This method creates a 'Quat32' structure from a given 'Vec32' structure
 */
void quat32_from_vector(Quat32* res, Vec32* vec);

/**
 * This method creates a 'Quat32' structure from a given 'Vec32' structure, assuming proper unit quaternion vector.
 */
void quat32_from_vector_fast(Quat32* res, Vec32* vec);

/**
 * This method calculates a * b and stores the result in res
 */
void quat32_multiply(Quat32* res, Quat32* a, Quat32* b);

/**
 * Calculate the conjucate of a and store it in res.
 */
void quat32_conjugate(Quat32* res, Quat32* a);

/**
 * Calculate the quaternion difference between a and b, and store the result in res.
 */
void quat32_difference(Quat32* res, Quat32* a, Quat32* b);

/**
 * This method calculates a' * b and stores the result in res
 */
void quat32_leftdivide(Quat32* res, Quat32* a, Quat32* b);

/**
 * This method transforms/rotates a vector by a quaternion
 * Vector length should be less than 2^31 to avoid overflow of intermediate results
 */
void quat32_transform(Vec32* res, Quat32* q, Vec32* v);

/**
 * This method inverses the transformation/rotation of a vector by a quaternion
 * Vector length should be less than 2^31 to avoid overflow of intermediate results
 */
void quat32_invtransform(Vec32* res, Quat32* q, Vec32* v);

/**
 * This method normalizes a vector
 */
void vec32_normalize_32f16(Vec32* res, Vec32* v);

/**
 * This method normalizes a quaternion
 */
void quat32_normalize(Quat32* res, Quat32* q);

/**
 * The following methods convert a quaternion to a 3x3 matrix and return one value of this matrix.
 * Useful only if a few matrix elements are required, but very inefficient if the entire matrix is needed.
 */
float quat32_m11(Quat32* q);
float quat32_m12(Quat32* q);
float quat32_m13(Quat32* q);
float quat32_m21(Quat32* q);
float quat32_m22(Quat32* q);
float quat32_m23(Quat32* q);
float quat32_m31(Quat32* q);
float quat32_m32(Quat32* q);
float quat32_m33(Quat32* q);

#endif // QUATERNION_H
