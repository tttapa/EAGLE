/***********************************************************************
*   Quaternion source file
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
*   See AHRS.c for some examples
************************************************************************/

#include "quaternion.h"

void quat32_from_vector(Quat32* res, Vec32* vec) {
	float d; 
	float c; /* c = cos(sqrt(d)/2) ~= 1 - d/8 + d^2/384 */
	float s; /* s = sin(sqrt(d)/2) / sqrt(d) ~= 1/2 - d/48 + d^2/3840 */
	d = (vec->x)*(vec->x)+(vec->y)*(vec->y)+(vec->z)*(vec->z); 	/* calculate squared length */
	c = cos(sqrt(d)/2);	 /* build rotation quaternion */
	if(d==0)s=0;
	else  s = sin(sqrt(d)/2)/sqrt(d);

	res->w = c;
	res->x = vec->x*s;
	res->y = vec->y*s;
	res->z = vec->z*s;	
}

void quat32_from_vector_fast(Quat32* res, Vec32* vec) {
	res->w = sqrt(1 - (vec->x)*(vec->x)+(vec->y)*(vec->y)+(vec->z)*(vec->z));
	res->x = vec->x;
	res->y = vec->y;
	res->z = vec->z;
}

void quat32_multiply(Quat32* res, Quat32* a, Quat32* b) {
	res->w = (a->w)*(b->w) - (a->x)*(b->x) - (a->y)*(b->y) - (a->z)*(b->z);
	res->x = (a->w)*(b->x) + (a->x)*(b->w) + (a->y)*(b->z) - (a->z)*(b->y);
	res->y = (a->w)*(b->y) - (a->x)*(b->z) + (a->y)*(b->w) + (a->z)*(b->x);
	res->z = (a->w)*(b->z) + (a->x)*(b->y) - (a->y)*(b->x) + (a->z)*(b->w);
}

void quat32_conjugate(Quat32* res, Quat32* a) {
	res->w = a->w;
	res->x = -(a->x);
	res->y = -(a->y);
	res->z = -(a->z);
}

void quat32_difference(Quat32* res, Quat32* a, Quat32* b) {
	Quat32 c;
	quat32_conjugate(&c, b);
	quat32_multiply(res, a, &c);
}

void quat32_leftdivide(Quat32* res, Quat32* a, Quat32* b) {
	res->w = (a->w)*(b->w) + (a->x)*(b->x) + (a->y)*(b->y) + (a->z)*(b->z);
	res->x = (a->w)*(b->x) - (a->x)*(b->w) - (a->y)*(b->z) + (a->z)*(b->y);
	res->y = (a->w)*(b->y) + (a->x)*(b->z) - (a->y)*(b->w) - (a->z)*(b->x);
	res->z = (a->w)*(b->z) - (a->x)*(b->y) + (a->y)*(b->x) - (a->z)*(b->w);
}

void quat32_transform(Vec32* res, Quat32* q, Vec32* v) {
	float tx = (q->y) * (v->z) - (q->z)*(v->y) + (q->w)*(v->x);
	float ty = (q->z) * (v->x) - (q->x)*(v->z) + (q->w)*(v->y);
	float tz = (q->x) * (v->y) - (q->y)*(v->x) + (q->w)*(v->z);
	res->x = (q->y)*tz - (q->z)*ty + (v->x);
	res->y = (q->z)*tx - (q->x)*tz + (v->y);
	res->z = (q->x)*ty - (q->y)*tx + (v->z);
}

void quat32_invtransform(Vec32* res, Quat32* q, Vec32* v) {
	float tx = (q->y) * (v->z) - (q->z)*(v->y) - (q->w)*(v->x);
	float ty = (q->z) * (v->x) - (q->x)*(v->z) - (q->w)*(v->y);
	float tz = (q->x) * (v->y) - (q->y)*(v->x) - (q->w)*(v->z);
	res->x = (q->y)*tz - (q->z)*ty + (v->x);
	res->y = (q->z)*tx - (q->x)*tz + (v->y);
	res->z = (q->x)*ty - (q->y)*tx + (v->z);
}

void vec32_normalize_32f16(Vec32* res, Vec32* v) {
    float n = (v->x)*(v->x)+(v->y)*(v->y)+(v->z)*(v->z);
    float m = 1/sqrt(n);   
    res->x = v->x * m;//mul_32f30(v->x, m);
    res->y = v->y * m;//mul_32f30(v->y, m);
    res->z = v->z * m;//mul_32f30(v->z, m);
}

void quat32_normalize(Quat32* res, Quat32* q) {
	float n = q->w*q->w+(q->x)*(q->x)+(q->y)*(q->y)+(q->z)*(q->z);
	float m = 1/sqrt(n);
	res->w = q->w*m;
	res->x = q->x*m;
	res->y = q->y*m;
	res->z = q->z*m;
}

float quat32_m11(Quat32* q) {
	return (q->w)*(q->w) + (q->x)*(q->x)-(q->y)*(q->y)-(q->z)*(q->z);
}

float quat32_m12(Quat32* q) {
	return (q->x)*(q->y) - (q->w)*(q->z);
}

float quat32_m13(Quat32* q) {
	return (q->x)*(q->z) + (q->w)*(q->y);
}

float quat32_m21(Quat32* q) {
	return (q->x)*(q->y) + (q->w)*(q->z);
}

float quat32_m22(Quat32* q) {
	return (q->w)*(q->w) - (q->x)*(q->x) + (q->y)*(q->y) - (q->z)*(q->z);
}

float quat32_m23(Quat32* q) {
	return (q->y)*(q->z) - (q->w)*(q->x);
}

float quat32_m31(Quat32* q) {
	return (q->x)*(q->z) - (q->w)*(q->y);
}

float quat32_m32(Quat32* q) {
	return (q->y)*(q->z) + (q->w)*(q->x);
}

float quat32_m33(Quat32* q) {
	return (q->w)*(q->w) - (q->x)*(q->x) - (q->y)*(q->y) + (q->z)*(q->z);
}
