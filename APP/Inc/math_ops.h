#ifndef MATH_OPS_H
#define MATH_OPS_H

#define SQRT3 			1.73205080757f
#define SQRT3_DIV2 	0.86602540378f
#define SQRT3_DIV3	0.57735026919f
#define PI_F 				3.1415926535897932f
#define PI_BY2_F 		6.2831853071795865f
#define PI_DIV2_F		1.5707963267948966f
#define LUT_MULT		81.4873308631f

#include "math.h"

float fast_fmaxf(float x, float y);
float fast_fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
//float roundf(float x);
void limit_norm(float *x, float *y, float limit);
void limit(float *x, float min, float max);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float sin_lut(float theta);
float cos_lut(float theta);

#endif
