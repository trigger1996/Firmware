/*
 * File: rotation_mat.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 27-Jan-2018 19:38:53
 */

#ifndef __ROTATION_MAT_H__
#define __ROTATION_MAT_H__

#pragma GCC diagnostic ignored "-Wfloat-equal"

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"

#if defined(_MSC_VER) && (_MSC_VER <= 1200)
#include <float.h>
#endif
#include <stddef.h>
#include "rtwtypes.h"

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
extern void rt_InitInfAndNaN(size_t realSize);
extern boolean_T rtIsInf(real_T value);
extern boolean_T rtIsInfF(real32_T value);
extern boolean_T rtIsNaN(real_T value);
extern boolean_T rtIsNaNF(real32_T value);

typedef struct {
	struct {
		uint32_T wordH;
		uint32_T wordL;
	} words;
} BigEndianIEEEDouble;

typedef struct {
	struct {
		uint32_T wordL;
		uint32_T wordH;
	} words;
} LittleEndianIEEEDouble;

typedef struct {
	union {
		real32_T wordLreal;
		uint32_T wordLuint;
	} wordL;
} IEEESingle;


/* Function Declarations */
#ifdef __cplusplus

extern "C" {

#endif

  extern void rotation_mat(double x0, double b_y0, double z0, double a, double b,
	double y, double *x_out, double *y_out, double *z_out);

  extern void rotation_mat_inv(double x0, double b_y0, double z0, double a,
	  double b, double y, double *x_out, double *y_out, double *z_out);

  extern void inv(const double x[9], double y[9]);


  extern real_T rtGetNaN(void);
  extern real32_T rtGetNaNF(void);

  extern real_T rtGetInf(void);
  extern real32_T rtGetInfF(void);
  extern real_T rtGetMinusInf(void);
  extern real32_T rtGetMinusInfF(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for rotation_mat.h
 *
 * [EOF]
 */
