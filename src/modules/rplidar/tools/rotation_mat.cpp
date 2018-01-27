/*
 * File: rotation_mat.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 27-Jan-2018 19:38:53
 */

/* Include Files */
#include "rotation_mat.h"

#define NumBitsPerChar	8U

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;

/* Function Definitions */

/*
 * 输入弧度数据
 *  a: 横滚角
 *  b: 俯仰角
 *  y: 偏航角
 * Arguments    : double x0
 *                double b_y0
 *                double z0
 *                double a
 *                double b
 *                double y
 *                double *x_out
 *                double *y_out
 *                double *z_out
 * Return Type  : void
 */
void rotation_mat(double x0, double b_y0, double z0, double a, double b, double
				  y, double *x_out, double *y_out, double *z_out)
{
  double dv0[9];
  double b_x0[3];
  int i0;
  double r[3];
  int i1;
  dv0[0] = cos(b) * cos(y);
  dv0[3] = cos(a) * sin(y) - sin(a) * sin(b) * cos(y);
  dv0[6] = sin(a) * sin(y) - cos(a) * sin(b) * cos(y);
  dv0[1] = -cos(b) * sin(y);
  dv0[4] = cos(a) * cos(y) - sin(a) * sin(b) * sin(y);
  dv0[7] = sin(a) * cos(y) + cos(a) * sin(b) * sin(y);
  dv0[2] = sin(b);
  dv0[5] = -sin(a) * cos(b);
  dv0[8] = cos(a) * cos(b);
  b_x0[0] = x0;
  b_x0[1] = b_y0;
  b_x0[2] = z0;
  for (i0 = 0; i0 < 3; i0++) {
	r[i0] = 0.0;
	for (i1 = 0; i1 < 3; i1++) {
	  r[i0] += dv0[i0 + 3 * i1] * b_x0[i1];
	}
  }

  *x_out = r[0];
  *y_out = r[1];
  *z_out = r[2];
}

/*
* 输入弧度数据
*  a: 横滚角
*  b: 俯仰角
*  y: 偏航角
* Arguments    : double x0
*                double b_y0
*                double z0
*                double a
*                double b
*                double y
*                double *x_out
*                double *y_out
*                double *z_out
* Return Type  : void
*/
void rotation_mat_inv(double x0, double b_y0, double z0, double a, double b,
	double y, double *x_out, double *y_out, double *z_out)
{
	double dv1[9];
	double rotation[9];
	double b_x0[3];
	int i2;
	double r[3];
	int i3;
	dv1[0] = cos(b) * cos(y);
	dv1[3] = cos(a) * sin(y) - sin(a) * sin(b) * cos(y);
	dv1[6] = sin(a) * sin(y) - cos(a) * sin(b) * cos(y);
	dv1[1] = -cos(b) * sin(y);
	dv1[4] = cos(a) * cos(y) - sin(a) * sin(b) * sin(y);
	dv1[7] = sin(a) * cos(y) + cos(a) * sin(b) * sin(y);
	dv1[2] = sin(b);
	dv1[5] = -sin(a) * cos(b);
	dv1[8] = cos(a) * cos(b);
	inv(dv1, rotation);
	b_x0[0] = x0;
	b_x0[1] = b_y0;
	b_x0[2] = z0;
	for (i2 = 0; i2 < 3; i2++) {
		r[i2] = 0.0;
		for (i3 = 0; i3 < 3; i3++) {
			r[i2] += rotation[i2 + 3 * i3] * b_x0[i3];
		}
	}

	*x_out = r[0];
	*y_out = r[1];
	*z_out = r[2];
}

/*
* Arguments    : const double x[9]
*                double y[9]
* Return Type  : void
*/
void inv(const double x[9], double y[9])
{
	double b_x[9];
	int p1;
	int p2;
	int p3;
	double absx11;
	double absx21;
	double absx31;
	int itmp;
	double b_y;
	memcpy(&b_x[0], &x[0], 9U * sizeof(double));
	p1 = 0;
	p2 = 3;
	p3 = 6;
	absx11 = fabs(x[0]);
	absx21 = fabs(x[1]);
	absx31 = fabs(x[2]);
	if ((absx21 > absx11) && (absx21 > absx31)) {
		p1 = 3;
		p2 = 0;
		b_x[0] = x[1];
		b_x[1] = x[0];
		b_x[3] = x[4];
		b_x[4] = x[3];
		b_x[6] = x[7];
		b_x[7] = x[6];
	}
	else {
		if (absx31 > absx11) {
			p1 = 6;
			p3 = 0;
			b_x[0] = x[2];
			b_x[2] = x[0];
			b_x[3] = x[5];
			b_x[5] = x[3];
			b_x[6] = x[8];
			b_x[8] = x[6];
		}
	}

	absx21 = b_x[1] / b_x[0];
	b_x[1] /= b_x[0];
	absx11 = b_x[2] / b_x[0];
	b_x[2] /= b_x[0];
	b_x[4] -= absx21 * b_x[3];
	b_x[5] -= absx11 * b_x[3];
	b_x[7] -= absx21 * b_x[6];
	b_x[8] -= absx11 * b_x[6];
	if (fabs(b_x[5]) > fabs(b_x[4])) {
		itmp = p2;
		p2 = p3;
		p3 = itmp;
		b_x[1] = absx11;
		b_x[2] = absx21;
		absx11 = b_x[4];
		b_x[4] = b_x[5];
		b_x[5] = absx11;
		absx11 = b_x[7];
		b_x[7] = b_x[8];
		b_x[8] = absx11;
	}

	absx31 = b_x[5];
	b_y = b_x[4];
	absx21 = b_x[5] / b_x[4];
	b_x[8] -= absx21 * b_x[7];
	absx11 = (absx21 * b_x[1] - b_x[2]) / b_x[8];
	absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
	y[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
	y[p1 + 1] = absx21;
	y[p1 + 2] = absx11;
	absx11 = -(absx31 / b_y) / b_x[8];
	absx21 = (1.0 - b_x[7] * absx11) / b_x[4];
	y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
	y[p2 + 1] = absx21;
	y[p2 + 2] = absx11;
	absx11 = 1.0 / b_x[8];
	absx21 = -b_x[7] * absx11 / b_x[4];
	y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
	y[p3 + 1] = absx21;
	y[p3 + 2] = absx11;
}


/* Function: rtGetInf ==================================================
* Abstract:
* Initialize rtInf needed by the generated code.
* Inf is initialized as non-signaling. Assumes IEEE.
*/
real_T rtGetInf(void)
{
	size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
	real_T inf = 0.0;
	if (bitsPerReal == 32U) {
		inf = rtGetInfF();
	}
	else {
		uint16_T one = 1U;
		enum {
			LittleEndian,
			BigEndian
		} machByteOrder = (*((uint8_T *)&one) == 1U) ? LittleEndian : BigEndian;
		switch (machByteOrder) {
		case LittleEndian:
		{
			union {
				LittleEndianIEEEDouble bitVal;
				real_T fltVal;
			} tmpVal;

			tmpVal.bitVal.words.wordH = 0x7FF00000U;
			tmpVal.bitVal.words.wordL = 0x00000000U;
			inf = tmpVal.fltVal;
			break;
		}

		case BigEndian:
		{
			union {
				BigEndianIEEEDouble bitVal;
				real_T fltVal;
			} tmpVal;

			tmpVal.bitVal.words.wordH = 0x7FF00000U;
			tmpVal.bitVal.words.wordL = 0x00000000U;
			inf = tmpVal.fltVal;
			break;
		}
		}
	}

	return inf;
}

/* Function: rtGetInfF ==================================================
* Abstract:
* Initialize rtInfF needed by the generated code.
* Inf is initialized as non-signaling. Assumes IEEE.
*/
real32_T rtGetInfF(void)
{
	IEEESingle infF;
	infF.wordL.wordLuint = 0x7F800000U;
	return infF.wordL.wordLreal;
}

/* Function: rtGetMinusInf ==================================================
* Abstract:
* Initialize rtMinusInf needed by the generated code.
* Inf is initialized as non-signaling. Assumes IEEE.
*/
real_T rtGetMinusInf(void)
{
	size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
	real_T minf = 0.0;
	if (bitsPerReal == 32U) {
		minf = rtGetMinusInfF();
	}
	else {
		uint16_T one = 1U;
		enum {
			LittleEndian,
			BigEndian
		} machByteOrder = (*((uint8_T *)&one) == 1U) ? LittleEndian : BigEndian;
		switch (machByteOrder) {
		case LittleEndian:
		{
			union {
				LittleEndianIEEEDouble bitVal;
				real_T fltVal;
			} tmpVal;

			tmpVal.bitVal.words.wordH = 0xFFF00000U;
			tmpVal.bitVal.words.wordL = 0x00000000U;
			minf = tmpVal.fltVal;
			break;
		}

		case BigEndian:
		{
			union {
				BigEndianIEEEDouble bitVal;
				real_T fltVal;
			} tmpVal;

			tmpVal.bitVal.words.wordH = 0xFFF00000U;
			tmpVal.bitVal.words.wordL = 0x00000000U;
			minf = tmpVal.fltVal;
			break;
		}
		}
	}

	return minf;
}

/* Function: rtGetMinusInfF ==================================================
* Abstract:
* Initialize rtMinusInfF needed by the generated code.
* Inf is initialized as non-signaling. Assumes IEEE.
*/
real32_T rtGetMinusInfF(void)
{
	IEEESingle minfF;
	minfF.wordL.wordLuint = 0xFF800000U;
	return minfF.wordL.wordLreal;
}


/* Function: rtGetNaN ==================================================
* Abstract:
* Initialize rtNaN needed by the generated code.
* NaN is initialized as non-signaling. Assumes IEEE.
*/
real_T rtGetNaN(void)
{
	size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
	real_T nan = 0.0;
	if (bitsPerReal == 32U) {
		nan = rtGetNaNF();
	}
	else {
		uint16_T one = 1U;
		enum {
			LittleEndian,
			BigEndian
		} machByteOrder = (*((uint8_T *)&one) == 1U) ? LittleEndian : BigEndian;
		switch (machByteOrder) {
		case LittleEndian:
		{
			union {
				LittleEndianIEEEDouble bitVal;
				real_T fltVal;
			} tmpVal;

			tmpVal.bitVal.words.wordH = 0xFFF80000U;
			tmpVal.bitVal.words.wordL = 0x00000000U;
			nan = tmpVal.fltVal;
			break;
		}

		case BigEndian:
		{
			union {
				BigEndianIEEEDouble bitVal;
				real_T fltVal;
			} tmpVal;

			tmpVal.bitVal.words.wordH = 0x7FFFFFFFU;
			tmpVal.bitVal.words.wordL = 0xFFFFFFFFU;
			nan = tmpVal.fltVal;
			break;
		}
		}
	}

	return nan;
}

/* Function: rtGetNaNF ==================================================
* Abstract:
* Initialize rtNaNF needed by the generated code.
* NaN is initialized as non-signaling. Assumes IEEE.
*/
real32_T rtGetNaNF(void)
{
	IEEESingle nanF = { { 0 } };
	uint16_T one = 1U;
	enum {
		LittleEndian,
		BigEndian
	} machByteOrder = (*((uint8_T *)&one) == 1U) ? LittleEndian : BigEndian;
	switch (machByteOrder) {
	case LittleEndian:
	{
		nanF.wordL.wordLuint = 0xFFC00000U;
		break;
	}

	case BigEndian:
	{
		nanF.wordL.wordLuint = 0x7FFFFFFFU;
		break;
	}
	}

	return nanF.wordL.wordLreal;
}

/* Function: rt_InitInfAndNaN ==================================================
* Abstract:
* Initialize the rtInf, rtMinusInf, and rtNaN needed by the
* generated code. NaN is initialized as non-signaling. Assumes IEEE.
*/
void rt_InitInfAndNaN(size_t realSize)
{
	(void)(realSize);
	rtNaN = rtGetNaN();
	rtNaNF = rtGetNaNF();
	rtInf = rtGetInf();
	rtInfF = rtGetInfF();
	rtMinusInf = rtGetMinusInf();
	rtMinusInfF = rtGetMinusInfF();
}

/* Function: rtIsInf ==================================================
* Abstract:
* Test if value is infinite
*/
boolean_T rtIsInf(real_T value)
{
	return ((value == rtInf || value == rtMinusInf) ? 1U : 0U);
}

/* Function: rtIsInfF =================================================
* Abstract:
* Test if single-precision value is infinite
*/
boolean_T rtIsInfF(real32_T value)
{
	return(((value) == rtInfF || (value) == rtMinusInfF) ? 1U : 0U);
}

/* Function: rtIsNaN ==================================================
* Abstract:
* Test if value is not a number
*/
boolean_T rtIsNaN(real_T value)
{
#if defined(_MSC_VER) && (_MSC_VER <= 1200)
	return _isnan(value) ? TRUE : FALSE;
#else
	return (value != value) ? 1U : 0U;
#endif
}

/* Function: rtIsNaNF =================================================
* Abstract:
* Test if single-precision value is not a number
*/
boolean_T rtIsNaNF(real32_T value)
{
#if defined(_MSC_VER) && (_MSC_VER <= 1200)
	return _isnan((real_T)value) ? true : false;
#else
	return (value != value) ? 1U : 0U;
#endif
}

/*
 * File trailer for rotation_mat.c
 *
 * [EOF]
 */
