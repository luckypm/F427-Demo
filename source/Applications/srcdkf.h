/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011-2014  Bill Nesbitt
*/

#ifndef _srcdkf_h
#define _srcdkf_h

#include "aq_math.h"

#define SRCDKF_H	(__sqrtf(3.0f) * 3.0f)
#define SRCDKF_RM	0.0001f		// Robbins-Monro stochastic term

#define MAX(a, b)	((a > b) ? a : b)

typedef void SRCDKFTimeUpdate_t(float32_t *x_I, float32_t *noise_I, float32_t *x_O, float32_t *u, float32_t dt, int n);
typedef void SRCDKFMeasurementUpdate_t(float32_t *u, float32_t *x, float32_t *noise_I, float32_t *y);//ÕâÖÖ¶¨Òå·¨ºÜÉÙ¼û
//Èç¹û¶¨ÒåÈçÏÂ: SRCDKFMeasurementUpdate_t abc;Ôò¶¨ÒåÁËÒ»¸öÃûÎªabcµÄº¯Êı£¬ÀàËÆÓÚ void abc(float32_t *u, float32_t *x, float32_t *noise_I, float32_t *y);
//Èç¹û¶¨ÒåÈçÏÂ: SRCDKFMeasurementUpdate_t *p;Ôò¶¨ÒåÁËÒ»¸öº¯ÊıÖ¸Õë£¬¸Ãº¯ÊıÖ¸ÕëÖ¸ÕëÒ»¸ö·µ»ØÖµÎªvoidÀàĞÍ£¬ĞÎ²ÎÎª4¸öfloatĞÍµÄº¯Êı;ÓÃ¸ÃÖ¸Õëµ÷ÓÃº¯ÊıÊ±Ç°Ãæ²»ĞèÒª¼Ó*

// define all temporary storage here so that it does not need to be allocated each iteration
typedef struct {
	int S;
	int V;
	int M;		// only used for parameter estimation
	int N;		// only used for parameter estimation
	int L;

	float32_t h;
	float32_t hh;
	float32_t w0m, wim, wic1, wic2;
	float32_t rm;

	arm_matrix_instance_f32 Sx;	// state covariance//×´Ì¬Ğ­·½²î¾ØÕó
	arm_matrix_instance_f32 SxT;	// Sx transposed//×´Ì¬Ğ­·½²î×ªÖÃ¾ØÕó
	arm_matrix_instance_f32 Sv;	// process noise//¹ı³ÌÔëÉù¾ØÕó
	arm_matrix_instance_f32 Sn;	// observation noise//¹Û²â/²âÁ¿ÔëÉù
	arm_matrix_instance_f32 x;	// state estimate vector//×´Ì¬¹À¼ÆÏòÁ¿ UKF_POSNµÈÊı¾İ¾ÍÊÇÀ´×ÔÓÚËü
	arm_matrix_instance_f32 Xa;	// augmented sigma points//À©Õ¹sigmaµã
	float32_t *xIn;
	float32_t *xNoise;
	float32_t *xOut;
	arm_matrix_instance_f32 qrTempS;//¸úQRÓĞ¹ØµÄÖĞ¼äÁ¿?
	arm_matrix_instance_f32 Y;	// resultant measurements from sigma points
	arm_matrix_instance_f32 y;	// measurement estimate vector//²âÁ¿¹À¼ÆÏòÁ¿£¬×¢ÊÍÖĞ´øvectorµÄ¶¼ÊÇÏòÁ¿£¬¼´Ö»ÓĞÒ»ÁĞµÄ¾ØÕó
	arm_matrix_instance_f32 qrTempM;
	arm_matrix_instance_f32 Sy;	// measurement covariance//²âÁ¿Ğ­·½²î
	arm_matrix_instance_f32 SyT;	// Sy transposed//²âÁ¿Ğ­·½²î×ªÖÃ
	arm_matrix_instance_f32 SyC;	// copy of Sy//¸´ÖÆ²âÁ¿Ğ­·½²î
	arm_matrix_instance_f32 Pxy;
	arm_matrix_instance_f32 C1;
	arm_matrix_instance_f32 C1T;
	arm_matrix_instance_f32 C2;
	arm_matrix_instance_f32 D;
	arm_matrix_instance_f32 K;//¿¨¶ûÂüÂË²¨ÔöÒæ
	arm_matrix_instance_f32 KT;	// only used for param est
	arm_matrix_instance_f32 inov;	// inovation//¸üĞÂ
	arm_matrix_instance_f32 inovT;// only used for param est
	arm_matrix_instance_f32 xUpdate;
	arm_matrix_instance_f32 qrFinal;//QR·Ö½âÒò×Ó?¾ØÕóÀíÂÛÖĞÓĞÒ»ÖÖQR·Ö½âÀíÂÛ£¬²»ÁË½â
	arm_matrix_instance_f32 rDiag;
	arm_matrix_instance_f32 Q, R, AQ;	// scratch

	SRCDKFTimeUpdate_t *timeUpdate;//¶¨ÒåÁËÒ»¸öº¯ÊıÖ¸Õë
	SRCDKFMeasurementUpdate_t *map;	// only used for param est//Í¬ÉÏ
} srcdkf_t;

extern srcdkf_t *srcdkfInit(int s, int m, int v, int n, SRCDKFTimeUpdate_t *timeUpdate);
extern float *srcdkfGetState(srcdkf_t *f);
extern void srcdkfSetVariance(srcdkf_t *f, float32_t *q, float32_t *v, float32_t *n, int nn);
extern void srcdkfGetVariance(srcdkf_t *f, float32_t *q);
extern void srcdkfTimeUpdate(srcdkf_t *f, float32_t *u, float32_t dt);
extern void srcdkfMeasurementUpdate(srcdkf_t *f, float32_t *u, float32_t *y, int M, int N, float32_t *noise, SRCDKFMeasurementUpdate_t *measurementUpdate);
extern void srcdkfFree(srcdkf_t *f);
extern srcdkf_t *paramsrcdkfInit(int w, int d, int n, SRCDKFMeasurementUpdate_t *map);
extern void paramsrcdkfUpdate(srcdkf_t *f, float32_t *u, float32_t *d);
extern void paramsrcdkfSetVariance(srcdkf_t *f, float32_t *v, float32_t *n);
extern void paramsrcdkfGetVariance(srcdkf_t *f, float32_t *v, float32_t *n);
extern void paramsrcdkfSetRM(srcdkf_t *f, float32_t rm);

#endif
