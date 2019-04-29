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

    Copyright ?2011-2014  Bill Nesbitt
*/

#ifndef _nav_ukf_h
#define _nav_ukf_h

#include "aq.h"
#include "srcdkf.h"

#define NAV_MIN_GPS_ACC		3.0f					    // minimum gps hAcc needed to enter auto nav modes, in meters
#define NAV_MAX_GPS_AGE		1e6					    // maximum age of position update needed to enter auto nav modes, in microseconds
#define NAV_MIN_FIX_ACC		4.0f					    // minimum gps hAcc still considered a valid "2D" fix, in meters
#define NAV_MAX_FIX_AGE		10e6					    // maximum age of position update still considered a valid "2D" fix, in microseconds

#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)			    // meters//赤道半径，单位为m
#define NAV_FLATTENING		(1.0 / 298.257223563)			    // WGS-84//地心坐标系(一种为GPS定位系统而建立的坐标系)地球的扁率=(长半轴-短半轴)/长半轴，这里长半轴就是赤道半径
#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))//地球偏心率的平方

#define UKF_LOG_SIZE		(17*sizeof(float))
#define UKF_LOG_BUF_SIZE	(UKF_LOG_SIZE*40)
//#define UKF_LOG_FNAME		"UKF"		// comment out to disable logging

#define SIM_S                   17		// states//状态量
#define SIM_M                   3		// max measurements//最大测量量
#define SIM_V                   12//16		// process noise//过程噪声量
#define SIM_N                   3		// max observation noise//最大观察噪声量

#define UKF_GYO_AVG_NUM		40

#define UKF_STATE_VELN		0
#define UKF_STATE_VELE		1
#define UKF_STATE_VELD		2
#define UKF_STATE_POSN		3
#define UKF_STATE_POSE		4
#define UKF_STATE_POSD		5
#define UKF_STATE_ACC_BIAS_X	6
#define UKF_STATE_ACC_BIAS_Y	7
#define UKF_STATE_ACC_BIAS_Z	8
#define UKF_STATE_GYO_BIAS_X	9
#define UKF_STATE_GYO_BIAS_Y	10
#define UKF_STATE_GYO_BIAS_Z	11
#define UKF_STATE_Q1		12
#define UKF_STATE_Q2		13
#define UKF_STATE_Q3		14
#define UKF_STATE_Q4		15
#define UKF_STATE_PRES_ALT	16

#define UKF_V_NOISE_ACC_BIAS_X	0
#define UKF_V_NOISE_ACC_BIAS_Y	1
#define UKF_V_NOISE_ACC_BIAS_Z	2
#define UKF_V_NOISE_GYO_BIAS_X	3
#define UKF_V_NOISE_GYO_BIAS_Y	4
#define UKF_V_NOISE_GYO_BIAS_Z	5
#define UKF_V_NOISE_RATE_X	6
#define UKF_V_NOISE_RATE_Y	7
#define UKF_V_NOISE_RATE_Z	8
#define UKF_V_NOISE_VELN	9
#define UKF_V_NOISE_VELE	10
#define UKF_V_NOISE_VELD	11

#define UKF_VELN		navUkfData.x[UKF_STATE_VELN]//0
#define UKF_VELE		navUkfData.x[UKF_STATE_VELE]// 1
#define UKF_VELD		navUkfData.x[UKF_STATE_VELD]// 2
#define UKF_POSN		navUkfData.x[UKF_STATE_POSN]// 3
#define UKF_POSE		navUkfData.x[UKF_STATE_POSE]// 4
#define UKF_POSD		navUkfData.x[UKF_STATE_POSD]// 5
#define UKF_ACC_BIAS_X		navUkfData.x[UKF_STATE_ACC_BIAS_X]// 6
#define UKF_ACC_BIAS_Y		navUkfData.x[UKF_STATE_ACC_BIAS_Y]// 7
#define UKF_ACC_BIAS_Z		navUkfData.x[UKF_STATE_ACC_BIAS_Z]// 8
#define UKF_GYO_BIAS_X		navUkfData.x[UKF_STATE_GYO_BIAS_X]// 9
#define UKF_GYO_BIAS_Y		navUkfData.x[UKF_STATE_GYO_BIAS_Y]// 10
#define UKF_GYO_BIAS_Z		navUkfData.x[UKF_STATE_GYO_BIAS_Z]// 11
#define UKF_Q1			navUkfData.x[UKF_STATE_Q1]// 12
#define UKF_Q2			navUkfData.x[UKF_STATE_Q2]// 13
#define UKF_Q3			navUkfData.x[UKF_STATE_Q3]// 14
#define UKF_Q4			navUkfData.x[UKF_STATE_Q4]// 15
#define UKF_PRES_ALT		navUkfData.x[UKF_STATE_PRES_ALT]// 16

#ifdef USE_PRES_ALT
#define UKF_ALTITUDE	UKF_PRES_ALT//由气压计计算出来的海拔
#else
#define UKF_ALTITUDE	UKF_POSD//由GPS高度计算出来的海拔
#endif

#define UKF_HIST		40
#define UKF_P0			101325.0f			    // standard static pressure at sea level

#define UKF_FLOW_ROT		-90.0f				    // optical flow mounting rotation in degrees
#define UKF_FOCAL_LENGTH	16.0f				    // 16mm
#define UKF_FOCAL_PX		(UKF_FOCAL_LENGTH / (4.0f * 6.0f) * 1000.0f)   // pixel size: 6um, binning 4 enabled

typedef struct {
    srcdkf_t *kf;
    float v0a[3];
    float v0m[3];
    double holdLat, holdLon;
    double r1, r2;
    float posN[UKF_HIST];//UKF_HIST=40
    float posE[UKF_HIST];
    float posD[UKF_HIST];
    float velN[UKF_HIST];
    float velE[UKF_HIST];
    float velD[UKF_HIST];
    int navHistIndex;
    float yaw, pitch, roll;
    float yawCos, yawSin;
    float *x;			// states
    float flowSumX, flowSumY;
    int32_t flowSumQuality;
    float flowSumAlt;
    float flowVelX, flowVelY;
    float flowPosN, flowPosE;
    float flowQuality;
    float flowAlt;
    float flowRotCos, flowRotSin;
    uint32_t flowCount, flowAltCount;
    int logPointer;
    volatile uint8_t flowLock;
    uint8_t flowInit;
    uint8_t logHandle;
} navUkfStruct_t;

extern navUkfStruct_t navUkfData;

extern void navUkfInit(void);
extern void navUkfInertialUpdate(void);
extern void simDoPresUpdate(float pres);
extern void simDoAccUpdate(float accX, float accY, float accZ);
extern void simDoMagUpdate(float magX, float magY, float magZ);
extern void navUkfGpsPosUpdate(uint32_t gpsMicros, double lat, double lon, float alt, float hAcc, float vAcc);
extern void navUkfGpsVelUpdate(uint32_t gpsMicros, float velN, float velE, float velD, float sAcc);
extern void navUkfFlowUpdate(void);
extern void navUkfOpticalFlow(int16_t x, int16_t y, uint8_t quality, float ground);
extern void navUkfSetGlobalPositionTarget(double lat, double lon);
extern void navUkfSetHereAsPositionTarget(void);
extern void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll);
extern void navUkfZeroRate(float zRate, int axis);
extern void navUkfFinish(void);
extern void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q);
extern void navUkfResetBias(void);
extern void navUkfResetVels(void);
extern void navUkfZeroPos(void);
extern void navUkfZeroVel(void);
extern void navUkfRotateVectorByQuat(float *vr, float *v, float *q);
extern float navUkfPresToAlt(float pressure);

#endif
