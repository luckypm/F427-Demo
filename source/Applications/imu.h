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

#ifndef _imu_h
#define _imu_h

#include "aq.h"
//#include "adc.h"
#include "d_imu.h"

#define IMU_ROOM_TEMP		20.0f
#define IMU_STATIC_STD		0.05f
#define IMU_STATIC_TIMEOUT	5	// seconds

// these define where to get certain data
#define AQ_YAW			navUkfData.yaw
#define AQ_PITCH		navUkfData.pitch
#define AQ_ROLL			navUkfData.roll
#define AQ_PRES_ADJ		UKF_PRES_ALT

#ifdef USE_DIGITAL_IMU
// using the Digital IMU as IMU
#define IMU_DRATEX		mpu6000Data.dRateGyo[0]//用的就是这个传感器，包含陀螺仪和加速度计 
#define IMU_DRATEY		mpu6000Data.dRateGyo[1]
#define IMU_DRATEZ		mpu6000Data.dRateGyo[2]
#define IMU_RATEX		mpu6000Data.gyo[0]
#define IMU_RATEY		mpu6000Data.gyo[1]
#define IMU_RATEZ		mpu6000Data.gyo[2]
#define IMU_RAW_RATEX   mpu6000Data.rawGyo[0]
#define IMU_RAW_RATEY   mpu6000Data.rawGyo[1]
#define IMU_RAW_RATEZ   mpu6000Data.rawGyo[2]
#define IMU_ACCX		mpu6000Data.acc[0]
#define IMU_ACCY		mpu6000Data.acc[1]
#define IMU_ACCZ		mpu6000Data.acc[2]
#define IMU_RAW_ACCX    mpu6000Data.rawAcc[0]
#define IMU_RAW_ACCY    mpu6000Data.rawAcc[1]
#define IMU_RAW_ACCZ    mpu6000Data.rawAcc[2]

#define IMU_MAGX		hmc5983Data.mag[0]//地磁用的是这个
#define IMU_MAGY		hmc5983Data.mag[1]
#define IMU_MAGZ		hmc5983Data.mag[2]
#define IMU_RAW_MAGX    hmc5983Data.rawMag[0]
#define IMU_RAW_MAGY    hmc5983Data.rawMag[1]
#define IMU_RAW_MAGZ    hmc5983Data.rawMag[2]
#define IMU_TEMP		dImuData.temp
#define IMU_LASTUPD		dImuData.lastUpdate
#define AQ_OUTER_TIMESTEP	DIMU_OUTER_DT
#define AQ_INNER_TIMESTEP	DIMU_INNER_DT
#define AQ_PRESSURE		ms5611Data.pres
#define AQ_MAG_ENABLED          hmc5983Data.enabled
#endif	// USE_DIGITAL_IMU


typedef struct {
    //OS_FlagID dRateFlag;
    //OS_FlagID sensorFlag;
    float sinRot, cosRot;
    uint32_t fullUpdates;
    uint32_t halfUpdates;
} imuStruct_t;

extern imuStruct_t imuData;

extern void imuInit(void);
extern void imuQuasiStatic(int n);
//extern void imuAdcDRateReady(void);
//extern void imuAdcSensorReady(void);
extern void imuDImuDRateReady(void);
extern void imuDImuSensorReady(void);

#endif
