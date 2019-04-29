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

#include "aq.h"
#include "run.h"
#include "comm.h"
#include "nav_ukf.h"
#include "imu.h"
#include "gps.h"
//#include "nav.h"
//#include "control.h"
//#include "logger.h"
#include "supervisor.h"
//#include "gimbal.h"
//#include "analog.h"
#include "config.h"
//#include "aq_timer.h"
//#include "aq_mavlink.h"
//#include "calib.h"
#include "alt_ukf.h"
//#include <CoOS.h>
#ifndef __CC_ARM
#include <intrinsics.h>
#endif

TaskHandle_t runTaskHandle;


runStruct_t runData __attribute__((section(".ccm")));

void runTaskCode(void *unused) {
    uint32_t axis = 0;
    uint32_t loops = 0;

    AQ_NOTICE("Run task started\n");

    while (1) {
	// wait for data
	 xEventGroupWaitBits((EventGroupHandle_t)EventGroupHandler,		
	 					(EventBits_t        )sensorFlag,
	 					(BaseType_t			)pdTRUE,				
	 					(BaseType_t			)pdTRUE,
	 			        (TickType_t			)portMAX_DELAY);

	// soft start GPS accuracy
	runData.accMask *= 0.999f;

	navUkfInertialUpdate();

	// record history for acc & mag & pressure readings for smoothing purposes
	// acc
	runData.sumAcc[0] -= runData.accHist[0][runData.sensorHistIndex];
	runData.sumAcc[1] -= runData.accHist[1][runData.sensorHistIndex];
	runData.sumAcc[2] -= runData.accHist[2][runData.sensorHistIndex];

	runData.accHist[0][runData.sensorHistIndex] = IMU_ACCX;
	runData.accHist[1][runData.sensorHistIndex] = IMU_ACCY;
	runData.accHist[2][runData.sensorHistIndex] = IMU_ACCZ;

	runData.sumAcc[0] += runData.accHist[0][runData.sensorHistIndex];//累加最近的10次数据
	runData.sumAcc[1] += runData.accHist[1][runData.sensorHistIndex];//累加最近的10次数据
	runData.sumAcc[2] += runData.accHist[2][runData.sensorHistIndex];//累加最近的10次数据

	// mag
	runData.sumMag[0] -= runData.magHist[0][runData.sensorHistIndex];
	runData.sumMag[1] -= runData.magHist[1][runData.sensorHistIndex];
	runData.sumMag[2] -= runData.magHist[2][runData.sensorHistIndex];

	runData.magHist[0][runData.sensorHistIndex] = IMU_MAGX;
	runData.magHist[1][runData.sensorHistIndex] = IMU_MAGY;
	runData.magHist[2][runData.sensorHistIndex] = IMU_MAGZ;

	runData.sumMag[0] += runData.magHist[0][runData.sensorHistIndex];//累加最近的10次数据
	runData.sumMag[1] += runData.magHist[1][runData.sensorHistIndex];//累加最近的10次数据
	runData.sumMag[2] += runData.magHist[2][runData.sensorHistIndex];//累加最近的10次数据

	// pressure
	runData.sumPres -= runData.presHist[runData.sensorHistIndex];//当第10次的数据进来时，减掉了第0次的数据，第11次的数据进来时，减掉了第1次的数据
	runData.presHist[runData.sensorHistIndex] = AQ_PRESSURE;
	runData.sumPres += runData.presHist[runData.sensorHistIndex];

	runData.sensorHistIndex = (runData.sensorHistIndex + 1) % RUN_SENSOR_HIST;//0 1 2 3 。。。9 0 1 2 3。。。

	if (!((loops+1) % 20)) {//19 39 59 79 99
	   simDoAccUpdate(runData.sumAcc[0]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumAcc[1]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumAcc[2]*(1.0f / (float)RUN_SENSOR_HIST));
	}
	else if (!((loops+7) % 20)) {//13 33 53 73 93 。。。
	   simDoPresUpdate(runData.sumPres*(1.0f / (float)RUN_SENSOR_HIST));//10次的数据累加
	}
#ifndef USE_DIGITAL_IMU
	else if (!((loops+13) % 20) && AQ_MAG_ENABLED) {
		
	   simDoMagUpdate(runData.sumMag[0]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumMag[1]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumMag[2]*(1.0f / (float)RUN_SENSOR_HIST));
	}
#endif
	// optical flow update
	else if (navUkfData.flowCount >= 10 && !navUkfData.flowLock) {
	    navUkfFlowUpdate();
	}
	// only accept GPS updates if there is no optical flow
	else if ((xEventGroupGetBits(EventGroupHandler) & gpsPosFlag)== 1 && navUkfData.flowQuality == 0.0f && gpsData.hAcc < NAV_MIN_GPS_ACC && gpsData.tDOP != 0.0f) {
	    navUkfGpsPosUpdate(gpsData.lastPosUpdate, gpsData.lat, gpsData.lon, gpsData.height, gpsData.hAcc + runData.accMask, gpsData.vAcc + runData.accMask);//分析数据发现，位置的突变后，调用这个函数后会影响UKF_VELN UKF_VELE UKF_VELD，这个影响比只有GPS速度的突变(这个通常不存在，因为速度突变了一般位置也就突变了)后，进入速度更新函数对UKF_POSN E D的影响更大
	    xEventGroupClearBits(EventGroupHandler, gpsPosFlag);
	    // refine static sea level pressure based on better GPS altitude fixes
	    if (gpsData.hAcc < runData.bestHacc && gpsData.hAcc < NAV_MIN_GPS_ACC) {
                navPressureAdjust(gpsData.height);//用GPS高度减掉UKF高度得出一个高度偏移量
		runData.bestHacc = gpsData.hAcc;
	    }
	}
	else if ((xEventGroupGetBits(EventGroupHandler) & gpsVelFlag)== 1 && navUkfData.flowQuality == 0.0f && gpsData.sAcc < NAV_MIN_GPS_ACC/2 && gpsData.tDOP != 0.0f) {
	    navUkfGpsVelUpdate(gpsData.lastVelUpdate, gpsData.velN, gpsData.velE, gpsData.velD, gpsData.sAcc + runData.accMask);
	    xEventGroupClearBits(EventGroupHandler, gpsVelFlag);
	}
	// observe zero position
	else if (!((loops+4) % 20) && (gpsData.hAcc >= NAV_MIN_GPS_ACC || gpsData.tDOP == 0.0f) && navUkfData.flowQuality == 0.0f) {
	    navUkfZeroPos();
	}
	// observer zero velocity
	else if (!((loops+10) % 20) && (gpsData.sAcc >= NAV_MIN_GPS_ACC/2 || gpsData.tDOP == 0.0f) && navUkfData.flowQuality == 0.0f) {
	    navUkfZeroVel();
	}
	// observe that the rates are exactly 0 if not flying or moving
	else if (!(supervisorData.state & STATE_FLYING)) {
	    float stdX, stdY, stdZ;

	    arm_std_f32(runData.accHist[0], RUN_SENSOR_HIST, &stdX);//计算X轴上加速度的标准偏差?
	    arm_std_f32(runData.accHist[1], RUN_SENSOR_HIST, &stdY);//计算Y轴上加速度的标准偏差?
	    arm_std_f32(runData.accHist[2], RUN_SENSOR_HIST, &stdZ);//计算Z轴上加速度的标准偏差?

	    if ((stdX + stdY + stdZ) < (IMU_STATIC_STD*2)) {
		if (!((axis + 0) % 3))
		    navUkfZeroRate(IMU_RATEX, 0);
		else if (!((axis + 1) % 3))
		    navUkfZeroRate(IMU_RATEY, 1);
		else
		    navUkfZeroRate(IMU_RATEZ, 2);
		axis++;
	    }
	}

        navUkfFinish();
        altUkfProcess(AQ_PRESSURE);

        // determine which altitude estimate to use
        if (gpsData.hAcc > 0.8f) {
            runData.altPos = &ALT_POS;//由气压值计算出来的海拔
            runData.altVel = &ALT_VEL;
        }
        else {
            runData.altPos = &UKF_ALTITUDE;//可这个值还是气压计计算出来的值
            runData.altVel = &UKF_VELD;
        }


//	if (!(loops % (int)(1.0f / AQ_OUTER_TIMESTEP)))
//	    loggerDoHeader();//每200次执行下这个函数
//	loggerDo();//每次都执行
//	gimbalUpdate();

#ifdef CAN_CALIB
	canTxIMUData(loops);
#endif
       // calibrate();//地磁校准就在这里

	loops++;
    }
}

void runInit(void) {
    float acc[3], mag[3];
    float pres;
    int i;

    memset((void *)&runData, 0, sizeof(runData));
	xTaskCreate((TaskFunction_t )runTaskCode,		
					(const char*	)"run_task", 	
					(uint16_t		)RUN_TASK_SIZE, 
					(void*			)NULL,				
					(UBaseType_t	)RUN_PRIORITY,	
					(TaskHandle_t*	)&runTaskHandle);  

    acc[0] = IMU_ACCX;
    acc[1] = IMU_ACCY;
    acc[2] = IMU_ACCZ;

    mag[0] = IMU_MAGX;
    mag[1] = IMU_MAGY;
    mag[2] = IMU_MAGZ;

    pres = AQ_PRESSURE;

    // initialize sensor history
    for (i = 0; i < RUN_SENSOR_HIST; i++) {
        runData.accHist[0][i] = acc[0];
        runData.accHist[1][i] = acc[1];
        runData.accHist[2][i] = acc[2];
        runData.magHist[0][i] = mag[0];
        runData.magHist[1][i] = mag[1];
        runData.magHist[2][i] = mag[2];
        runData.presHist[i] = pres;

        runData.sumAcc[0] += acc[0];
        runData.sumAcc[1] += acc[1];
        runData.sumAcc[2] += acc[2];
        runData.sumMag[0] += mag[0];
        runData.sumMag[1] += mag[1];
        runData.sumMag[2] += mag[2];
        runData.sumPres += pres;
    }

    runData.sensorHistIndex = 0;

    runData.bestHacc = 1000.0f;
    runData.accMask = 1000.0f;

    // use altUkf altitude & vertical velocity estimates to start with
    runData.altPos = &ALT_POS;
    runData.altVel = &ALT_VEL;

}
void navPressureAdjust(float altitude) {
    runData.presAltOffset = altitude - ALTITUDE;
}

