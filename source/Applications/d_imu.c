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

#include "d_imu.h"
#include "comm.h"
#include <math.h>

dImuStruct_t dImuData __attribute__((section(".ccm")));
TaskHandle_t dIMUTaskHandle;
EventGroupHandle_t EventGroupHandler;	//事件标志组句柄

const uint16_t dImuCalibParameters[] = {
    IMU_ACC_BIAS_X,
    IMU_ACC_BIAS_Y,
    IMU_ACC_BIAS_Z,
    IMU_ACC_BIAS1_X,
    IMU_ACC_BIAS1_Y,
    IMU_ACC_BIAS1_Z,
    IMU_ACC_BIAS2_X,
    IMU_ACC_BIAS2_Y,
    IMU_ACC_BIAS2_Z,
    IMU_ACC_BIAS3_X,
    IMU_ACC_BIAS3_Y,
    IMU_ACC_BIAS3_Z,
    IMU_ACC_SCAL_X,
    IMU_ACC_SCAL_Y,
    IMU_ACC_SCAL_Z,
    IMU_ACC_SCAL1_X,
    IMU_ACC_SCAL1_Y,
    IMU_ACC_SCAL1_Z,
    IMU_ACC_SCAL2_X,
    IMU_ACC_SCAL2_Y,
    IMU_ACC_SCAL2_Z,
    IMU_ACC_SCAL3_X,
    IMU_ACC_SCAL3_Y,
    IMU_ACC_SCAL3_Z,
    IMU_ACC_ALGN_XY,
    IMU_ACC_ALGN_XZ,
    IMU_ACC_ALGN_YX,
    IMU_ACC_ALGN_YZ,
    IMU_ACC_ALGN_ZX,
    IMU_ACC_ALGN_ZY,
    IMU_GYO_BIAS_X,
    IMU_GYO_BIAS_Y,
    IMU_GYO_BIAS_Z,
    IMU_GYO_BIAS1_X,
    IMU_GYO_BIAS1_Y,
    IMU_GYO_BIAS1_Z,
    IMU_GYO_BIAS2_X,
    IMU_GYO_BIAS2_Y,
    IMU_GYO_BIAS2_Z,
    IMU_GYO_BIAS3_X,
    IMU_GYO_BIAS3_Y,
    IMU_GYO_BIAS3_Z,
    IMU_GYO_SCAL_X,
    IMU_GYO_SCAL_Y,
    IMU_GYO_SCAL_Z,
    IMU_GYO_ALGN_XY,
    IMU_GYO_ALGN_XZ,
    IMU_GYO_ALGN_YX,
    IMU_GYO_ALGN_YZ,
    IMU_GYO_ALGN_ZX,
    IMU_GYO_ALGN_ZY,
    IMU_MAG_BIAS_X,
    IMU_MAG_BIAS_Y,
    IMU_MAG_BIAS_Z,
    IMU_MAG_BIAS1_X,
    IMU_MAG_BIAS1_Y,
    IMU_MAG_BIAS1_Z,
    IMU_MAG_BIAS2_X,
    IMU_MAG_BIAS2_Y,
    IMU_MAG_BIAS2_Z,
    IMU_MAG_BIAS3_X,
    IMU_MAG_BIAS3_Y,
    IMU_MAG_BIAS3_Z,
    IMU_MAG_SCAL_X,
    IMU_MAG_SCAL_Y,
    IMU_MAG_SCAL_Z,
    IMU_MAG_SCAL1_X,
    IMU_MAG_SCAL1_Y,
    IMU_MAG_SCAL1_Z,
    IMU_MAG_SCAL2_X,
    IMU_MAG_SCAL2_Y,
    IMU_MAG_SCAL2_Z,
    IMU_MAG_SCAL3_X,
    IMU_MAG_SCAL3_Y,
    IMU_MAG_SCAL3_Z,
    IMU_MAG_ALGN_XY,
    IMU_MAG_ALGN_XZ,
    IMU_MAG_ALGN_YX,
    IMU_MAG_ALGN_YZ,
    IMU_MAG_ALGN_ZX,
    IMU_MAG_ALGN_ZY
};
static void dIMUCalcTempDiff(void) {
		float temp = 0.0f;
		int i = 0;
	
#ifdef DIMU_HAVE_MPU6000
		if (mpu6000Data.enabled) {
			temp += mpu6000Data.temp;
			i++;
		}
#endif
#ifdef DIMU_HAVE_MAX21100
		if (max21100Data.enabled) {
			temp += max21100Data.temp;
			i++;
		}
#endif
#ifdef DIMU_HAVE_MS5611
		if (ms5611Data.enabled) {
		  temp += ms5611Data.temp;
		  i++;
		}
#endif
		dImuData.temp = temp / (float)i;
	
		dImuData.dTemp = dImuData.temp - IMU_ROOM_TEMP;
		dImuData.dTemp2 = dImuData.dTemp * dImuData.dTemp;
		dImuData.dTemp3 = dImuData.dTemp2 * dImuData.dTemp;
	}


void dIMURequestCalibWrite(void) {
    if (!dImuData.calibReadWriteFlag)
        dImuData.calibReadWriteFlag = 2;
}

void dIMURequestCalibRead(void) {
    if (!dImuData.calibReadWriteFlag)
        dImuData.calibReadWriteFlag = 1;
}

void dIMUTare(void) {
    float acc[3], gyo[3];
    uint32_t lastUpdate;
    float samples = 0.5f / DIMU_OUTER_DT; // //samples=100
    int i;

    // reset all parameters
    for (i = 0; dImuCalibParameters[i] != IMU_MAG_BIAS_X; i++)
    p[dImuCalibParameters[i]] = 0.0f;//注意这里把所有的偏差等值都赋值为0

    p[IMU_ACC_SCAL_X] = 1.0f;//然后再把加速度计和陀螺仪的SCAL都赋值为1
    p[IMU_ACC_SCAL_Y] = 1.0f;           //当进行水平校准时，以Z轴加速度为例，程序运行到下面时一定会跳出，p[IMU_ACC_BIAS_Z]=0，其它相关偏差也为0，但
    p[IMU_ACC_SCAL_Z] = 1.0f;           //p[IMU_ACC_SCAL_Z]为1；接着进入mpu6000CalibAcc()函数看注释

    p[IMU_GYO_SCAL_X] = 1.0f;
    p[IMU_GYO_SCAL_Y] = 1.0f;
    p[IMU_GYO_SCAL_Z] = 1.0f;

    lastUpdate = dImuData.lastUpdate;

    // let new averages settle相当于等了0.5s
    for (i = 0; i < (int)samples; i++) {
        while (lastUpdate == dImuData.lastUpdate)
            ;
        lastUpdate = dImuData.lastUpdate;
    }

    for (i = 0; i < 3; i++) {
        acc[i] = 0.0f;
        gyo[i] = 0.0f;
    }

    for (i = 0; i < (int)samples; i++) {
        while (lastUpdate == dImuData.lastUpdate)//等待，等到dImuData.lastUpdate更新后才执行以下的程序，这就意味着运行到这里一段时间(过了这个任务的时间片后)一定会跳出去，因为dImuData.lastUpdate要更新
            ;
        lastUpdate = dImuData.lastUpdate;//传感器数据更新了才采样

        acc[0] += IMU_RAW_ACCX;//校准值到时候是加在raw值上的，所以这里要采样raw值
        acc[1] += IMU_RAW_ACCY;
        acc[2] += IMU_RAW_ACCZ;

        gyo[0] += IMU_RAW_RATEX;
        gyo[1] += IMU_RAW_RATEY;
        gyo[2] += IMU_RAW_RATEZ;
    }

    p[IMU_ACC_BIAS_X] = -(acc[0] / samples);
    p[IMU_ACC_BIAS_Y] = -(acc[1] / samples);
	//这里的方向与原AQ中不同
    p[IMU_ACC_BIAS_Z] = -(GRAVITY + (acc[2] / samples));//先求出100次Z轴的原始加速度值的平均值，再求出这个平均值与9.8的偏差

    p[IMU_GYO_BIAS_X] = -(gyo[0] / samples);
    p[IMU_GYO_BIAS_Y] = -(gyo[1] / samples);	
    p[IMU_GYO_BIAS_Z] = -(gyo[2] / samples);

    //navUkfResetBias();
    //navUkfResetVels();
}

static void dIMUReadCalib(void) {
#ifdef DIMU_HAVE_EEPROM
    uint8_t *buf;
    int size;
    int p1 = 0;

    buf = eepromOpenRead();

    if (buf == 0) {
        AQ_NOTICE("DIMU: cannot read EEPROM parameters!\n");
    }
    else {
        while ((size = eepromRead(DIMU_EEPROM_BLOCK_SIZE)) != 0)
            p1 = configParseParams((char *)buf, size, p1);
            //p1 = 0;

        AQ_NOTICE("DIMU: read calibration parameters from EEPROM\n");
    }
#endif
}

static void dIMUWriteCalib(void) {
#ifdef DIMU_HAVE_EEPROM
    char *lineBuf;
    uint8_t *buf;
    int n;
    int i, j, k;
	lineBuf = (char *)aqCalloc(128, sizeof(char));
    if (lineBuf == 0) {
        AQ_NOTICE("DIMU: Error writing to EEPROM, cannot allocate memory.\n");
        return;
    }

    buf = eepromOpenWrite();

    k = 0;
    for (i = 0; i < sizeof(dImuCalibParameters) / sizeof(uint16_t); i++) {
        n = configFormatParam(lineBuf, dImuCalibParameters[i]);

        for (j = 0; j < n; j++) {
            buf[k++] = lineBuf[j];
                if (k == DIMU_EEPROM_BLOCK_SIZE) {
                eepromWrite();
                k = 0;
            }
        }
    }
    if (k != 0)
        eepromWrite();
    if (lineBuf)
        aqFree(lineBuf, 128, sizeof(char));

    AQ_NOTICE("DIMU: wrote calibration parameters to EEPROM\n");

    eepromClose();
#endif
}

static void dIMUReadWriteCalib(void) {
#ifdef DIMU_HAVE_EEPROM

#ifdef DIMU_HAVE_MPU6000
    mpu6000Disable();
#endif
#ifdef DIMU_HAVE_MAX21100
    max21100Disable();
#endif
#ifdef DIMU_HAVE_HMC5983
    hmc5983Disable();
#endif
#ifdef DIMU_HAVE_MS5611
    ms5611Disable();//读写CALIB参数时将MS5611传感器禁用
#endif

    if (dImuData.calibReadWriteFlag == 1)
	dIMUReadCalib();
    else if (dImuData.calibReadWriteFlag == 2)
	dIMUWriteCalib();

    dImuData.calibReadWriteFlag = 0;

#ifdef DIMU_HAVE_MPU6000
    mpu6000Enable();
#endif
#ifdef DIMU_HAVE_MAX21100
    max21100Enable();
#endif
#ifdef DIMU_HAVE_HMC5983
    hmc5983Enable();
#endif
#ifdef DIMU_HAVE_MS5611
    ms5611Enable();
#endif

#endif  //DIMU_HAVE_EEPROM
}


void dIMUTaskCode(void *pvParameters) {
    uint32_t loops = 0;

    while (1) {
    // wait for work
       xEventGroupWaitBits((EventGroupHandle_t	)EventGroupHandler,		
	 					   (EventBits_t			)dIMU_EVENT_BIT,
	 					   (BaseType_t			)pdTRUE,				
	 					   (BaseType_t			)pdTRUE,
	 			           (TickType_t			)portMAX_DELAY);	
	  if (dImuData.calibReadWriteFlag)
            dIMUReadWriteCalib();
        // double rate gyo loop
#ifdef DIMU_HAVE_MPU6000
        mpu6000DrateDecode();
#endif
#ifdef DIMU_HAVE_MAX21100
        max21100DrateDecode();
#endif
		imuDImuDRateReady();
        // full sensor loop
        if (!(loops % (DIMU_OUTER_PERIOD/DIMU_INNER_PERIOD))) {
#ifdef DIMU_HAVE_MPU6000
            mpu6000Decode();
#endif
#ifdef DIMU_HAVE_MAX21100
            max21100Decode();
#endif
#ifdef DIMU_HAVE_HMC5983
			hmc5983Read();
            hmc5983Decode();
#endif
#ifdef DIMU_HAVE_MS5611//第5步
            ms5611Decode();
#endif
            dImuData.lastUpdate = timerMicros();
			imuDImuSensorReady();
			dIMUCalcTempDiff();
        }

        loops++;
		utilTaskPeriodTime(dImuData.timer);
    }
}

void dIMUInit(void) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

#ifdef DIMU_HAVE_MPU6000
    mpu6000PreInit();
#endif
#ifdef DIMU_HAVE_MAX21100
    max21100PreInit();
#endif
#ifdef DIMU_HAVE_EEPROM
    eepromPreInit();
#endif
#ifdef DIMU_HAVE_HMC5983
    hmc5983PreInit();
#endif
#ifdef DIMU_HAVE_MS5611
    ms5611PreInit();
#endif

#ifdef DIMU_HAVE_MPU6000
    mpu6000Init();
#endif
#ifdef DIMU_HAVE_MAX21100
    max21100Init();
#endif
#ifdef DIMU_HAVE_EEPROM
    eepromInit();
    dIMUReadCalib();
#endif
#ifdef DIMU_HAVE_HMC5983
    if (hmc5983Init() == 0)
      AQ_NOTICE("DIMU: MAG sensor init failed!\n");
#endif
#ifdef DIMU_HAVE_MS5611
   if (ms5611Init() == 0)//第2步
     AQ_NOTICE("DIMU: PRES sensor init failed!\n");
#endif
     xTaskCreate((TaskFunction_t )dIMUTaskCode,     	
                (const char*    )"dIMU_task",   	
                (uint16_t       )DIMU_STACK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )DIMU_TASK_PRIORITY,	
                (TaskHandle_t*  )&dIMUTaskHandle);  

    // setup digital IMU timer
    DIMU_EN;

    // Time base configuration for 1MHz (us)
    TIM_TimeBaseStructure.TIM_Period = 0xffff;
    TIM_TimeBaseStructure.TIM_Prescaler = (DIMU_CLOCK / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(DIMU_TIM, &TIM_TimeBaseStructure);

    // reset
    TIM_SetCounter(DIMU_TIM, 0);

    // Output Compare for alarms
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(DIMU_TIM, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(DIMU_TIM, TIM_OCPreload_Disable);

    TIM_OC2Init(DIMU_TIM, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(DIMU_TIM, TIM_OCPreload_Disable);

    // Enable the global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DIMU_IRQ_CH;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // reset
    TIM_SetCounter(DIMU_TIM, 0);

    dIMUCancelAlarm1();

    // go...
    TIM_Cmd(DIMU_TIM, ENABLE);

#ifdef DIMU_HAVE_MPU6000
    mpu6000Enable();
#endif
#ifdef DIMU_HAVE_MAX21100
    max21100Enable();
#endif
#ifdef DIMU_HAVE_HMC5983
    hmc5983Enable();
#endif
#ifdef DIMU_HAVE_MS5611//第3步
    ms5611Enable();
#endif

    // setup IMU timestep alarm
    dImuData.nextPeriod = DIMU_TIM->CCR2 + DIMU_INNER_PERIOD;
    DIMU_TIM->CCR2 = dImuData.nextPeriod;
    DIMU_TIM->DIER |= TIM_IT_CC2;

#ifdef DIMU_HAVE_MPU6000
    //mpu6600InitialBias();
#endif
#ifdef DIMU_HAVE_MAX21100
    max21100InitialBias();
#endif
#ifdef DIMU_HAVE_MS5611//第4步
    //ms5611InitialBias();
#endif
}

void dIMUCancelAlarm1(void) {
    DIMU_TIM->DIER &= (uint16_t)~TIM_IT_CC1;
    TIM_ClearITPendingBit(DIMU_TIM, TIM_IT_CC1);
}

void dIMUSetAlarm1(int32_t us, dIMUCallback_t *callback, int parameter) {
    // schedule it
    dImuData.alarm1Callback = callback;
    dImuData.alarm1Parameter = parameter;

    DIMU_TIM->SR = (uint16_t)~TIM_IT_CC1;
    DIMU_TIM->CCR1 = DIMU_TIM->CNT + us;
    DIMU_TIM->DIER |= TIM_IT_CC1;
}


void DIMU_ISR(void) {
	BaseType_t Result,xHigherPriorityTaskWoken;
    // CC2 is used for IMU period timing
    if (TIM_GetITStatus(DIMU_TIM, TIM_IT_CC2) != RESET) {
		DIMU_TIM->SR = (uint16_t)~TIM_IT_CC2;

		// set next alarm
		dImuData.nextPeriod += DIMU_INNER_PERIOD;
		DIMU_TIM->CCR2 = dImuData.nextPeriod;
		Result=xEventGroupSetBitsFromISR(EventGroupHandler,dIMU_EVENT_BIT,&xHigherPriorityTaskWoken);
		if(Result!=pdFAIL)
		{
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
    }
    else if (TIM_GetITStatus(DIMU_TIM, TIM_IT_CC1) != RESET) {
		DIMU_TIM->SR = (uint16_t)~TIM_IT_CC1;

		// Disable the Interrupt
		DIMU_TIM->DIER &= (uint16_t)~TIM_IT_CC1;

		dImuData.alarm1Callback(dImuData.alarm1Parameter);
    }
	

}

