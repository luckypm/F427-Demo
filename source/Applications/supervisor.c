

#include "aq.h"
#include "config.h"
#include "comm.h"
#include "supervisor.h"
#include "digital.h"
#include "util.h"
#include "d_imu.h"
#include "gps.h"
#include <stdlib.h>
#include <string.h>

supervisorStruct_t supervisorData __attribute__((section(".ccm")));

TaskHandle_t supervisoTaskHandle;

void supervisorLEDsOn(void) {
    digitalLo(supervisorData.readyLed);
}

void supervisorLEDsOff(void) {
    digitalHi(supervisorData.readyLed);
}
void supervisorLEDsTogg(void) {
    digitalTogg(supervisorData.readyLed);
}


void supervisorTare(void) {
    supervisorLEDsOn();
    dIMUTare();
    AQ_NOTICE("Level calibration complete.\n");
    supervisorLEDsOff();
}

void supervisorTaskCode(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for ( ;; )
	{
		//用来显示GPS与指南针通迅是否正常；换不同的GPS与指南针时需要重启
		if(supervisorData.loops > 30 && (IMU_LASTUPD - gpsData.lastMessage) < 1000000 && (IMU_LASTUPD - hmc5983Data.lastUpdate) < 1000000){
		//if(supervisorData.cnt > 30 && (IMU_LASTUPD - gpsData.lastMessage) < 1000000){
			supervisorLEDsTogg();
		}
		else if(!(supervisorData.loops % 10)){
			supervisorLEDsTogg();
		}
		if(supervisorData.tareSwitch)
		{
			supervisorData.tareSwitch = 0;
			supervisorTare();
#ifdef DIMU_HAVE_EEPROM
                dIMURequestCalibWrite();
#endif
			
		}
		supervisorData.loops++;
		utilTaskPeriodTime(supervisorData.timer);
		/* 执行周期1Hz */
		vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_RATE_MS));
	}
}

void supervisorInit(void) {
    memset((void *)&supervisorData, 0, sizeof(supervisorData));
    supervisorData.readyLed = digitalInit(SUPERVISOR_READY_PORT, SUPERVISOR_READY_PIN, 1);
	xTaskCreate((TaskFunction_t )supervisorTaskCode,     	
                (const char*    )"supervisor_task",   	
                (uint16_t       )SUPERVISOR_STACK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )SUPERVISOR_PRIORITY,	
                (TaskHandle_t*  )&supervisoTaskHandle);  

}
