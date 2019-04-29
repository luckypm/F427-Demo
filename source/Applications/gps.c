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

#include "gps.h"
#include "telemetry.h"
#include "serial.h"
#include "digital.h"
#include "comm.h"
#include "config.h"
#include "util.h"
#include "filer.h"
#include "d_imu.h"
#include "supervisor.h"
#include "ext_irq.h"
#include <string.h>

gpsStruct_t gpsData __attribute__((section(".ccm")));

TaskHandle_t gpsTaskHandle;


#ifdef GPS_LOG_BUF
char gpsLog[GPS_LOG_BUF];
#endif

void gpsSendSetup(void) {
    ubloxSendSetup();
}

void gpsCheckBaud(serialPort_t *s) {
    static int i = GPS_RETRIES;
    if (((IMU_LASTUPD - gpsData.lastMessage) > 10000000) && (--i)>0) {
	if (!gpsData.baudCycle[++gpsData.baudSlot])
	    gpsData.baudSlot = 0;
	AQ_NOTICE("GPS: trying new baud rate\n");
	serialChangeBaud(s, gpsData.baudCycle[gpsData.baudSlot]);
	ubloxInitGps();//如果上面的波特率与GPS的吻合，进入这个函数后再去设置GPS的波特率就会成功得到一个ACK值
	serialChangeBaud(s, GPS_BAUD_RATE);//再把串口的波特率改掉
	gpsSendSetup();
	gpsData.lastMessage = IMU_LASTUPD;
    }
}

void gpsTaskCode(void *p) {
    serialPort_t *s = gpsData.gpsPort;
    char c;
#ifdef GPS_LOG_BUF
    int logPointer = 0;
#endif
    unsigned int ret = 0;

    AQ_NOTICE("GPS task task started\n");
	//初始配置ublox
    ubloxInit();

    while (1) {
	yield(1);
	gpsCheckBaud(s);

	while (serialAvailable(s)) {
	    c = serialRead(s);
	    ret = ubloxCharIn(c);

	    // position update更新了位置
	    if (ret == 1) {
		// notify world of new data
		xEventGroupSetBits(EventGroupHandler,gpsPosFlag);
	    }
	    // velocity update更新了速度
	    else if (ret == 2) {
		// notify world of new data
		xEventGroupSetBits(EventGroupHandler,gpsVelFlag);
	    }
	    // lost sync
	    else if (ret == 3) {
		gpsCheckBaud(s);
	    }

#ifdef GPS_LOG_BUF
	    gpsLog[logPointer] = c;
	    logPointer = (logPointer + 1) % GPS_LOG_BUF;
#endif
	}

#ifdef GPS_LOG_BUF
	filerSetHead(gpsData.logHandle, logPointer);
#endif


    }
}

void gpsPassThrough(commRcvrStruct_t *r) {
    while (commAvailable(r))
	serialWrite(gpsData.gpsPort, commReadChar(r));
}

void gpsTpHandler() {
    unsigned long tp = timerMicros();
    unsigned long diff = (tp - gpsData.lastTimepulse);

    if (diff > 950000 && diff < 1050000)
	gpsData.microsPerSecond -= (gpsData.microsPerSecond - (signed long)((tp - gpsData.lastTimepulse)<<11))>>5;
    gpsData.lastTimepulse = tp;
    gpsData.TPtowMS = gpsData.lastReceivedTPtowMS;
}

void gpsInit(void) {
    AQ_NOTICE("GPS init\n");

    memset((void *)&gpsData, 0, sizeof(gpsData));

    gpsData.baudCycle[0] = GPS_BAUD_RATE;
    gpsData.baudCycle[1] = 9600;
    gpsData.baudCycle[2] = 38400;
	gpsData.baudCycle[2] = 115200;
    gpsData.baudSlot = 0;
	//RX用了DMA，TX没用
    gpsData.gpsPort = serialOpen(GPS_USART, GPS_BAUD_RATE, USART_HardwareFlowControl_None, 512, 512);

    xTaskCreate((TaskFunction_t )gpsTaskCode,     	
                (const char*    )"gps_task",   	
                (uint16_t       )GPS_STACK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )GPS_PRIORITY,	
                (TaskHandle_t*  )&gpsTaskHandle);  

#ifdef GPS_TP_PORT //未使用该引脚，GPS配置时也关闭了时钟脉冲，所以这里不会触发
    extRegisterCallback(GPS_TP_PORT, GPS_TP_PIN, EXTI_Trigger_Rising, 1, GPIO_PuPd_NOPULL, gpsTpHandler);
#endif

    gpsData.microsPerSecond = AQ_US_PER_SEC<<11;

    gpsData.hAcc = gpsData.vAcc = gpsData.sAcc = 100.0f;

#ifdef GPS_LOG_BUF
    gpsData.logHandle = filerGetHandle(GPS_FNAME);
    filerStream(gpsData.logHandle, gpsLog, GPS_LOG_BUF);
#endif
	//可以定义一个直通GPS的串口，由该串口向GPS发送数据，未使用
    commRegisterRcvrFunc(COMM_STREAM_TYPE_GPS, gpsPassThrough);
}

void gpsSendPacket(unsigned char len, char *buf) {
    unsigned int i;

    for (i = 0; i < len; i++)
	serialWrite(gpsData.gpsPort, buf[i]);
}
