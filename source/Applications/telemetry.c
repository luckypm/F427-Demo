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

    Copyright 漏 2011-2014  Bill Nesbitt
*/

#include "aq.h"
#include "telemetry.h"
#include "util.h"
#include "config.h"
#include "rcc.h"
#include "supervisor.h"
#include "comm.h"
#include "imu.h"
#include "run.h"
#include "nav_ukf.h"
#include "d_imu.h"
#include "alt_ukf.h"
#include <string.h>
#include <stdio.h>

telemetryStruct_t telemetryData __attribute__((section(".ccm")));
char ListBuf[250];//数组得足够大，否则会造成堆栈溢出


/**************************************************************************************/
/*         下面部分为与匿名地面站相连接时的发送协议代码，来源于匿名代码示例
*
*/
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

u8 data_to_send[50];	//发送数据缓存

static uint8_t * ANO_DT_Send_Status(uint8_t *ptr,float angle_rol, float angle_pit, float angle_yaw, float alt, u8 fly_model, u8 armed)
{
	uint8_t _cnt=0;
	int16_t _temp;
	int32_t _temp2;

	_temp2 = (int32_t)alt;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int16_t)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	for(u8 i=0;i<_cnt;i++)
		*ptr++ = data_to_send[i];
	return ptr;

}
static uint8_t *ANO_DT_Send_Senser(uint8_t *ptr,float a_x,float a_y,float a_z,float g_x,float g_y,float g_z,float m_x,float m_y,float m_z)
{
	u8 _cnt=0;
	int16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = (int16_t)(a_x*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(a_y*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(a_z*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int16_t)(g_x*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(g_y*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(g_z*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (int16_t)(m_x*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(m_y*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(m_z*100);	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	for(u8 i=0;i<_cnt;i++)
		*ptr++ = data_to_send[i];
	return ptr;
}


// 调用得到各任务的状态或时间信息函数，都会比较耗时间，在调试阶段可用，发布阶段最好不用
#ifdef DEBUG_TASK_INFO
#if defined DEBUG_GET_TASK_STATUS
static uint8_t *telemtrySendTaskList(uint8_t *ptr) {
	char head1[] ="Name\t\tState\tPrio\tStack\tNum\r\n";
	char head2[] ="**************************************************************\r\n";
	int i,j,k;
	//int len;
	vTaskList(ListBuf);
	//len = strlen(listTab);
	i = j = k = 0;
	while(head1[i] != 0){
		*ptr++ = head1[i];
		i++;
	}
	while(head2[j] != 0){
		*ptr++ = head2[j];
		j++;
	}
	while(ListBuf[k] != 0){
		*ptr++ = ListBuf[k];
		k++;
	}
    return ptr;
}
#elif defined DEBUG_GET_TASK_RUN_TIME
static uint8_t *telemtrySendTaskTimeList(uint8_t *ptr) {
	char head1[] ="Task\t\tAbs Time(*50us)\t% Time\r\n";
	char head2[] ="**************************************************************\r\n";
	int i,j,k;
	//int len;
	vTaskGetRunTimeStats(ListBuf);
	//len = strlen(listTab);
	i = j = k = 0;
	while(head1[i] != 0){
		*ptr++ = head1[i];
		i++;
	}
	while(head2[j] != 0){
		*ptr++ = head2[j];
		j++;
	}
	while(ListBuf[k] != 0){
		*ptr++ = ListBuf[k];
		k++;
	}
    return ptr;
}
#endif
#else
static void telemtryGetImuList( char *list) {
	sprintf(list,"Name\t\tX\tY\tZ\r\n");
	list += strlen(list);
	sprintf(list,"-------------------------------------------------\r\n");
	list += strlen(list);
	sprintf(list,"%s\t\t%.2f\t%.2f\t%.2f\t\r\n","GYO",IMU_RATEX,IMU_RATEY,IMU_RATEZ);
	list += strlen(list);
	sprintf(list,"%s\t\t%.2f\t%.2f\t%.2f\t\r\n","ACC",IMU_ACCX,IMU_ACCY,IMU_ACCZ);
	list += strlen(list);
	sprintf(list,"%s\t\t%.2f\t%.2f\t%.2f\t\r\n","MAG",IMU_MAGX,IMU_MAGY,IMU_MAGZ);
	list += strlen(list);
	sprintf(list,"%s\t\t%.2f\t\r\n","PRE_ALT",ALT_POS);
	list += strlen(list);
	sprintf(list,"%s\t%.2f\t\r\n","PRESSURE",AQ_PRESSURE);
}

static uint8_t *telemtrySendImuList(uint8_t *ptr) {
	telemtryGetImuList(ListBuf);
	int i = 0;
	while(ListBuf[i] != 0){
	*ptr++ = ListBuf[i];
	i++;
	}
	return ptr;
}
#endif
//static uint8_t *telemtrySendVariable(uint8_t *ptr, char *name,float f) {
//	char buf[64];
//	int8_t ret = 0;
//    int8_t j;
//	ret = sprintf(buf,"%s = %.2f   ",name,f);
//    for (j = 0; j < ret; j++)
//		*ptr++ = buf[j];
//    return ptr;
//}


void telemetryDo(void) {
    commTxBuf_t *txBuf;
    uint8_t *ptr;


    telemetryData.loops++;

    //if (!(telemetryData.loops % (unsigned int)p[TELEMETRY_RATE])) {
	if (!(telemetryData.loops % 20)) {
	if (telemetryData.telemetryEnable) {
		//telemetryDisable();
	    txBuf = commGetTxBuf(COMM_STREAM_TYPE_TELEMETRY, 512);

	    // fail as we cannot block
	    if (txBuf != 0) {

		ptr = &txBuf->buf;
#ifdef DEBUG_TASK_INFO
	#if defined DEBUG_GET_TASK_STATUS
		ptr = telemtrySendTaskList(ptr);
	#elif defined DEBUG_GET_TASK_RUN_TIME
		ptr = telemtrySendTaskTimeList(ptr);
	#endif
		*ptr++ = '\n';

#else 
	#ifdef USE_ANO_GS
		ptr = ANO_DT_Send_Status(ptr,AQ_ROLL,AQ_PITCH,AQ_YAW,ALTITUDE,0,0);
		ptr = ANO_DT_Send_Senser(ptr,IMU_ACCX,IMU_ACCY,IMU_ACCZ,IMU_RATEX,IMU_RATEY,IMU_RATEZ,IMU_MAGX,IMU_MAGY,IMU_MAGZ);
	#else

		ptr = telemtrySendImuList(ptr);
		*ptr++ = '\n';
	#endif
#endif

		commSendTxBuf(txBuf, ptr - &txBuf->buf );
	    }
	}
    }

}

void telemetrySendNotice(const char *s) {
    commTxBuf_t *txBuf;
    uint8_t *ptr;
    txBuf = commGetTxBuf(COMM_STREAM_TYPE_TELEMETRY, 64);//长度固定

    if (txBuf > 0) {
	ptr = &txBuf->buf;
	do {
	    *ptr++ = *s;
	} while (*(s++));

	//最后ptr还自增了一次，指向了字符串末尾的空字符，故减1
	commSendTxBuf(txBuf, ptr - &txBuf->buf-1);
    }

}

void telemetryEnable(void) {
    telemetryData.telemetryEnable = 1;
}

void telemetryDisable(void) {
    telemetryData.telemetryEnable = 0;
}

void telemetryInit(void) {
    memset((void *)&telemetryData, 0, sizeof(telemetryData));

    commRegisterTelemFunc(telemetryDo);
    commRegisterNoticeFunc(telemetrySendNotice);
	telemetryEnable();
}
