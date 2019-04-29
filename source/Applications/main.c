#include "sys.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "d_imu.h"
#include "supervisor.h"
#include "filer.h"
#include "sdio.h"
#include "comm.h"
#include "telemetry.h"
#include "gps.h"
#include "shell.h"
#include "run.h"
#include "nav_ukf.h"
#include "alt_ukf.h"
#include "aq_mavlink.h"
#include "uwb.h"


//�������ȼ�
#define INIT_TASK_PRIO		1
//�����ջ��С	
#define INIT_STK_SIZE 		400//֮ǰ����Ϊ128ʱ������ʼ���������˺��ջ�������׵��³������hardfault
//������
TaskHandle_t initTask_Handler;
//������
void init_task(void *pvParameters);

UBaseType_t initTask_uxHighWaterMark;



int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
	rccConfiguration();
	//������ʼ����
    xTaskCreate((TaskFunction_t )init_task,            //������
                (const char*    )"init_task",          //��������
                (uint16_t       )INIT_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )INIT_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&initTask_Handler);   //������              
    vTaskStartScheduler();          //�����������
}
 
//��ʼ����������
void init_task(void *pvParameters)
{
    EventGroupHandler = xEventGroupCreate();
	TickInit();
	sdioLowLevelInit();
 	//filerInit();
	configInit();
	commInit();
	//telemetryInit();
    imuInit();
#ifdef USE_MAVLINK
    mavlinkInit();
#endif
	navUkfInit();
    altUkfInit();
	supervisorInit();
	runInit();
	info();
	gpsInit();
	//uwbInit();
	//shellInit();
#ifdef DEBUG_TASK_INFO
	initTask_uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
#endif
    vTaskDelete(initTask_Handler); //ɾ����ʼ����
}



