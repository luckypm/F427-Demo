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


//任务优先级
#define INIT_TASK_PRIO		1
//任务堆栈大小	
#define INIT_STK_SIZE 		400//之前这里为128时，当初始化函数多了后堆栈不够容易导致程序进入hardfault
//任务句柄
TaskHandle_t initTask_Handler;
//任务函数
void init_task(void *pvParameters);

UBaseType_t initTask_uxHighWaterMark;



int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	rccConfiguration();
	//创建开始任务
    xTaskCreate((TaskFunction_t )init_task,            //任务函数
                (const char*    )"init_task",          //任务名称
                (uint16_t       )INIT_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )INIT_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&initTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}
 
//开始任务任务函数
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
    vTaskDelete(initTask_Handler); //删除开始任务
}



