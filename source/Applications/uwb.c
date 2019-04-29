
#include "uwb.h"
#include "aq.h"
#include "config.h"
#include "comm.h"
#include "supervisor.h"
#include "util.h"
#include "d_imu.h"
#include <stdlib.h>
#include <string.h>

uwbStruct_t uwbData __attribute__((section(".ccm")));

TaskHandle_t uwbTaskHandle;
arm_status status;

//float32_t A[9] = {1,2,3,2,4,5,3,5,6};//错,顺序主子式为0，也不能求解
//float32_t A[9] = {1,2,3,2,2,1,3,4,3};//对,注意不能加const，加了也会计算错误
//float32_t A[9] = {5,0,0,0,3,1,0,2,1};//对
//float32_t A[9] = { 0, -2,  1,
//				   3,  0, -2,
//				   -2, 3,  0};//错
//float32_t A[9] = {2,1,-3,1,2,-2,-1,3,2};//对
//float32_t AI[9];
float p1[3] = {30,0,0};
float p2[3] = {0,0,0};
float p3[3] = {0,30,0};
float p4[3] = {0,0,30};
//float R[4] = {30,42.426,30,51.962};
//float R[4] = {42.426,51.962,42.426,42.426};
float R[4] = {30,42.426,51.962,30};




float32_t B_f32[3];
float32_t A_f32[9];
/* ----------------------------------------------------------------------
* Temporary buffers  for storing intermediate values
* ------------------------------------------------------------------- */
/* Transpose of A Buffer */
float32_t AT_f32[9];
/* (Transpose of A * A) Buffer */
float32_t ATMA_f32[9];
/* Inverse(Transpose of A * A)	Buffer */
float32_t ATMAI_f32[9];
/* Test Output Buffer */
float32_t X_f32[3];

static void setArrayA_f32(void){
	A_f32[0] = 2*(p1[0] - p2[0]);
	A_f32[1] = 2*(p1[1] - p2[1]);
	A_f32[2] = 2*(p1[2] - p2[2]);
	A_f32[3] = 2*(p1[0] - p3[0]);
	A_f32[4] = 2*(p1[1] - p3[1]);
	A_f32[5] = 2*(p1[2] - p3[2]);
	A_f32[6] = 2*(p1[0] - p4[0]);
	A_f32[7] = 2*(p1[1] - p4[1]);
	A_f32[8] = 2*(p1[2] - p4[2]);
}
static void setArrayB_f32(void){
    float d1,d2,d3,d4;
	d1 = p1[0]*p1[0] + p1[1]*p1[1] + p1[2]*p1[2];
	d2 = p2[0]*p2[0] + p2[1]*p2[1] + p2[2]*p2[2];
	d3 = p3[0]*p3[0] + p3[1]*p3[1] + p3[2]*p3[2];
	d4 = p4[0]*p4[0] + p4[1]*p4[1] + p4[2]*p4[2];
	B_f32[0] = R[1]*R[1] - R[0]*R[0] + d1 - d2;
	B_f32[1] = R[2]*R[2] - R[0]*R[0] + d1 - d3;
	B_f32[2] = R[3]*R[3] - R[0]*R[0] + d1 - d4;
}
static void uwbMatrixInit(void){
  uint16_t srcRows, srcColumns;
  srcRows = 3;
  srcColumns = 3;
  arm_mat_init_f32(&uwbData.A, srcRows, srcColumns, (float32_t *)A_f32);
  arm_mat_init_f32(&uwbData.AT, srcRows, srcColumns, AT_f32);
  arm_mat_init_f32(&uwbData.ATMA, srcRows, srcColumns, ATMA_f32);
  arm_mat_init_f32(&uwbData.ATMAI, srcRows, srcColumns, ATMAI_f32);
  srcRows = 3;
  srcColumns = 1;
  arm_mat_init_f32(&uwbData.B, srcRows, srcColumns, (float32_t *)B_f32);
  arm_mat_init_f32(&uwbData.X, srcRows, srcColumns, X_f32);
}
static void uwbPositionSolution(void) {
  arm_status status;
  uint32_t startTime = timerMicros();
  
  setArrayB_f32();
  /* calculation of A transpose */
  status = arm_mat_trans_f32(&uwbData.A, &uwbData.AT);
  /* calculation of AT Multiply with A */
  status = arm_mat_mult_f32(&uwbData.AT, &uwbData.A, &uwbData.ATMA);
  /* calculation of Inverse((Transpose(A) * A) */
  status = arm_mat_inverse_f32(&uwbData.ATMA, &uwbData.ATMAI);

  /* calculation of (Inverse((Transpose(A) * A)) *  Transpose(A)) */
  status = arm_mat_mult_f32(&uwbData.ATMAI, &uwbData.AT, &uwbData.ATMA);
  /* calculation ((Inverse((Transpose(A) * A)) *  Transpose(A)) * B) */
  status = arm_mat_mult_f32(&uwbData.ATMA, &uwbData.B, &uwbData.X);
  uwbData.deltaTime = timerMicros() - startTime;

}

void uwbTaskCode(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for ( ;; )
	{	
		//if(!uwbData.flag){
			//uwbData.flag = 1;
		uwbPositionSolution();
		//}
		utilTaskPeriodTime(uwbData.timer);
		/* 执行周期10Hz */
		vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_RATE_MS));
	}
}

void uwbInit(void) {
    memset((void *)&uwbData, 0, sizeof(uwbData));
	setArrayA_f32();
	uwbMatrixInit();
	xTaskCreate((TaskFunction_t )uwbTaskCode,     	
                (const char*    )"uwb_task",   	
                (uint16_t       )UWB_STACK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )UWB_PRIORITY,	
                (TaskHandle_t*  )&uwbTaskHandle);  

}

