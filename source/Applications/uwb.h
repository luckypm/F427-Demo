#ifndef _uwb_h
#define _uwb_h

#include "arm_math.h"


#define UWB_STACK_SIZE	    200
#define UWB_PRIORITY	    10


typedef struct {
	arm_matrix_instance_f32 A;      /* Matrix A Instance */
    arm_matrix_instance_f32 AT;     /* Matrix AT(A transpose) instance */
    arm_matrix_instance_f32 ATMA;   /* Matrix ATMA( AT multiply with A) instance */
    arm_matrix_instance_f32 ATMAI;  /* Matrix ATMAI(Inverse of ATMA) instance */
    arm_matrix_instance_f32 B;      /* Matrix B instance */
    arm_matrix_instance_f32 X;      /* Matrix X(Unknown Matrix) instance */
	uint8_t flag;
	uint32_t timer[2];
	uint32_t deltaTime;
} uwbStruct_t;

extern uwbStruct_t uwbData;

extern void uwbInit(void);

#endif
