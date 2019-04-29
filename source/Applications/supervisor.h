
#ifndef _supervisor_h
#define _supervisor_h

#include "digital.h"

#define SUPERVISOR_STACK_SIZE	    200
#define SUPERVISOR_PRIORITY	    	10
enum supervisorStates {
    STATE_INITIALIZING	= 0x00,//二进制为         0000 0000
    STATE_CALIBRATION	= 0x01,//二进制为         0000 0001
    STATE_DISARMED	= 0x02,//二进制为             0000 0010
    STATE_ARMED		= 0x04,//二进制为             0000 0100
    STATE_FLYING	= 0x08,//二进制为             0000 1000
    STATE_RADIO_LOSS1	= 0x10,//二进制为        0001 0000
    STATE_RADIO_LOSS2	= 0x20,//二进制为        0010 0000
    STATE_LOW_BATTERY1	= 0x40,//二进制为        0100 0000
    STATE_LOW_BATTERY2	= 0x80//二进制为         1000 0000
};


typedef struct {
    digitalPin *readyLed;
	uint8_t tareSwitch;
	uint8_t state;
	uint32_t timer[2];
	uint32_t loops;
} supervisorStruct_t;

extern supervisorStruct_t supervisorData;

extern void supervisorInit(void);
extern void supervisorTare(void);

#endif
