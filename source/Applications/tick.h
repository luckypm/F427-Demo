#ifndef __TICK_H
#define __TICK_H

#include "stm32f4xx.h"
#include "rcc.h"

#define timerMicros()	TIM5->CNT
#define TICK_CLOCK		(rccClocks.PCLK1_Frequency * 2)

void TickInit(void);
void delay_micros(unsigned long t);
void delay(unsigned long t);
void ConfigureTimeForRunTimeStats(void);
extern volatile unsigned long long FreeRTOSRunTimeTicks;


#endif /* __TICK_H */

/* end of file */

