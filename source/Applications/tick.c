#include "tick.h"

volatile unsigned long long FreeRTOSRunTimeTicks;

/* 采用直接读取TIM5定时器值，作为整个系统的运行时钟 */

void TickInit(void)
{
	/* TIM5时钟为90MHz，需要配置成1us的计数周期 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 0xffffffff; //t is the time between each Timer irq.
	TIM_TimeBaseInitStructure.TIM_Prescaler = (TICK_CLOCK/1000000 - 1);
	
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

	TIM_Cmd(TIM5, ENABLE);
}

void ConfigureTimeForRunTimeStats(void)
{
	/* TIM7配置成1us的计数周期 */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 50-1; //t is the time between each Timer irq.
	TIM_TimeBaseInitStructure.TIM_Prescaler = (TICK_CLOCK/1000000 - 1);; //t = (1+TIM_Prescaler/SystemCoreClock)*(1+TIM_Period)
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStructure);

	TIM_Cmd(TIM7, ENABLE);
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); 

	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) 
	{
		FreeRTOSRunTimeTicks++;//每50us加1
	}
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  
}


void delay_micros(unsigned long t) {
    t = t + timerMicros();

    while (timerMicros() < t)
        ;
}

// delay for given milli seconds
void delay(unsigned long t) {
    delay_micros(t * 1000);
}


/* end of file */

