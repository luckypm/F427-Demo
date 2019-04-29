

#include "aq.h"

RCC_ClocksTypeDef rccClocks;

void rccConfiguration(void) {
   

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_DMA2, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // enable timer clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM7, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 | RCC_APB2Periph_TIM9 | RCC_APB2Periph_TIM10 | RCC_APB2Periph_TIM11, ENABLE);

    SYSCFG_CompensationCellCmd(ENABLE);

    // Clear reset flags
    RCC_ClearFlag();

    RCC_GetClocksFreq(&rccClocks);

}
