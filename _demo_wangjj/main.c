#include "stm32f10x.h"
#include <stdint.h>
#include "./include_stm32f103_wangjj/SYSTEM_wangjj.h"

int main(void)
{
    RCC_Init();
    RCC_SetMCO(RCC_CFGR_MCO_SYSCLK);
    RCC_ClockEnable(&RCC->APB2ENR,RCC_APB2ENR_IOPAEN,ENABLE);
    RCC_ClockEnable(&RCC->APB2ENR,RCC_APB2ENR_IOPBEN,ENABLE);
    RCC_ClockEnable(&RCC->APB2ENR,RCC_APB2ENR_IOPCEN,ENABLE);

    GPIO_Init(GPIOC,13,0x03);
    GPIO_WritePin(GPIOC,13,1);
    while (1)
    {
        GPIO_TogglePin(GPIOC,13);
    }
}