#include "stm32f10x.h"
#include <stdint.h>
#include "./include_stm32f103_wangjj/SYSTEM_wangjj.h"

int main(void)
{
    RCC_Init();
    RCC_SetMCO(RCC_CFGR_MCO_SYSCLK);

    RCC_ClockEnable(&RCC->APB2ENR, RCC_APB2ENR_IOPAEN, ENABLE);
    RCC_ClockEnable(&RCC->APB2ENR, RCC_APB2ENR_IOPBEN, ENABLE);
    RCC_ClockEnable(&RCC->APB2ENR, RCC_APB2ENR_IOPCEN, ENABLE);
    RCC_ClockEnable(&RCC->APB2ENR, RCC_APB2ENR_AFIOEN, ENABLE);
    RCC_ClockEnable(&RCC->APB2ENR, RCC_APB2ENR_USART1EN, ENABLE);

    RCC_ClockEnable(&RCC->AHBENR, RCC_AHBENR_DMA1EN, ENABLE);
    RCC_ClockEnable(&RCC->APB1ENR, RCC_APB1ENR_TIM2EN, ENABLE);
    RCC_ClockEnable(&RCC->APB1ENR, RCC_APB1ENR_TIM3EN, ENABLE);

    GPIO_Init(GPIOC, 13, 0x03);
    GPIO_Init(GPIOA, 6, 0b1011);
    USART1_Init();

    Timer_Init(TIM3, 7199, 9999); // 设置预分频值为7200-1,设置自动重装载值为10000
    SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE);
    SET_BIT(TIM3->CCER, TIM_CCER_CC1E);
    SET_BIT(TIM3->CCR1, 7000);
    SET_BIT(TIM3->CR1, TIM_CR1_CEN);

    while (1)
    {
        GPIO_TogglePin(GPIOC, 13);
        delay_ms(1000);
        log("hahahahha!!!!!");
    }
}
