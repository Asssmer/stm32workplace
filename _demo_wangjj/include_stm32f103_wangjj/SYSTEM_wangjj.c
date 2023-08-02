#include "stm32f10x.h"
#include <stdint.h>
#include "./SYSTEM_wangjj.h"

// prompt:
// 请给出完整的代码 : void RCC_SetMCOSource(uint32_t mco_source)
// 1.我在使用STM32C8T6, 我不希望使用HAL库, 我希望使用裸机编程
// 2.我正在试图编写一个可复用的库
// 3.我希望函数完整, 健壮,去耦合,模块化,干净利落
// 4.不要使用新的宏定义,不要新的全局定义,这样函数可以去耦合
// 5.位操作使用stm32f10x.h中的位操作宏,例如:SET_BIT(REG, BIT)
void RCC_Init()
{
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY))
        ;
    SET_BIT(RCC->CR, RCC_CR_HSEON);
    while (!READ_BIT(RCC->CR, RCC_CR_HSERDY))
        ;
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLXTPRE);
    SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMULL, RCC_CFGR_PLLMULL9);
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY))
        ;
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    //????????????????????????????????????????????????????????????
    // while (READ_BIT(RCC->CR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    // ;
}

void RCC_SetMCO(uint32_t x)
{
    RCC->APB2ENR |= 1 << 2;                                       // 使能GPIOA时钟GPIOAAAAAAAAAAAAAAAAAAAA
    *((volatile unsigned int *)(GPIOA_BASE + 0x04)) &= ~(0b1111); // 清除PA8控制位
    *((volatile unsigned int *)(GPIOA_BASE + 0x04)) |= 0b1011;    // 设置PA8为复用推挽输出
    MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, x);
}

void RCC_ClockEnable(volatile uint32_t *bus, uint32_t Peripheral, FunctionalState State)
{
    if (State != DISABLE)
    {
        SET_BIT(*bus, (uint32_t)Peripheral);
    }
    else
    {
        CLEAR_BIT(*bus, (uint32_t)Peripheral);
    }
}

void GPIO_Init(GPIO_TypeDef *GPIOx, uint16_t Pin, uint16_t Mode)
{
    // 0x00: 输入模式（模拟）
    // 0x04: 浮空输入
    // 0x08: 输入模式（下拉）

    // 0x01: 输出模式，开漏
    // 0x02: 输出模式，推挽，2MHz
    // 0x03: 输出模式，推挽，50MHz

    if (Pin < 8)
    {
        CLEAR_BIT(GPIOx->CRL, (0x0F) << (Pin * 4));
        SET_BIT(GPIOx->CRL, Mode << (Pin * 4));
    }
    else
    {
        CLEAR_BIT(GPIOx->CRH, (0x0F) << ((Pin - 8) * 4));
        SET_BIT(GPIOx->CRH, Mode << ((Pin - 8) * 4));
    }
}

void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t Pin, uint8_t Value)
{
    if (Value)
        GPIOx->BSRR |= (uint16_t)1 << Pin;
    else
        GPIOx->BRR |= (uint16_t)1 << Pin;
}

uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t Pin)
{
    return (GPIOx->IDR >> Pin) & 0x01;
}

void GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t Pin)
{
    uint16_t pinMask = 1 << Pin;
    GPIOx->ODR ^= pinMask;
}

void Timer_Init(TIM_TypeDef *TIMx, uint16_t Prescaler, uint16_t Period)
{
    SET_BIT(TIMx->PSC, Prescaler);
    SET_BIT(TIMx->ARR, Period);
}

void Timer_Start(TIM_TypeDef *TIMx)
{
    uint16_t uieState = TIMx->DIER & TIM_DIER_UIE;
    CLEAR_BIT(TIMx->DIER, TIM_DIER_UIE);
    TIMx->CNT = 0;
    SET_BIT(TIMx->EGR, TIM_EGR_UG);
    SET_BIT(TIMx->DIER, uieState);
    SET_BIT(TIMx->CR1, TIM_CR1_CEN);
}

void Timer_Stop(TIM_TypeDef *TIMx)
{
    CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN);
}

void Timer_SetCallback(TIM_TypeDef *TIMx, void (*Callback)(void))
{
    timer_callback = Callback;
    // 配置中断
    SET_BIT(TIMx->DIER, TIM_DIER_UIE);
}

void Timer_delay_ms(uint16_t milliseconds){
    

}
// 中断处理程序
