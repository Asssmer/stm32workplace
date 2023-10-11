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
    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;
    SET_BIT(RCC->CFGR, RCC_CFGR_HPRE_DIV1);  // AHB prescaler: No division
    SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV2); // APB1 prescaler: Divide by 2
    SET_BIT(RCC->CFGR, RCC_CFGR_PPRE2_DIV1); // APB2 prescaler: No division
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
    TIMx->PSC = Prescaler; // 设置预分频值
    TIMx->ARR = Period;    // 设置自动重装载值
}

void Timer_Start(TIM_TypeDef *TIMx)
{
    uint16_t uieState = TIMx->DIER & TIM_DIER_UIE; // 保存当前定时器中断使能（UIE）状态
    CLEAR_BIT(TIMx->DIER, TIM_DIER_UIE);           // 清除UIE位，禁用更新中断
    SET_BIT(TIMx->EGR, TIM_EGR_UG);                // 设置UG位，强制更新，这将立即更新所有定时器设置
    SET_BIT(TIMx->DIER, uieState);                 // 恢复原来的UIE设置，即如果之前启用了更新中断，现在重新启用
    SET_BIT(TIMx->CR1, TIM_CR1_CEN);               // 设置CEN位，使能定时器计数
}

void Timer_Stop(TIM_TypeDef *TIMx)
{
    CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN); // 清除CEN位，停止定时器
    CLEAR_BIT(TIMx->SR, TIM_SR_UIF);
    SET_BIT(TIMx->EGR, TIM_EGR_UG); // 强制更新（使所有设置生效）
}

void delay_ms(uint16_t ms)
{
    Timer_Init(TIM2, 71, 999);    // 设置预分频值为7200-1,设置自动重装载值为10000
    SET_BIT(TIM2->CR1, TIM_CR1_URS); // 设置URS,这样UIF才不会被UG位设置改变
    uint16_t loops = ms;
    Timer_Start(TIM2);
    for (uint16_t i = 0; i < loops; i++)
    {
        while (!(TIM2->SR & TIM_SR_UIF))
            ;                    // 等待更新标志位
        TIM2->SR &= ~TIM_SR_UIF; // 清除更新标志位
    }
    Timer_Stop(TIM2);
}

void delay_s(uint16_t s)
{
    Timer_Init(TIM2, 7199, 9999);    // 设置预分频值为7200-1,得到10Mhz,周期0.1ms,设置自动重装载值为10000
    SET_BIT(TIM2->CR1, TIM_CR1_URS); // 设置URS,这样UIF才不会被UG位设置改变
    uint16_t loops = s;
    Timer_Start(TIM2);
    for (uint16_t i = 0; i < loops; i++)
    {
        while (!(TIM2->SR & TIM_SR_UIF))
            ;                    // 等待更新标志位
        TIM2->SR &= ~TIM_SR_UIF; // 清除更新标志位
    }
    Timer_Stop(TIM2);
}

void USART1_Init(void)
{
    NVIC_EnableIRQ(DMA1_Channel4_IRQn); // 启用DMA1通道4的中断

    GPIOA->CRH &= ~(0xFF << 4);    // 清除PA9,PA10控制位
    GPIOA->CRH |= 0b01001011 << 4; // 设置PA9为复用推挽输出,PA10为浮空输入

    USART1->BRR = 72000000 / 115200; // 设置波特率
    USART1->CR1 &= ~(1 << 12);       // 设置字长
    USART1->CR2 &= ~(0b11 << 12);    // 设置停止位
    USART1->CR1 |= 1 << 2;           // 使能接收RE
    USART1->CR1 |= 1 << 3;           // 使能发送TE
    USART1->CR1 |= 1 << 13;          // 使能USART1,UE
}

int log(uint8_t *string)
{
    uint8_t *start = string;
    uint16_t count_size = 0;
    while (string[count_size])
    {
        if (++count_size > 1024)
        {
            log("string is too long!");
            return -1;
        }
    }
    SET_BIT(USART1->CR3, USART_CR3_DMAT); // 使能USART,DMA,TX

    while (DMA1_Channel4->CCR & DMA_CCR4_EN)
        ; // 先确定关闭通道
    DMA1_Channel4->CMAR = (uint32_t)string;
    DMA1_Channel4->CNDTR = count_size;
    DMA1_Channel4->CPAR = (uint32_t)&USART1->DR; // 设置外设寄存器地址
    SET_BIT(DMA1_Channel4->CCR, DMA_CCR4_DIR | DMA_CCR4_MINC | DMA_CCR4_TCIE);

    while (!(USART1->SR & USART_SR_TXE))
        ;                    // 等待TXE
    DMA1_Channel4->CCR |= 1; // 开启通道
    while (!(USART1->SR & USART_SR_TC))
        ;
    return 0;
}

// 中断处理程序
void DMA1_Channel4_IRQHandler(void)
{
    if (READ_BIT(DMA1->ISR, DMA_ISR_TCIF4))
    {
        SET_BIT(DMA1->IFCR, DMA_IFCR_CTCIF4); // 清除传输完成中断标志
        CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN);
    }
}