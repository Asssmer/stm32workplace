#ifndef SYSTEM_WANGJJ_WANGJIAJUN
#define SYSTEM_WANGJJ_WANGJIAJUN


#define reg(x)  (*((volatile unsigned int *)(x)))
#define bit(x)  ((unsigned int)1<<x)
// prompt:
// 1.我在使用STM32C8T6,我不希望使用HAL库,我希望使用裸机编程
// 2.我正在试图编写一个可复用的库
// 3.我希望根据使用频率抽象RCC,并编写RCC相关的处理函数,请你给出函数列表并给出解释
// 4.我希望函数数量精简
void RCC_Init(uint32_t clock_source);
void RCC_SetBusPrescalers(uint32_t AHB_Prescaler, uint32_t APB1_Prescaler, uint32_t APB2_Prescaler);
void RCC_EnablePeripheralClock(uint32_t peripheral_bus, uint32_t peripheral_id);
void RCC_SetMCOSource(uint32_t mco_source);




#endif