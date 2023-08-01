#ifndef SYSTEM_WANGJJ_WANGJIAJUN
#define SYSTEM_WANGJJ_WANGJIAJUN


// prompt:
// 1.我在使用STM32C8T6,我不希望使用HAL库,我希望使用裸机编程
// 2.我正在试图编写一个可复用的库
// 3.我希望根据使用频率抽象RCC,并编写RCC相关的处理函数,请你给出函数列表并给出解释
// 4.我希望函数数量精简
// 5.位操作使用stm32f10x.h中的位操作宏,例如:SET_BIT(REG, BIT)
void RCC_Init();
void RCC_SetMCO(uint32_t x);
void RCC_ClockEnable(volatile uint32_t* bus,uint32_t Peripheral, FunctionalState State);
// prompt:
// 1.我在使用STM32C8T6,我不希望使用HAL库,我希望使用裸机编程
// 2.我正在试图编写一个可复用的库
// 3.我希望根据使用频率抽象GPIO,并编写GPIO相关的处理函数,请你给出函数列表并给出解释
// 4.我希望函数数量精简
// 5.位操作使用stm32f10x.h中的位操作宏,例如:SET_BIT(REG, BIT)
void GPIO_Init(GPIO_TypeDef* GPIOx, uint16_t Pin, uint16_t Mode);
void GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t Pin, uint8_t Value);
uint8_t GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t Pin);
void GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t Pin);


#endif