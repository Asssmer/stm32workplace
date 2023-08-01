#ifndef SYSTEM_WANGJJ_WANGJIAJUN
#define SYSTEM_WANGJJ_WANGJIAJUN


// prompt:
// 1.����ʹ��STM32C8T6,�Ҳ�ϣ��ʹ��HAL��,��ϣ��ʹ��������
// 2.��������ͼ��дһ���ɸ��õĿ�
// 3.��ϣ������ʹ��Ƶ�ʳ���RCC,����дRCC��صĴ�����,������������б���������
// 4.��ϣ��������������
// 5.λ����ʹ��stm32f10x.h�е�λ������,����:SET_BIT(REG, BIT)
void RCC_Init();
void RCC_SetMCO(uint32_t x);
void RCC_ClockEnable(volatile uint32_t* bus,uint32_t Peripheral, FunctionalState State);
// prompt:
// 1.����ʹ��STM32C8T6,�Ҳ�ϣ��ʹ��HAL��,��ϣ��ʹ��������
// 2.��������ͼ��дһ���ɸ��õĿ�
// 3.��ϣ������ʹ��Ƶ�ʳ���GPIO,����дGPIO��صĴ�����,������������б���������
// 4.��ϣ��������������
// 5.λ����ʹ��stm32f10x.h�е�λ������,����:SET_BIT(REG, BIT)
void GPIO_Init(GPIO_TypeDef* GPIOx, uint16_t Pin, uint16_t Mode);
void GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t Pin, uint8_t Value);
uint8_t GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t Pin);
void GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t Pin);


#endif