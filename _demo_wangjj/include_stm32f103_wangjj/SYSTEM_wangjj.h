#ifndef SYSTEM_WANGJJ_WANGJIAJUN
#define SYSTEM_WANGJJ_WANGJIAJUN


#define reg(x)  (*((volatile unsigned int *)(x)))
#define bit(x)  ((unsigned int)1<<x)
// prompt:
// 1.����ʹ��STM32C8T6,�Ҳ�ϣ��ʹ��HAL��,��ϣ��ʹ��������
// 2.��������ͼ��дһ���ɸ��õĿ�
// 3.��ϣ������ʹ��Ƶ�ʳ���RCC,����дRCC��صĴ�����,������������б���������
// 4.��ϣ��������������
void RCC_Init(uint32_t clock_source);
void RCC_SetBusPrescalers(uint32_t AHB_Prescaler, uint32_t APB1_Prescaler, uint32_t APB2_Prescaler);
void RCC_EnablePeripheralClock(uint32_t peripheral_bus, uint32_t peripheral_id);
void RCC_SetMCOSource(uint32_t mco_source);




#endif