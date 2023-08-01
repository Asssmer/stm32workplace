#include "stm32f10x.h"
#include <stdint.h>
#include "./SYSTEM_wangjj.h"

// prompt:
// ����������Ĵ��� : void RCC_SetMCOSource(uint32_t mco_source)
// 1.����ʹ��STM32C8T6, �Ҳ�ϣ��ʹ��HAL��, ��ϣ��ʹ��������
// 2.��������ͼ��дһ���ɸ��õĿ�
// 3.��ϣ����������, ��׳,ȥ���,ģ�黯,�ɾ�����
// 4.��Ҫʹ���µĺ궨��,��Ҫ�µ�ȫ�ֶ���,������������ȥ���
// 5.λ����ʹ��stm32f10x.h�е�λ������,����:SET_BIT(REG, BIT)
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
    RCC->APB2ENR |= 1 << 2;                                       // ʹ��GPIOAʱ��GPIOAAAAAAAAAAAAAAAAAAAA
    *((volatile unsigned int *)(GPIOA_BASE + 0x04)) &= ~(0b1111); // ���PA8����λ
    *((volatile unsigned int *)(GPIOA_BASE + 0x04)) |= 0b1011;    // ����PA8Ϊ�����������
    MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, x);
}
void RCC_ClockEnable(volatile uint32_t* bus,uint32_t Peripheral, FunctionalState State)
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

// prompt:
// ����������Ĵ��� : void GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t Pin)
// 1.����ʹ��STM32C8T6, �Ҳ�ϣ��ʹ��HAL��, ��ϣ��ʹ��������
// 2.��������ͼ��дһ���ɸ��õĿ�
// 3.��ϣ����������,��׳,ȥ���,ģ�黯,����
// 4.��Ҫʹ���µĺ궨��,��Ҫ�µ�ȫ�ֶ���,������������ȥ���
// 5.λ����ʹ��stm32f10x.h�е�λ������,����:SET_BIT(REG, BIT)
void GPIO_Init(GPIO_TypeDef *GPIOx, uint16_t Pin, uint16_t Mode)
{

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
    // 0x00: ����ģʽ��ģ�⣩
    // 0x04: ��������
    // 0x08: ����ģʽ��������

    // 0x01: ���ģʽ����©
    // 0x02: ���ģʽ�����죬2MHz
    // 0x03: ���ģʽ�����죬50MHz
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
