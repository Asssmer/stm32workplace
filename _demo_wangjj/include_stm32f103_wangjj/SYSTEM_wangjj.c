#include "stm32f10x.h"
#include "./SYSTEM_wangjj.h"
// prompt:
// ����������Ĵ��� : void RCC_Init(uint32_t clock_source)
// 1.����ʹ��STM32C8T6, �Ҳ�ϣ��ʹ��HAL��, ��ϣ��ʹ��������
// 2.��������ͼ��дһ���ɸ��õĿ�
// 3.��ϣ��������������, ����, ��׳
// 4.��Ҫʹ�ú궨��, ��ϣ���õ���ֻ��һ����������
void RCC_Init(uint32_t clock_source)
{
    // HSI
    if (clock_source == 0x00000000)
    {
        RCC->CR |= 0x00000001; // Turn on HSI
        while (!(RCC->CR & 0x00000002))
            ; // Wait for HSI ready
    }

    // HSE
    if (clock_source == 0x00000001)
    {
        RCC->CR |= 0x00010000; // Turn on HSE
        while (!(RCC->CR & 0x00020000))
            ; // Wait for HSE ready
    }

    // Set PLL source to HSE
    RCC->CFGR &= ~0x00010000;
    RCC->CFGR |= 0x00010000;

    // Set PLL multiplication to 9 (hardcoded to 9 as an example)
    RCC->CFGR &= ~0x003C0000;
    RCC->CFGR |= 0x001C0000; // this corresponds to PLLMULL9

    // Enable PLL
    RCC->CR |= 0x01000000;
    // Wait till PLL is ready
    while ((RCC->CR & 0x02000000) == 0)
        ;

    // Select PLL as system clock source
    RCC->CFGR &= ~0x00000003;
    RCC->CFGR |= 0x00000002;

    // Wait till PLL is used as system clock source
    while ((RCC->CFGR & 0x0000000C) != 0x00000008)
        ;
}

void RCC_SetBusPrescalers(uint32_t AHB_Prescaler, uint32_t APB1_Prescaler, uint32_t APB2_Prescaler)
{
    // First, clear the related bits
    RCC->CFGR &= ~(0xF8); // Clear AHB, APB1, and APB2 prescaler bits

    // Now set the prescalers
    RCC->CFGR |= (AHB_Prescaler << 4);   // AHB prescaler is in bits 7:4
    RCC->CFGR |= (APB1_Prescaler << 8);  // APB1 prescaler is in bits 10:8
    RCC->CFGR |= (APB2_Prescaler << 11); // APB2 prescaler is in bits 13:11
}

void RCC_EnablePeripheralClock(uint32_t peripheral_bus, uint32_t peripheral_id)
{
    // peripheral_bus��ֵӦΪ1��2��3���ֱ����AHB��APB1��APB2���ߡ�
    // peripheral_id����Ŀ��������ص�λ����
    if (peripheral_bus == 1)
    { // AHB peripheral
        RCC->AHBENR |= peripheral_id;
    }
    else if (peripheral_bus == 2)
    { // APB1 peripheral
        RCC->APB1ENR |= peripheral_id;
    }
    else if (peripheral_bus == 3)
    { // APB2 peripheral
        RCC->APB2ENR |= peripheral_id;
    }
}

void RCC_SetMCOSource(uint32_t mco_source)
{
    // 0x4��ϵͳʱ��(SYSCLK)���
    // 0x5���ڲ�8MHz��RC����ʱ�����
    // 0x6���ⲿ3-25MHz����ʱ�����
    // 0x7��PLLʱ��2��Ƶ�����
    // 0x8��PLL2ʱ�����
    // 0x9��PLL3ʱ��2��Ƶ�����
    // Enable clock for GPIOA
    RCC->APB2ENR |= 0x00000004;

    // Configure PA8 as alternate function output push-pull
    GPIOA->CRH &= ~0x0000000F;
    GPIOA->CRH |= 0x0000000B;

    // Clear MCO bits
    RCC->CFGR &= ~0xF0000000;

    // Set MCO source
    RCC->CFGR |= (mco_source << 24);
}

