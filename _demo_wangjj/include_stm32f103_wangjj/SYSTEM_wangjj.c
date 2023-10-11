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
    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;
    SET_BIT(RCC->CFGR, RCC_CFGR_HPRE_DIV1);  // AHB prescaler: No division
    SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV2); // APB1 prescaler: Divide by 2
    SET_BIT(RCC->CFGR, RCC_CFGR_PPRE2_DIV1); // APB2 prescaler: No division
}

void RCC_SetMCO(uint32_t x)
{
    RCC->APB2ENR |= 1 << 2;                                       // ʹ��GPIOAʱ��GPIOAAAAAAAAAAAAAAAAAAAA
    *((volatile unsigned int *)(GPIOA_BASE + 0x04)) &= ~(0b1111); // ���PA8����λ
    *((volatile unsigned int *)(GPIOA_BASE + 0x04)) |= 0b1011;    // ����PA8Ϊ�����������
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
    // 0x00: ����ģʽ��ģ�⣩
    // 0x04: ��������
    // 0x08: ����ģʽ��������
    // 0x01: ���ģʽ����©
    // 0x02: ���ģʽ�����죬2MHz
    // 0x03: ���ģʽ�����죬50MHz

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
    TIMx->PSC = Prescaler; // ����Ԥ��Ƶֵ
    TIMx->ARR = Period;    // �����Զ���װ��ֵ
}

void Timer_Start(TIM_TypeDef *TIMx)
{
    uint16_t uieState = TIMx->DIER & TIM_DIER_UIE; // ���浱ǰ��ʱ���ж�ʹ�ܣ�UIE��״̬
    CLEAR_BIT(TIMx->DIER, TIM_DIER_UIE);           // ���UIEλ�����ø����ж�
    SET_BIT(TIMx->EGR, TIM_EGR_UG);                // ����UGλ��ǿ�Ƹ��£��⽫�����������ж�ʱ������
    SET_BIT(TIMx->DIER, uieState);                 // �ָ�ԭ����UIE���ã������֮ǰ�����˸����жϣ�������������
    SET_BIT(TIMx->CR1, TIM_CR1_CEN);               // ����CENλ��ʹ�ܶ�ʱ������
}

void Timer_Stop(TIM_TypeDef *TIMx)
{
    CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN); // ���CENλ��ֹͣ��ʱ��
    CLEAR_BIT(TIMx->SR, TIM_SR_UIF);
    SET_BIT(TIMx->EGR, TIM_EGR_UG); // ǿ�Ƹ��£�ʹ����������Ч��
}

void delay_ms(uint16_t ms)
{
    Timer_Init(TIM2, 71, 999);    // ����Ԥ��ƵֵΪ7200-1,�����Զ���װ��ֵΪ10000
    SET_BIT(TIM2->CR1, TIM_CR1_URS); // ����URS,����UIF�Ų��ᱻUGλ���øı�
    uint16_t loops = ms;
    Timer_Start(TIM2);
    for (uint16_t i = 0; i < loops; i++)
    {
        while (!(TIM2->SR & TIM_SR_UIF))
            ;                    // �ȴ����±�־λ
        TIM2->SR &= ~TIM_SR_UIF; // ������±�־λ
    }
    Timer_Stop(TIM2);
}

void delay_s(uint16_t s)
{
    Timer_Init(TIM2, 7199, 9999);    // ����Ԥ��ƵֵΪ7200-1,�õ�10Mhz,����0.1ms,�����Զ���װ��ֵΪ10000
    SET_BIT(TIM2->CR1, TIM_CR1_URS); // ����URS,����UIF�Ų��ᱻUGλ���øı�
    uint16_t loops = s;
    Timer_Start(TIM2);
    for (uint16_t i = 0; i < loops; i++)
    {
        while (!(TIM2->SR & TIM_SR_UIF))
            ;                    // �ȴ����±�־λ
        TIM2->SR &= ~TIM_SR_UIF; // ������±�־λ
    }
    Timer_Stop(TIM2);
}

void USART1_Init(void)
{
    NVIC_EnableIRQ(DMA1_Channel4_IRQn); // ����DMA1ͨ��4���ж�

    GPIOA->CRH &= ~(0xFF << 4);    // ���PA9,PA10����λ
    GPIOA->CRH |= 0b01001011 << 4; // ����PA9Ϊ�����������,PA10Ϊ��������

    USART1->BRR = 72000000 / 115200; // ���ò�����
    USART1->CR1 &= ~(1 << 12);       // �����ֳ�
    USART1->CR2 &= ~(0b11 << 12);    // ����ֹͣλ
    USART1->CR1 |= 1 << 2;           // ʹ�ܽ���RE
    USART1->CR1 |= 1 << 3;           // ʹ�ܷ���TE
    USART1->CR1 |= 1 << 13;          // ʹ��USART1,UE
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
    SET_BIT(USART1->CR3, USART_CR3_DMAT); // ʹ��USART,DMA,TX

    while (DMA1_Channel4->CCR & DMA_CCR4_EN)
        ; // ��ȷ���ر�ͨ��
    DMA1_Channel4->CMAR = (uint32_t)string;
    DMA1_Channel4->CNDTR = count_size;
    DMA1_Channel4->CPAR = (uint32_t)&USART1->DR; // ��������Ĵ�����ַ
    SET_BIT(DMA1_Channel4->CCR, DMA_CCR4_DIR | DMA_CCR4_MINC | DMA_CCR4_TCIE);

    while (!(USART1->SR & USART_SR_TXE))
        ;                    // �ȴ�TXE
    DMA1_Channel4->CCR |= 1; // ����ͨ��
    while (!(USART1->SR & USART_SR_TC))
        ;
    return 0;
}

// �жϴ������
void DMA1_Channel4_IRQHandler(void)
{
    if (READ_BIT(DMA1->ISR, DMA_ISR_TCIF4))
    {
        SET_BIT(DMA1->IFCR, DMA_IFCR_CTCIF4); // �����������жϱ�־
        CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR4_EN);
    }
}