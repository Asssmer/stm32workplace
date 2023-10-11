#define STM32F10X_MD
#include "stm32f10x.h"
// #include "stdlib.h"
// #include "stdio.h"

#define RCC_BASE 0x40021000

#define RCC_CR *((volatile unsigned int *)(RCC_BASE + 0x00))
#define RCC_CFGR *((volatile unsigned int *)(RCC_BASE + 0x04))
#define RCC_CIR *((volatile unsigned int *)(RCC_BASE + 0x08))
#define RCC_APB2RSTR *((volatile unsigned int *)(RCC_BASE + 0x0C))
#define RCC_APB1RSTR *((volatile unsigned int *)(RCC_BASE + 0x10))
#define RCC_AHBENR *((volatile unsigned int *)(RCC_BASE + 0x14))
#define RCC_APB2ENR *((volatile unsigned int *)(RCC_BASE + 0x18))
#define RCC_APB1ENR *((volatile unsigned int *)(RCC_BASE + 0x1C))
#define RCC_BDCR *((volatile unsigned int *)(RCC_BASE + 0x20))
#define RCC_CSR *((volatile unsigned int *)(RCC_BASE + 0x24))

#define GPIOC_BASE 0x40011000

#define GPIOC_CRL *((volatile unsigned int *)(GPIOC_BASE + 0x00))
#define GPIOC_CRH *((volatile unsigned int *)(GPIOC_BASE + 0x04))
#define GPIOC_IDR *((volatile uint16_t *)(GPIOC_BASE + 0x08))
#define GPIOC_ODR *((volatile unsigned int *)(GPIOC_BASE + 0x0C))
#define GPIOC_BSRR *((volatile unsigned int *)(GPIOC_BASE + 0x10))
#define GPIOC_BRR *((volatile unsigned int *)(GPIOC_BASE + 0x14))
#define GPIOC_LCKR *((volatile unsigned int *)(GPIOC_BASE + 0x18))

#define GPIOB_BASE 0x40010C00
#define GPIOB_CRL *((volatile unsigned int *)(GPIOB_BASE + 0x00))
#define GPIOB_CRH *((volatile unsigned int *)(GPIOB_BASE + 0x04))
#define GPIOB_IDR *((volatile uint16_t *)(GPIOB_BASE + 0x08))
#define GPIOB_ODR *((volatile unsigned int *)(GPIOB_BASE + 0x0C))
#define GPIOB_BSRR *((volatile unsigned int *)(GPIOB_BASE + 0x10))
#define GPIOB_BRR *((volatile unsigned int *)(GPIOB_BASE + 0x14))
#define GPIOB_LCKR *((volatile unsigned int *)(GPIOB_BASE + 0x18))

#define GPIOA_BASE 0x40010800
#define GPIOA_CRL *((volatile unsigned int *)(GPIOA_BASE + 0x00))
#define GPIOA_CRH *((volatile unsigned int *)(GPIOA_BASE + 0x04))
#define GPIOA_IDR *((volatile uint16_t *)(GPIOA_BASE + 0x08))
#define GPIOA_ODR *((volatile unsigned int *)(GPIOA_BASE + 0x0C))
#define GPIOA_BSRR *((volatile unsigned int *)(GPIOA_BASE + 0x10))
#define GPIOA_BRR *((volatile unsigned int *)(GPIOA_BASE + 0x14))
#define GPIOA_LCKR *((volatile unsigned int *)(GPIOA_BASE + 0x18))

#define AFIO_BASE 0x40010000

#define AFIO_EVCR *((volatile unsigned int *)(AFIO_BASE + 0x00))
#define AFIO_MAPR *((volatile unsigned int *)(AFIO_BASE + 0x04))
#define AFIO_EXTICR1 *((volatile unsigned int *)(AFIO_BASE + 0x08))
#define AFIO_EXTICR2 *((volatile unsigned int *)(AFIO_BASE + 0x0C))
#define AFIO_EXTICR3 *((volatile unsigned int *)(AFIO_BASE + 0x10))
#define AFIO_EXTICR4 *((volatile unsigned int *)(AFIO_BASE + 0x14))
#define AFIO_MAPR2 *((volatile unsigned int *)(AFIO_BASE + 0x1C))

#define TIM2_BASE 0x40000000
#define TIM2_CR1 *((volatile uint16_t *)(TIM2_BASE + 0x00))
#define TIM2_PSC *((volatile uint16_t *)(TIM2_BASE + 0x28))
#define TIM2_ARR *((volatile uint16_t *)(TIM2_BASE + 0x2C))
#define TIM2_SR *((volatile uint16_t *)(TIM2_BASE + 0x10))
#define TIM2_CNT *((volatile uint16_t *)(TIM2_BASE + 0x24))

#define USART1 0x40013800
#define USART1_SR *((volatile unsigned int *)(USART1 + 0x00))
#define USART1_DR *((volatile unsigned int *)(USART1 + 0x04))
#define USART1_BRR *((volatile unsigned int *)(USART1 + 0x08))
#define USART1_CR1 *((volatile unsigned int *)(USART1 + 0x0C))
#define USART1_CR2 *((volatile unsigned int *)(USART1 + 0x10))
#define USART1_CR3 *((volatile unsigned int *)(USART1 + 0x14))
#define USART1_GTPR *((volatile unsigned int *)(USART1 + 0x18))

#define NVIC_BASE 0xE000E100
#define NVIC_ISER0 *((volatile unsigned int *)(NVIC_BASE + 0x00))
#define NVIC_ISER1 *((volatile unsigned int *)(NVIC_BASE + 0x04))
#define NVIC_ISER2 *((volatile unsigned int *)(NVIC_BASE + 0x08))
#define NVIC_ICER0 *((volatile unsigned int *)(NVIC_BASE + 0x80))
#define NVIC_ICER1 *((volatile unsigned int *)(NVIC_BASE + 0x84))
#define NVIC_ICER2 *((volatile unsigned int *)(NVIC_BASE + 0x88))
#define NVIC_ISPR0 *((volatile unsigned int *)(NVIC_BASE + 0x100))
#define NVIC_ISPR1 *((volatile unsigned int *)(NVIC_BASE + 0x104))
#define NVIC_ISPR2 *((volatile unsigned int *)(NVIC_BASE + 0x108))
#define NVIC_ICPR0 *((volatile unsigned int *)(NVIC_BASE + 0x180))
#define NVIC_ICPR1 *((volatile unsigned int *)(NVIC_BASE + 0x184))
#define NVIC_ICPR2 *((volatile unsigned int *)(NVIC_BASE + 0x188))
#define NVIC_IABR0 *((volatile unsigned int *)(NVIC_BASE + 0x200))
#define NVIC_IABR1 *((volatile unsigned int *)(NVIC_BASE + 0x204))
#define NVIC_IABR2 *((volatile unsigned int *)(NVIC_BASE + 0x208))

#define NVIC_IPR0 *((volatile unsigned int *)(NVIC_BASE + 0x300 + (0 * 4)))
#define NVIC_IPR3 *((volatile unsigned int *)(NVIC_BASE + 0x300 + (3 * 4)))
#define NVIC_IPR9 *((volatile unsigned int *)(NVIC_BASE + 0x300 + (9 * 4)))

// EXTI
#define EXTI_BASE 0x40010400

#define EXTI_IMR *((volatile unsigned int *)(EXTI_BASE + 0x00))
#define EXTI_EMR *((volatile unsigned int *)(EXTI_BASE + 0x04))
#define EXTI_RTSR *((volatile unsigned int *)(EXTI_BASE + 0x08))
#define EXTI_FTSR *((volatile unsigned int *)(EXTI_BASE + 0x0C))
#define EXTI_SWIER *((volatile unsigned int *)(EXTI_BASE + 0x10))
#define EXTI_PR *((volatile unsigned int *)(EXTI_BASE + 0x14))

// DMA1 Base Address
#define DMA1_BASE 0x40020000
#define DMA1_ISR *((volatile unsigned int *)(DMA1_BASE + 0x00))
#define DMA1_IFCR *((volatile unsigned int *)(DMA1_BASE + 0x04))

// DMA1 Channel 1
#define DMA1_CCR1 *((volatile unsigned int *)(DMA1_BASE + 0x08))
#define DMA1_CNDTR1 *((volatile unsigned int *)(DMA1_BASE + 0x0C))
#define DMA1_CPAR1 *((volatile unsigned int *)(DMA1_BASE + 0x10))
#define DMA1_CMAR1 *((volatile unsigned int *)(DMA1_BASE + 0x14))

// DMA1 Channel 2
#define DMA1_CCR2 *((volatile unsigned int *)(DMA1_BASE + 0x1C))
#define DMA1_CNDTR2 *((volatile unsigned int *)(DMA1_BASE + 0x20))
#define DMA1_CPAR2 *((volatile unsigned int *)(DMA1_BASE + 0x24))
#define DMA1_CMAR2 *((volatile unsigned int *)(DMA1_BASE + 0x28))

// DMA1 Channel 3
#define DMA1_CCR3 *((volatile unsigned int *)(DMA1_BASE + 0x30))
#define DMA1_CNDTR3 *((volatile unsigned int *)(DMA1_BASE + 0x34))
#define DMA1_CPAR3 *((volatile unsigned int *)(DMA1_BASE + 0x38))
#define DMA1_CMAR3 *((volatile unsigned int *)(DMA1_BASE + 0x3C))

// DMA1 Channel 4
#define DMA1_CCR4 *((volatile unsigned int *)(DMA1_BASE + 0x44))
#define DMA1_CNDTR4 *((volatile unsigned int *)(DMA1_BASE + 0x48))
#define DMA1_CPAR4 *((volatile unsigned int *)(DMA1_BASE + 0x4C))
#define DMA1_CMAR4 *((volatile unsigned int *)(DMA1_BASE + 0x50))

// DMA1 Channel 5
#define DMA1_CCR5 *((volatile unsigned int *)(DMA1_BASE + 0x58))
#define DMA1_CNDTR5 *((volatile unsigned int *)(DMA1_BASE + 0x5C))
#define DMA1_CPAR5 *((volatile unsigned int *)(DMA1_BASE + 0x60))
#define DMA1_CMAR5 *((volatile unsigned int *)(DMA1_BASE + 0x64))

// DMA1 Channel 6
#define DMA1_CCR6 *((volatile unsigned int *)(DMA1_BASE + 0x6C))
#define DMA1_CNDTR6 *((volatile unsigned int *)(DMA1_BASE + 0x70))
#define DMA1_CPAR6 *((volatile unsigned int *)(DMA1_BASE + 0x74))
#define DMA1_CMAR6 *((volatile unsigned int *)(DMA1_BASE + 0x78))

// DMA1 Channel 7
#define DMA1_CCR7 *((volatile unsigned int *)(DMA1_BASE + 0x80))
#define DMA1_CNDTR7 *((volatile unsigned int *)(DMA1_BASE + 0x84))
#define DMA1_CPAR7 *((volatile unsigned int *)(DMA1_BASE + 0x88))
#define DMA1_CMAR7 *((volatile unsigned int *)(DMA1_BASE + 0x8C))

// DMA2 Base Address
#define DMA2_BASE 0x40020400
#define DMA2_ISR *((volatile unsigned int *)(DMA2_BASE + 0x00))
#define DMA2_IFCR *((volatile unsigned int *)(DMA2_BASE + 0x04))

// DMA2 Channel 1
#define DMA2_CCR1 *((volatile unsigned int *)(DMA2_BASE + 0x08))
#define DMA2_CNDTR1 *((volatile unsigned int *)(DMA2_BASE + 0x0C))
#define DMA2_CPAR1 *((volatile unsigned int *)(DMA2_BASE + 0x10))
#define DMA2_CMAR1 *((volatile unsigned int *)(DMA2_BASE + 0x14))

// DMA2 Channel 2
#define DMA2_CCR2 *((volatile unsigned int *)(DMA2_BASE + 0x1C))
#define DMA2_CNDTR2 *((volatile unsigned int *)(DMA2_BASE + 0x20))
#define DMA2_CPAR2 *((volatile unsigned int *)(DMA2_BASE + 0x24))
#define DMA2_CMAR2 *((volatile unsigned int *)(DMA2_BASE + 0x28))

// DMA2 Channel 3
#define DMA2_CCR3 *((volatile unsigned int *)(DMA2_BASE + 0x30))
#define DMA2_CNDTR3 *((volatile unsigned int *)(DMA2_BASE + 0x34))
#define DMA2_CPAR3 *((volatile unsigned int *)(DMA2_BASE + 0x38))
#define DMA2_CMAR3 *((volatile unsigned int *)(DMA2_BASE + 0x3C))

// DMA2 Channel 4
#define DMA2_CCR4 *((volatile unsigned int *)(DMA2_BASE + 0x44))
#define DMA2_CNDTR4 *((volatile unsigned int *)(DMA2_BASE + 0x48))
#define DMA2_CPAR4 *((volatile unsigned int *)(DMA2_BASE + 0x4C))
#define DMA2_CMAR4 *((volatile unsigned int *)(DMA2_BASE + 0x50))

// DMA2 Channel 5
#define DMA2_CCR5 *((volatile unsigned int *)(DMA2_BASE + 0x58))
#define DMA2_CNDTR5 *((volatile unsigned int *)(DMA2_BASE + 0x5C))
#define DMA2_CPAR5 *((volatile unsigned int *)(DMA2_BASE + 0x60))
#define DMA2_CMAR5 *((volatile unsigned int *)(DMA2_BASE + 0x64))

void system_init(void);
void NVIC_init(void);
void GPIO_init(void);
void DMA1_init(void);
void TIM2_init(void);
void USART1_init(void);
void USART1_DMA_send(uint8_t *buffer, uint16_t length);
int log(uint8_t *string);
void intToStr(int num, char *str, uint16_t size);

void USART1_DMA_receive(uint8_t *buffer, uint16_t length);

void delay_ms(uint16_t ms);
void delay_us(uint16_t us);
void GPIO_toggle13(void);
unsigned char is_button_pressed(void);
void DHT11_init(void);

uint8_t buff = '\0';
uint8_t buff_uart1_send[] = "hello world\n";
uint8_t buff_uart1_receive[256];

int main(void)
{
    system_init();
    NVIC_init();
    GPIO_init();
    DMA1_init();
    TIM2_init();
    USART1_init();
    DHT11_init();
    log("START!12345");
    while (1)
    {
    }
}
//
// ��������
//
void GPIO_toggle13(void)
{
    GPIOC_ODR ^= 1 << 13;
}

void delay_ms(uint16_t ms)
{
    TIM2_PSC = 7200 - 1;      // ʹ��7200-1��Ԥ��Ƶֵ��ʹ��ÿ����������Ϊ0.1ms
    TIM2_ARR = (ms - 1) * 10; // �����Զ�����ֵΪ������ӳٺ�����
    TIM2_CNT = 0;             // ����CNT�Ĵ���
    TIM2_CR1 |= 1;            // ������ʱ��
    while (!(TIM2_SR & 1))
    {
    }               // �ȴ������¼���־λ
    TIM2_SR = 0;    // ��������¼���־λ
    TIM2_CR1 &= ~1; // �رն�ʱ��
}

void delay_us(uint16_t us)
{

    TIM2_PSC = 72 - 1; // ʹ��72-1��Ԥ��Ƶֵ��ʹ��ÿ����������Ϊ1us
    TIM2_ARR = us - 1; // �����Զ�����ֵΪ������ӳ�΢����
    TIM2_CNT = 0;      // ����CNT�Ĵ���
    TIM2_CR1 |= 1;     // ������ʱ��
    while (!(TIM2_SR & 1))
    {
    }               // �ȴ������¼���־λ
    TIM2_SR = 0;    // ��������¼���־λ
    TIM2_CR1 &= ~1; // �رն�ʱ��
}

unsigned char is_button_pressed(void)
{
    static unsigned char button_state = 0;
    unsigned char new_state = !(GPIOA_IDR & (1 << 1));

    // ���״̬�����ı�
    if (button_state != new_state)
    {
        delay_ms(10);
        if (new_state == !(GPIOA_IDR & (1 << 1)))
        {
            button_state = new_state;
            GPIO_toggle13();
        }
    }
    return button_state;
}

void system_init(void)
{
    // RCC����

    RCC_CR |= (1 << 16);          // ����HSE
    while (!(RCC_CR & (1 << 17))) // �ȴ�HSE׼����
    {
    }
    RCC_CFGR &= 0b00;                               // SW HSI
    while ((RCC_CFGR & (0b11 << 2)) != (0b00 << 2)) // �ȴ�HSIʱ�ӱ�ѡΪϵͳʱ��
    {
    }
    RCC_CR &= ~(1 << 24);      // �ر�PLL
    while (RCC_CR & (1 << 25)) // �ȴ�PLL�ر�
    {
    }
    RCC_CFGR &=~(1<< 17);  // ����PLLXTPRE
    RCC_CFGR |= 1 << 16; // ����ΪPLL����ΪHSE
    RCC_CFGR &= ~(0b1111 << 18); // ���Ϊ0
    RCC_CFGR |= 0b0100 << 18; // ��Ƶ6��,�ﵽ48Mhz
    // RCC_CFGR |= 0b0111 << 18; // ��Ƶ9��,�ﵽ72Mhz
    RCC_CFGR |= 0b0100 << 8;  // ��Ƶ2��,APB1����Ϊ36Mhz
    RCC_CFGR &= ~(0b100 << 24); // ���MCO���
    RCC_CFGR |= 0b100 << 24; // MCO���ϵͳʱ��
    RCC_CR |= 1 << 24; // ʹ��PLL,�ȴ�ready
    while (!RCC_CR & (1 << 25))
    {
    }
    RCC_CFGR |= 0b10;                               // SW PLL
    while ((RCC_CFGR & (0b11 << 2)) != (0b10 << 2)) // �ȴ�PLLʱ�ӱ�ѡΪϵͳʱ��
    {
    }
}

void NVIC_init(void)
{
    NVIC_SetPriority(USART1_IRQn, 10); // USART1���ȼ�10
    NVIC_EnableIRQ(USART1_IRQn);       // ʹ��USART1ȫ���ж�

    NVIC_SetPriority(DMA1_Channel4_IRQn, 11); // DMA14���ȼ�10
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);       // ʹ��DMA14ȫ���ж�

    NVIC_SetPriority(EXTI15_10_IRQn, 12); // �ж����ȼ�
    NVIC_EnableIRQ(EXTI15_10_IRQn);       // ʹ��EXTI0

    /*
    void __disable_irq(void) // Disable Interrupts
    void __enable_irq(void) // Enable Interrupts

    void NVIC_SetPriorityGrouping(uint32_t priority_grouping)
    void NVIC_EnableIRQ(IRQn_t IRQn)
    void NVIC_DisableIRQ(IRQn_t IRQn)
    uint32_t NVIC_GetPendingIRQ (IRQn_t IRQn)
    void NVIC_SetPendingIRQ (IRQn_t IRQn)
    void NVIC_ClearPendingIRQ (IRQn_t IRQn)
    uint32_t NVIC_GetActive (IRQn_t IRQn)
    void NVIC_SetPriority (IRQn_t IRQn, uint32_t priority)
    uint32_t NVIC_GetPriority (IRQn_t IRQn)
    void NVIC_SystemReset (void)
    */

    // EXTI
    // ����EXTI��·�ʹ�����ʽ
    EXTI_IMR |= 1 << 15;
    EXTI_RTSR |= 1 << 15; // �����ش����ж�
    EXTI_FTSR |= 1 << 15; // �½��ش����ж�
}

void GPIO_init(void)
{
    // ʹ��ʱ��
    RCC_APB2ENR |= 1 << 2; // ʹ��GPIOAʱ��GPIOAAAAAAAAAAAAAAAAAAAA
    RCC_APB2ENR |= 1 << 3; // ʹ��GPIOAʱ��GPIOBBBBBBBBBBBBBBBBBBBB
    RCC_APB2ENR |= 1 << 4; // ʹ��PORTCʱ��GPIOCCCCCCCCCCCCCCCCCCCC
    RCC_APB2ENR |= 1;      // ʹ��AFIOEN(Alternate function I/O clock enable)

    // MCO--PA8
    GPIOA_CRH &= ~(0b1111); // ���PA8����λ
    GPIOA_CRH |= 0b1011;    // ����PA8Ϊ�����������

    // LED��
    GPIOC_CRH &= ~(0xF << 20); // �������λ
    GPIOC_CRH |= 0b0011 << 20; // ����PC13Ϊ�������
    // ����
    GPIOA_CRL &= ~(0b1111 << 4); // ���PA1����λ
    GPIOA_CRL |= 0b1000 << 4;    // ����PA1Ϊ����/��������
    GPIOA_ODR |= 0b1 << 1;       // ����PA1Ϊ�ڲ���������
    // �����½������
    GPIOA_CRH &= ~(0b1111 << 28); // ���PA15����λ
    GPIOA_CRH |= 0b0100 << 28;    // ����PA15Ϊ��������
    AFIO_EXTICR4 &= 0;            // ���ӳ��λ
    AFIO_EXTICR4 |= 0b0000 << 12; // ���ó�PA15ӳ��
}

void DMA1_init(void)
{
    RCC_AHBENR |= 1;            // ʹ��DMA1ʱ��
    DMA1_CCR4 &= ~1;            // �ȹر�ͨ��
    DMA1_CCR4 |= 1 << 4;        // ����:�洢��-->����
    DMA1_CCR4 |= 1 << 7;        // �洢������ģʽ
    DMA1_CCR4 |= 1 << 1;        // ʹ��TCIE
    DMA1_CCR4 &= ~(0b00 << 8);  // �������ݿ��8
    DMA1_CCR4 &= ~(0b00 << 10); // �洢�����ݿ��8
    DMA1_CCR4 |= 0b10 << 12;    // ͨ�����ȼ�
}

void TIM2_init(void)
{
    RCC_APB1ENR |= 1; // ʹ��TIM2ʱ��
}

void USART1_init(void)
{
    RCC_APB2ENR |= 1 << 14;       // ʹ��USART1ʱ��
    RCC_APB2ENR |= 1 << 2;        // ʹ��GPIOAʱ��
    GPIOA_CRH &= ~(0xFF << 4);    // ���PA9,PA10����λ
    GPIOA_CRH |= 0b01001011 << 4; // ����PA9Ϊ�����������,PA10Ϊ��������
    USART1_BRR = 72000000 / 9600; // ���ò�����Ϊ9600
    USART1_CR1 &= ~(1 << 12);     // �����ֳ�
    USART1_CR2 &= ~(0b11 << 12);  // ����ֹͣλ
    USART1_CR1 |= 1 << 5;         // ʹ�ܽ����ж�

    USART1_CR1 |= 1 << 2;  // ʹ�ܽ���RE
    USART1_CR1 |= 1 << 3;  // ʹ�ܷ���TE
    USART1_CR1 |= 1 << 13; // ʹ��USART1,UE
}

void USART1_DMA_send(uint8_t *buffer, uint16_t length)
{
    while (DMA1_CCR4 & 1)
    {
    } // ��ȷ���ر�ͨ��
    DMA1_CMAR4 = buffer;
    DMA1_CNDTR4 = length;
    while (!(USART1_SR & (1 << 7)))
    {
    }                        // �ȴ�TXE
    DMA1_CPAR4 = &USART1_DR; // ��������Ĵ�����ַ
    USART1_CR3 |= 1 << 7;    // ʹ�ܴ���1�ķ���DMA
    DMA1_CCR4 |= 1;          // ����ͨ��
    while (!(USART1_SR & (1 << 6)))
    {
    } // �ȴ�TC
}

void USART1_DMA_receive(uint8_t *buffer, uint16_t length)
{
}

int log(uint8_t *string)
{
    uint8_t *start = string;
    uint16_t count_size = 0;
    while (*string++)
    {
        count_size++;
        if (count_size > 1024)
        {
            log("string is too long!");
            return -1;
        }
    }
    USART1_DMA_send(start, count_size);
}

void intToStr(int num, char *str, uint16_t size)
{
    char *start = str;
    if (size == 0) // �������Ļ�������С�Ƿ�Ϊ0
    {
        return;
    }
    if (num < 0) // �������������
    {
        if (size == 1) // ȷ����������С�㹻��Ÿ���
        {
            *str = '\0';
            return;
        }
        *str++ = '-';
        size--;
        num = -num;
    }
    if (num / 10 != 0) // �ݹ�ת������Ϊ�ַ���
    {
        intToStr(num / 10, str, size);
    }
    if (strlen(str) + 2 <= size) // ȷ����������С�㹻��ŵ�ǰ�����ַ���ĩβ��'\0'
    {
        str[strlen(str)] = num % 10 + '0';
    }
    else
    {
        if (start != str) // ��������������󣬾Ͱ��ַ�����ǰ����
        {
            *(str - 1) = '\0';
        }
        else
        {
            *str = '\0';
        }
    }
}

void DHT11_init(void)
{
    // start!
    GPIOB_CRL &= ~(0b1111 << 20); // ���PB5����λ
    GPIOB_CRL |= 0b0011 << 20;    // ����PB5Ϊ���������

    // GPIOB_BRR |= (uint16_t)1 << 5; // ����PB5
    GPIOB_BSRR = (uint16_t)1 << 5; // ����PB5
    // delay_ms(1000);                // ��ʱ18ms
    // delay_us(1000);                // ��ʱ30us
    // GPIOB_BRR |= (uint16_t)1 << 5; // ����PB5

    // check response
    // GPIOB_CRL &= ~(0b1111 << 20); // ���PB5����λ
    // GPIOB_CRL |= 0b0100 << 20;    // ����PB5Ϊ�ϸ�������

    // if (GPIOB_IDR & ((uint16_t)1 << 5))
    // {
    //     log("DHT11 FAIL1\n");
    // }
    // else
    // {
    //     delay_us(80); // �ȴ�80us
    //     if (GPIOB_IDR & ((uint16_t)1 << 5))
    //     {
    //         delay_us(80); // �Ѿ�����,�ȴ�80us
    //         log("DHT11 OK\n");
    //     }
    // }
}

//     uint8_t dht11_check_response(void) {

//     if (!GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN)) { // ���DHT11����������
//         delay_us(80); // �ȴ�80us
//         if (GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN)) { // ���DHT11����������
//             delay_us(80); // �ȴ�80us
//             return 1; // DHT11��Ӧ�ɹ�
//         }
//     }

//     return 0; // DHT11��Ӧʧ��
// }

//
// �жϴ�����
//
void USART1_IRQHandler(void)
{
    GPIO_toggle13();
    USART1_SR &= ~(1 << 5); // ����жϱ�־λ
}

void DMA1_Channel4_IRQHandler(void)
{
    while (DMA1_ISR & (1 << 13))
    {
        DMA1_IFCR |= (1 << 13); // �����������жϱ�־
        DMA1_CCR4 &= ~1;        // �ر�DMAͨ��
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_PR & (1 << 15)) // ����жϱ�־λ�Ƿ�ΪPA15���Ŵ������ж�
    {
        log("IN EXTI\n");
        EXTI_PR |= 1 << 15; // ����жϱ�־λ
    }
}