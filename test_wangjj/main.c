#define STM32F10X_MD
#include "stm32f10x.h"

#ifndef int8_t
typedef signed char int8_t;
typedef short int16_t;
typedef int int32_t;
typedef long long int64_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
#endif

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
#define GPIOC_IDR *((volatile unsigned int *)(GPIOC_BASE + 0x08))
#define GPIOC_ODR *((volatile unsigned int *)(GPIOC_BASE + 0x0C))
#define GPIOC_BSRR *((volatile unsigned int *)(GPIOC_BASE + 0x10))
#define GPIOC_BRR *((volatile unsigned int *)(GPIOC_BASE + 0x14))
#define GPIOC_LCKR *((volatile unsigned int *)(GPIOC_BASE + 0x18))

#define GPIOA_BASE 0x40010800
#define GPIOA_CRL *((volatile unsigned int *)(GPIOA_BASE + 0x00))
#define GPIOA_CRH *((volatile unsigned int *)(GPIOA_BASE + 0x04))
#define GPIOA_IDR *((volatile uint16_t *)(GPIOA_BASE + 0x08))
#define GPIOA_ODR *((volatile unsigned int *)(GPIOA_BASE + 0x0C))
#define GPIOA_BSRR *((volatile unsigned int *)(GPIOA_BASE + 0x10))
#define GPIOA_BRR *((volatile unsigned int *)(GPIOA_BASE + 0x14))
#define GPIOA_LCKR *((volatile unsigned int *)(GPIOA_BASE + 0x18))

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
void USART1_DMA_receive(uint8_t *buffer, uint16_t length);

void delay_ms(uint32_t ms);
void GPIO_toggle13(void);
unsigned char is_button_pressed(void);

uint8_t buff = '\0';
uint8_t buff_uart1_send[] = "hello world\n";
uint8_t buff_uart1_receive[256];

int main(void)
{

}
//
// 函数定义
//
void GPIO_toggle13(void)
{
    GPIOC_ODR ^= 1 << 13;
}
void delay_ms(unsigned int ms)
{

    TIM2_PSC = 7200 - 1; // 使用7200-1的预分频值，使得每个计数周期为0.1ms
    TIM2_ARR = ms * 10;  // 设置自动重载值为所需的延迟毫秒数
    TIM2_CNT = 0;        // 清零CNT寄存器
    TIM2_CR1 |= 1;       // 开启定时器
    while (!(TIM2_SR & 1))
    {
    }               // 等待更新事件标志位
    TIM2_SR &= ~1;  // 清除更新事件标志位
    TIM2_CR1 &= ~1; // 关闭定时器
}

unsigned char is_button_pressed(void)
{
    static unsigned char button_state = 0;
    unsigned char new_state = !(GPIOA_IDR & (1 << 1));

    // 如果状态发生改变
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
    // RCC配置
    RCC_CR |= (1 << 0); // 开启HSI
    while (!(RCC_CR & (1 << 1)))
    {
    }; // 等待HSI准备好

    RCC_CFGR |= (int)1 << 16; // 设置为PLL输入
    RCC_CFGR |= 0b0111 << 18; // 分频9倍,达到72Mhz
    RCC_CFGR |= 0b0100 << 8;  // 分频2倍,APB1降低为36Mhz
    RCC_CR |= (int)1 << 24;   // 使能PLL,等待ready
    while (!RCC_CR & ((int)1 << 25))
    {
    }
}

void NVIC_init(void)
{
    NVIC_EnableIRQ(USART1_IRQn);       // 使能USART1全局中断
    NVIC_SetPriority(USART1_IRQn, 10); // USART1优先级10

    NVIC_EnableIRQ(DMA1_Channel4_IRQn);       // 使能DMA14全局中断
    NVIC_SetPriority(DMA1_Channel4_IRQn, 10); // DMA14优先级10

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
}

void GPIO_init(void)
{
    // LED灯
    RCC_APB2ENR |= 1 << 4;                 // 使能PORTC时钟
    GPIOC_CRH &= ~(0xF << ((13 - 8) * 4)); // 清除控制位
    GPIOC_CRH |= 0b0011 << ((13 - 8) * 4); // 设置PC13为推挽输出
    // 按键
    RCC_APB2ENR |= 1 << 14;      // 使能GPIOA时钟
    GPIOA_CRL &= ~(0b1111 << 4); // 清除PA1控制位
    GPIOA_CRL |= 0b1000 << 4;    // 设置PA1为上拉/下拉输入
    GPIOA_ODR |= 0b1 << 1;       // 启用PA1为内部上拉电阻
}

void DMA1_init(void)
{
    RCC_AHBENR |= 1;            // 使能DMA1时钟
    DMA1_CCR4 &= ~1;            // 先关闭通道
    DMA1_CCR4 |= 1 << 4;        // 方向:存储器-->外设
    DMA1_CCR4 |= 1 << 7;        // 存储器增量模式
    DMA1_CCR4 |= 1 << 1;        // 使能TCIE
    DMA1_CCR4 &= ~(0b00 << 8);  // 外设数据宽度8
    DMA1_CCR4 &= ~(0b00 << 10); // 存储器数据宽度8
    DMA1_CCR4 |= 0b10 << 12;    // 通道优先级
}

void TIM2_init(void)
{
    RCC_APB1ENR |= 1; // 使能TIM2时钟
}

void USART1_init(void)
{
    RCC_APB2ENR |= 1 << 14;       // 使能GPIOA时钟
    RCC_APB2ENR |= 1 << 2;        // 使能USART1时钟
    GPIOA_CRH &= ~(0xFF << 4);    // 清除PA9,PA10控制位
    GPIOA_CRH |= 0b01001011 << 4; // 设置PA9为复用推挽输出,PA10为浮空输入
    USART1_BRR = 72000000 / 9600; // 设置波特率为9600
    USART1_CR1 &= ~(1 << 12);     // 设置字长
    USART1_CR2 &= ~(0b11 << 12);  // 设置停止位
    USART1_CR1 |= 1 << 5;         // 使能接收中断

    USART1_CR1 |= 1 << 2;  // 使能接收RE
    USART1_CR1 |= 1 << 3;  // 使能发送TE
    USART1_CR1 |= 1 << 13; // 使能USART1,UE
}

void USART1_DMA_send(uint8_t *buffer, uint16_t length)
{
    // while (DMA1_CCR4 & 1)
    // {
    // } // 先确定关闭通道
    DMA1_CMAR4 = buffer;
    DMA1_CNDTR4 = length;
    DMA1_CPAR4 = &USART1_DR; // 设置外设寄存器地址
    USART1_CR3 |= 1 << 7;    // 使能串口1的发送DMA
    DMA1_CCR4 |= 1;          // 开启通道
    while (!(USART1_SR & (1 << 6)))
    {
    } // 等待TC
}

void USART1_DMA_receive(uint8_t *buffer, uint16_t length)
{
}

//
// 中断处理函数
//
void USART1_IRQHandler(void)
{
    buff = USART1_DR;
    GPIO_toggle13();
}

void DMA1_Channel4_IRQHandler(void)
{
    while (DMA1_ISR & (1 << 13))
    {
        GPIO_toggle13();
        DMA1_IFCR |= (1 << 13); // 清除传输完成中断标志
        DMA1_CCR4 &= ~1;        // 关闭DMA通道
    }
}