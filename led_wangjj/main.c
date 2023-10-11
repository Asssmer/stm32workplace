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
// 函数定义
//
void GPIO_toggle13(void)
{
    GPIOC_ODR ^= 1 << 13;
}

void delay_ms(uint16_t ms)
{
    TIM2_PSC = 7200 - 1;      // 使用7200-1的预分频值，使得每个计数周期为0.1ms
    TIM2_ARR = (ms - 1) * 10; // 设置自动重载值为所需的延迟毫秒数
    TIM2_CNT = 0;             // 清零CNT寄存器
    TIM2_CR1 |= 1;            // 开启定时器
    while (!(TIM2_SR & 1))
    {
    }               // 等待更新事件标志位
    TIM2_SR = 0;    // 清除更新事件标志位
    TIM2_CR1 &= ~1; // 关闭定时器
}

void delay_us(uint16_t us)
{

    TIM2_PSC = 72 - 1; // 使用72-1的预分频值，使得每个计数周期为1us
    TIM2_ARR = us - 1; // 设置自动重载值为所需的延迟微秒数
    TIM2_CNT = 0;      // 清零CNT寄存器
    TIM2_CR1 |= 1;     // 开启定时器
    while (!(TIM2_SR & 1))
    {
    }               // 等待更新事件标志位
    TIM2_SR = 0;    // 清除更新事件标志位
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

    RCC_CR |= (1 << 16);          // 开启HSE
    while (!(RCC_CR & (1 << 17))) // 等待HSE准备好
    {
    }
    RCC_CFGR &= 0b00;                               // SW HSI
    while ((RCC_CFGR & (0b11 << 2)) != (0b00 << 2)) // 等待HSI时钟被选为系统时钟
    {
    }
    RCC_CR &= ~(1 << 24);      // 关闭PLL
    while (RCC_CR & (1 << 25)) // 等待PLL关闭
    {
    }
    RCC_CFGR &=~(1<< 17);  // 配置PLLXTPRE
    RCC_CFGR |= 1 << 16; // 设置为PLL输入为HSE
    RCC_CFGR &= ~(0b1111 << 18); // 清除为0
    RCC_CFGR |= 0b0100 << 18; // 分频6倍,达到48Mhz
    // RCC_CFGR |= 0b0111 << 18; // 分频9倍,达到72Mhz
    RCC_CFGR |= 0b0100 << 8;  // 分频2倍,APB1降低为36Mhz
    RCC_CFGR &= ~(0b100 << 24); // 清空MCO输出
    RCC_CFGR |= 0b100 << 24; // MCO输出系统时钟
    RCC_CR |= 1 << 24; // 使能PLL,等待ready
    while (!RCC_CR & (1 << 25))
    {
    }
    RCC_CFGR |= 0b10;                               // SW PLL
    while ((RCC_CFGR & (0b11 << 2)) != (0b10 << 2)) // 等待PLL时钟被选为系统时钟
    {
    }
}

void NVIC_init(void)
{
    NVIC_SetPriority(USART1_IRQn, 10); // USART1优先级10
    NVIC_EnableIRQ(USART1_IRQn);       // 使能USART1全局中断

    NVIC_SetPriority(DMA1_Channel4_IRQn, 11); // DMA14优先级10
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);       // 使能DMA14全局中断

    NVIC_SetPriority(EXTI15_10_IRQn, 12); // 中断优先级
    NVIC_EnableIRQ(EXTI15_10_IRQn);       // 使能EXTI0

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
    // 配置EXTI线路和触发方式
    EXTI_IMR |= 1 << 15;
    EXTI_RTSR |= 1 << 15; // 上升沿触发中断
    EXTI_FTSR |= 1 << 15; // 下降沿触发中断
}

void GPIO_init(void)
{
    // 使能时钟
    RCC_APB2ENR |= 1 << 2; // 使能GPIOA时钟GPIOAAAAAAAAAAAAAAAAAAAA
    RCC_APB2ENR |= 1 << 3; // 使能GPIOA时钟GPIOBBBBBBBBBBBBBBBBBBBB
    RCC_APB2ENR |= 1 << 4; // 使能PORTC时钟GPIOCCCCCCCCCCCCCCCCCCCC
    RCC_APB2ENR |= 1;      // 使能AFIOEN(Alternate function I/O clock enable)

    // MCO--PA8
    GPIOA_CRH &= ~(0b1111); // 清除PA8控制位
    GPIOA_CRH |= 0b1011;    // 设置PA8为复用推挽输出

    // LED灯
    GPIOC_CRH &= ~(0xF << 20); // 清除控制位
    GPIOC_CRH |= 0b0011 << 20; // 设置PC13为推挽输出
    // 按键
    GPIOA_CRL &= ~(0b1111 << 4); // 清除PA1控制位
    GPIOA_CRL |= 0b1000 << 4;    // 设置PA1为上拉/下拉输入
    GPIOA_ODR |= 0b1 << 1;       // 启用PA1为内部上拉电阻
    // 上升下降沿侦测
    GPIOA_CRH &= ~(0b1111 << 28); // 清除PA15控制位
    GPIOA_CRH |= 0b0100 << 28;    // 设置PA15为浮空输入
    AFIO_EXTICR4 &= 0;            // 清除映射位
    AFIO_EXTICR4 |= 0b0000 << 12; // 设置成PA15映射
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
    RCC_APB2ENR |= 1 << 14;       // 使能USART1时钟
    RCC_APB2ENR |= 1 << 2;        // 使能GPIOA时钟
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
    while (DMA1_CCR4 & 1)
    {
    } // 先确定关闭通道
    DMA1_CMAR4 = buffer;
    DMA1_CNDTR4 = length;
    while (!(USART1_SR & (1 << 7)))
    {
    }                        // 等待TXE
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
    if (size == 0) // 检查输入的缓冲区大小是否为0
    {
        return;
    }
    if (num < 0) // 处理负整数的情况
    {
        if (size == 1) // 确保缓冲区大小足够存放负号
        {
            *str = '\0';
            return;
        }
        *str++ = '-';
        size--;
        num = -num;
    }
    if (num / 10 != 0) // 递归转换数字为字符串
    {
        intToStr(num / 10, str, size);
    }
    if (strlen(str) + 2 <= size) // 确保缓冲区大小足够存放当前数字字符和末尾的'\0'
    {
        str[strlen(str)] = num % 10 + '0';
    }
    else
    {
        if (start != str) // 如果缓冲区不够大，就把字符串提前结束
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
    GPIOB_CRL &= ~(0b1111 << 20); // 清除PB5控制位
    GPIOB_CRL |= 0b0011 << 20;    // 设置PB5为上推挽输出

    // GPIOB_BRR |= (uint16_t)1 << 5; // 拉低PB5
    GPIOB_BSRR = (uint16_t)1 << 5; // 拉高PB5
    // delay_ms(1000);                // 延时18ms
    // delay_us(1000);                // 延时30us
    // GPIOB_BRR |= (uint16_t)1 << 5; // 拉低PB5

    // check response
    // GPIOB_CRL &= ~(0b1111 << 20); // 清除PB5控制位
    // GPIOB_CRL |= 0b0100 << 20;    // 设置PB5为上浮空输入

    // if (GPIOB_IDR & ((uint16_t)1 << 5))
    // {
    //     log("DHT11 FAIL1\n");
    // }
    // else
    // {
    //     delay_us(80); // 等待80us
    //     if (GPIOB_IDR & ((uint16_t)1 << 5))
    //     {
    //         delay_us(80); // 已经拉高,等待80us
    //         log("DHT11 OK\n");
    //     }
    // }
}

//     uint8_t dht11_check_response(void) {

//     if (!GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN)) { // 如果DHT11拉低数据线
//         delay_us(80); // 等待80us
//         if (GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN)) { // 如果DHT11拉高数据线
//             delay_us(80); // 等待80us
//             return 1; // DHT11响应成功
//         }
//     }

//     return 0; // DHT11响应失败
// }

//
// 中断处理函数
//
void USART1_IRQHandler(void)
{
    GPIO_toggle13();
    USART1_SR &= ~(1 << 5); // 清除中断标志位
}

void DMA1_Channel4_IRQHandler(void)
{
    while (DMA1_ISR & (1 << 13))
    {
        DMA1_IFCR |= (1 << 13); // 清除传输完成中断标志
        DMA1_CCR4 &= ~1;        // 关闭DMA通道
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_PR & (1 << 15)) // 检查中断标志位是否为PA15引脚触发的中断
    {
        log("IN EXTI\n");
        EXTI_PR |= 1 << 15; // 清除中断标志位
    }
}