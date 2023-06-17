#include "stm32f10x.h"
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
#define GPIOA_IDR *((volatile unsigned int *)(GPIOA_BASE + 0x08))
#define GPIOA_ODR *((volatile unsigned int *)(GPIOA_BASE + 0x0C))
#define GPIOA_BSRR *((volatile unsigned int *)(GPIOA_BASE + 0x10))
#define GPIOA_BRR *((volatile unsigned int *)(GPIOA_BASE + 0x14))
#define GPIOA_LCKR *((volatile unsigned int *)(GPIOA_BASE + 0x18))

#define TIM2_BASE 0x40000000
#define TIM2_CR1 *((volatile unsigned short *)(TIM2_BASE + 0x00))
#define TIM2_PSC *((volatile unsigned short *)(TIM2_BASE + 0x28))
#define TIM2_ARR *((volatile unsigned short *)(TIM2_BASE + 0x2C))
#define TIM2_SR *((volatile unsigned short *)(TIM2_BASE + 0x10))

#define USART1 0x40013800
#define USART1_SR *((volatile unsigned int *)(USART1 + 0x00))
#define USART1_DR *((volatile unsigned int *)(USART1 + 0x04))
#define USART1_BRR *((volatile unsigned int *)(USART1 + 0x08))
#define USART1_CR1 *((volatile unsigned int *)(USART1 + 0x0C))
#define USART1_CR2 *((volatile unsigned int *)(USART1 + 0x10))
#define USART1_CR3 *((volatile unsigned int *)(USART1 + 0x14))
#define USART1_GTPR *((volatile unsigned int *)(USART1 + 0x18))

void system_init(void);
void delay_ms(unsigned int ms);
void GPIO_toggle13(void);
void UART1_send(unsigned char *data);
void UART1_receive(unsigned char *data);

/*
char
unsigned char
signed char
int
unsigned int
short
unsigned short
long
unsigned long

typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
typedef long long          int64_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long long uint64_t;
*/
unsigned char buff = 0x44;

#include "stm32f10x.h"

void USART1_Init(void) 
{
    // 使能USART1和GPIOA时钟
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;
    
    // 设置PA9为复用推挽输出
    GPIOA_CRH &= ~GPIO_CRH_MODE9;
    GPIOA_CRH |= GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1;
    GPIOA_CRH &= ~GPIO_CRH_CNF9;
    GPIOA_CRH |= GPIO_CRH_CNF9_1;
    
    // 设置PA10为输入悬空（浮空输入）
    GPIOA_CRH &= ~GPIO_CRH_MODE10;
    GPIOA_CRH &= ~GPIO_CRH_CNF10;
    GPIOA_CRH |= GPIO_CRH_CNF10_0;
    
    // 设置USART参数
    USART1_BRR = 0x1D4C; // 波特率为9600，假设PCLK2 = 72MHz
    USART1_CR1 |= USART_CR1_TE; // 使能串口发送
    USART1_CR1 |= USART_CR1_UE; // 使能串口
}

void USART1_SendChar(char ch)
{
    while (!(USART1_SR & USART_SR_TXE)); // 等待发送数据寄存器为空
    USART1_DR = (uint8_t)ch; // 写入数据
}

int main(void)
{
    USART1_Init();
    
    while (1)
    {
        USART1_SendChar('H');
        USART1_SendChar('e');
        USART1_SendChar('l');
        USART1_SendChar('l');
        USART1_SendChar('o');
        USART1_SendChar('\r');
        USART1_SendChar('\n');
        
        for (volatile int i = 0; i < 5000000; i++); // 延迟
    }
}
