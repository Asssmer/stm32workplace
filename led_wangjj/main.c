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
#define GPIOA_CRL *((volatile unsigned int *)(GPIOC_BASE + 0x00))
#define GPIOA_CRH *((volatile unsigned int *)(GPIOC_BASE + 0x04))
#define GPIOA_IDR *((volatile unsigned int *)(GPIOC_BASE + 0x08))
#define GPIOA_ODR *((volatile unsigned int *)(GPIOC_BASE + 0x0C))
#define GPIOA_BSRR *((volatile unsigned int *)(GPIOC_BASE + 0x10))
#define GPIOA_BRR *((volatile unsigned int *)(GPIOC_BASE + 0x14))
#define GPIOA_LCKR *((volatile unsigned int *)(GPIOC_BASE + 0x18))

#define TIM2_BASE 0x40000000
#define TIM2_CR1 *((volatile unsigned short *)(TIM2_BASE + 0x00))
#define TIM2_PSC *((volatile unsigned short *)(TIM2_BASE + 0x28))
#define TIM2_ARR *((volatile unsigned short *)(TIM2_BASE + 0x2C))
#define TIM2_SR *((volatile unsigned short *)(TIM2_BASE + 0x10))

#define USART1 0x40013800
#define USART1_SR *((volatile unsigned int *)(TIM2_BASE + 0x00))
#define USART1_DR *((volatile unsigned char *)(TIM2_BASE + 0x04))
#define USART1_BRR *((volatile unsigned int *)(TIM2_BASE + 0x08))
#define USART1_CR1 *((volatile unsigned int *)(TIM2_BASE + 0x0C))
#define USART1_CR2 *((volatile unsigned int *)(TIM2_BASE + 0x10))
#define USART1_CR3 *((volatile unsigned int *)(TIM2_BASE + 0x14))
#define USART1_GTPR *((volatile unsigned int *)(TIM2_BASE + 0x18))

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
*/

int main(void)
{
	system_init();
	// while (1)
	// {
	// 	GPIO_toggle13();
	// 	delay_ms(10000);
	// }
	unsigned char buff='/0';
	while(1){
		UART1_receive(&buff);
		UART1_send(&buff);
	}
}
//
// 函数定义
//
void system_init(void)
{
	RCC_CR |= (1 << 0); // 开启HSI
	while (!(RCC_CR & (1 << 1)))
	{
	}; // 等待HSI准备好

	RCC_CFGR |= (int)1 << 16; // 设置为PLL输入
	RCC_CFGR |= 0b0111 << 18; // 分频9倍,达到72Mhz
	RCC_CFGR |= 0b0100 << 8;  // 分频2倍,APB1降低为36Mhz
	RCC_CR |= (int)1 << 24;	  // 使能PLL,等待ready
	while (!RCC_CR & ((int)1 << 25))
	{
	}
	RCC_APB2ENR |= 1 << 4;				   // 使能PORTC时钟
	GPIOC_CRH &= ~(0xF << ((13 - 8) * 4)); // 清除控制位
	GPIOC_CRH |= 0b0011 << ((13 - 8) * 4); // 设置PC13为推挽输出

	RCC_APB1ENR |= 1; // 使能TIM2时钟

	RCC_APB2ENR |= 1 << 2;	// 使能USART1时钟
	RCC_APB2ENR |= 1 << 14; // 使能GPIOA时钟

	GPIOA_CRH &= ~0xFF << 4;	  // 清除PA9,PA10控制位
	GPIOC_CRH |= 0b01000011 << 4; // 设置PA9为推挽输出,PA10为浮空输入

	USART1_BRR = 72000000 / 9600; // 设置波特率为9600
	USART1_CR1 |= 1 << 3;		  // 使能发送TE
	USART1_CR1 |= 1 << 2;		  // 使能接收RE
	USART1_CR1 |= 1 << 13;		  // 使能USART1,UE
}
void GPIO_toggle13(void)
{
	GPIOC_ODR ^= 1 << 13;
}
void delay_ms(unsigned int ms)
{

	TIM2_PSC = 7200 - 1; // 使用7200-1的预分频值，使得每个计数周期为0.1ms
	TIM2_ARR = ms;		 // 设置自动重载值为所需的延迟毫秒数
	TIM2_CR1 |= 1;		 // 开启定时器
	while (!(TIM2_SR & 1))
	{
	}				// 等待更新事件标志位
	TIM2_SR &= ~1;	// 清除更新事件标志位
	TIM2_CR1 &= ~1; // 关闭定时器
}
void UART1_send(unsigned char *data)
{
	if (USART1_SR & (1 << 7))
	{					  // 如果TXE标志位为1（发送数据寄存器空）
		USART1_DR = *data; // 写入数据
	}
}
void UART1_receive(unsigned char *data)
{
	if (USART1_SR & (1 << 5))
	{					  // 如果RXNE标志位为1（接收到数据）
		*data = USART1_DR; // 读取数据
	}
}