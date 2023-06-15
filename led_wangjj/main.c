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

#define TIM2_BASE 0x40000000
#define TIM2_CR1 *((volatile unsigned int *)(TIM2_BASE + 0x00))
#define TIM2_CR2 *((volatile unsigned int *)(TIM2_BASE + 0x04))
#define TIM2_SMCR *((volatile unsigned int *)(TIM2_BASE + 0x08))
#define TIM2_DIER *((volatile unsigned int *)(TIM2_BASE + 0x0C))
#define TIM2_SR *((volatile unsigned int *)(TIM2_BASE + 0x10))
#define TIM2_EGR *((volatile unsigned int *)(TIM2_BASE + 0x14))
#define TIM2_CCMR1 *((volatile unsigned int *)(TIM2_BASE + 0x18))
#define TIM2_CCMR2 *((volatile unsigned int *)(TIM2_BASE + 0x1C))
#define TIM2_CCER *((volatile unsigned int *)(TIM2_BASE + 0x20))
#define TIM2_CNT *((volatile unsigned int *)(TIM2_BASE + 0x24))
#define TIM2_CCER *((volatile unsigned int *)(TIM2_BASE + 0x28))
#define TIM2_PSC *((volatile unsigned int *)(TIM2_BASE + 0x20))
#define TIM2_ARR *((volatile unsigned int *)(TIM2_BASE + 0x2C))
#define TIM2_PSC *((volatile unsigned int *)(TIM2_BASE + 0x20))
#define TIM2_CCR1 *((volatile unsigned int *)(TIM2_BASE + 0x34))
#define TIM2_CCR2 *((volatile unsigned int *)(TIM2_BASE + 0x38))
#define TIM2_CCR3 *((volatile unsigned int *)(TIM2_BASE + 0x3C))
#define TIM2_CCR4 *((volatile unsigned int *)(TIM2_BASE + 0x40))
#define TIM2_DCR *((volatile unsigned int *)(TIM2_BASE + 0x48))
#define TIM2_DMAR *((volatile unsigned int *)(TIM2_BASE + 0x4C))

void system_init(void);
void GPIO_toggle13(void);

int main(void)
{
	system_init();
	while (1)
	{
		GPIO_toggle13();
	}
}
//
//函数定义
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
}
void GPIO_toggle13(void)
{
	GPIOC_ODR ^= 1 << 13;
}