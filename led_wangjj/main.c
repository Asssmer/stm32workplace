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
#define TIM2_CR1 *((volatile unsigned short *)(TIM2_BASE + 0x00))
#define TIM2_PSC *((volatile unsigned short *)(TIM2_BASE + 0x28))
#define TIM2_ARR *((volatile unsigned short *)(TIM2_BASE + 0x2C))
#define TIM2_SR *((volatile unsigned short *)(TIM2_BASE + 0x10))

void system_init(void);
void delay_ms(unsigned int ms);
void GPIO_toggle13(void);

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
	while (1)
	{
		GPIO_toggle13();
		delay_ms(10000);
	}
}
//
// ��������
//
void system_init(void)
{
	RCC_CR |= (1 << 0); // ����HSI
	while (!(RCC_CR & (1 << 1)))
	{
	}; // �ȴ�HSI׼����

	RCC_CFGR |= (int)1 << 16; // ����ΪPLL����
	RCC_CFGR |= 0b0111 << 18; // ��Ƶ9��,�ﵽ72Mhz
	RCC_CFGR |= 0b0100 << 8;  // ��Ƶ2��,APB1����Ϊ36Mhz
	RCC_CR |= (int)1 << 24;	  // ʹ��PLL,�ȴ�ready
	while (!RCC_CR & ((int)1 << 25))
	{
	}
	RCC_APB2ENR |= 1 << 4;				   // ʹ��PORTCʱ��
	GPIOC_CRH &= ~(0xF << ((13 - 8) * 4)); // �������λ
	GPIOC_CRH |= 0b0011 << ((13 - 8) * 4); // ����PC13Ϊ�������

	RCC_APB1ENR |= 1; // ʹ��TIM2ʱ��
}
void GPIO_toggle13(void)
{
	GPIOC_ODR ^= 1 << 13;
}

void delay_ms(unsigned int ms)
{

	TIM2_PSC = 7200 - 1; // ʹ��7200-1��Ԥ��Ƶֵ��ʹ��ÿ����������Ϊ0.1ms
	TIM2_ARR = ms;		 // �����Զ�����ֵΪ������ӳٺ�����
	TIM2_CR1 |= 1;		 // ������ʱ��
	while (!(TIM2_SR & 1))
	{
	}				// �ȴ������¼���־λ
	TIM2_SR &= ~1;	// ��������¼���־λ
	TIM2_CR1 &= ~1; // �رն�ʱ��
}