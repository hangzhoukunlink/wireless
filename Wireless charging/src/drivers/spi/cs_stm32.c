/*
*	miaofng@2010 initial version
*/

#include "config.h"
#include "spi.h"

#ifdef CONFIG_CPU_STM32
#include "stm32f10x.h"

int spi_cs_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

#ifdef CONFIG_SPI_CS_PA4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PA12
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PB1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PB10
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PB12
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PC3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PC4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PC5
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PC7
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PC8
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PF11
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PB0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PB6
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PB7
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PD12
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);
#endif

#ifdef CONFIG_SPI_CS_PA3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PA2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PD8
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif

#ifdef CONFIG_SPI_CS_PD9
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif

	return 0;
}

int spi_cs_set(int addr, int level)
{
	BitAction ba = Bit_RESET;
	if(level)
		ba = Bit_SET;

#ifdef CONFIG_SPI_CS_PA4
	if(addr == SPI_CS_PA4) {
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PA12
	if(addr == SPI_CS_PA12) {
		GPIO_WriteBit(GPIOA, GPIO_Pin_12, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PB1
	if(addr == SPI_CS_PB1) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_1, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PB6
	if(addr == SPI_CS_PB6) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_6, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PB7
	if(addr == SPI_CS_PB7) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_7, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PB10
	if(addr == SPI_CS_PB10) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_10, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PB12
	if(addr == SPI_CS_PB12) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PC3
	if(addr == SPI_CS_PC3) {
		GPIO_WriteBit(GPIOC, GPIO_Pin_3, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PC4
	if(addr == SPI_CS_PC4) {
		GPIO_WriteBit(GPIOC, GPIO_Pin_4, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PC5
	if(addr == SPI_CS_PC5) {
		GPIO_WriteBit(GPIOC, GPIO_Pin_5, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PC7
	if(addr == SPI_CS_PC7) {
		GPIO_WriteBit(GPIOC, GPIO_Pin_7, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PC8
	if(addr == SPI_CS_PC8) {
		GPIO_WriteBit(GPIOC, GPIO_Pin_8, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PF11
	if(addr == SPI_CS_PF11) {
		GPIO_WriteBit(GPIOF, GPIO_Pin_11, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PB0
	if(addr == SPI_CS_PB0) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_0, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PD12
	if(addr == SPI_CS_PD12) {
		GPIO_WriteBit(GPIOD, GPIO_Pin_12, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PA3
	if(addr == SPI_CS_PA3) {
		GPIO_WriteBit(GPIOA, GPIO_Pin_3, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PA2
	if(addr == SPI_CS_PA2) {
		GPIO_WriteBit(GPIOA, GPIO_Pin_2, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PD8
	if(addr == SPI_CS_PD8) {
		GPIO_WriteBit(GPIOD, GPIO_Pin_8, ba);
	}
#endif

#ifdef CONFIG_SPI_CS_PD9
	if(addr == SPI_CS_PD9) {
		GPIO_WriteBit(GPIOD, GPIO_Pin_9, ba);
	}
#endif
	return 0;
}

#endif