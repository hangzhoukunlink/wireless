#include "lpt_ad5347.h"
#include "config.h"
#include "stm32f10x.h"
#include "assert.h"
#include "stdio.h"

/*
	pin map:
			PD0 ~ 9		:	DB0 ~ DB9	10bit
			PD10 ~ 12	:	A0 ~ A2		channel select
			PD13		:	CLR			clear register
			PD14		:	WR
			PD15		:	RD
			PC6			:	LDAC		simultaneously update
*/
#define db_bank GPIOD
#define db_clock RCC_APB2Periph_GPIOD
#define db_pins (\
			GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | \
			GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 \
)
#define channel_select_pins (\
			GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 \
)
#define clr_pin GPIO_Pin_13
#define wr_pin GPIO_Pin_14
#define rd_pin GPIO_Pin_15

#define ldac_bank GPIOC
#define ldac_clock RCC_APB2Periph_GPIOC
#define ldac_pin GPIO_Pin_6

#define clr_set(level)	(GPIO_WriteBit(db_bank, clr_pin, level))
#define wr_set(level)	(GPIO_WriteBit(db_bank, wr_pin, level))
#define rd_set(level)	(GPIO_WriteBit(db_bank, rd_pin, level))
#define ldac_set(level)	(GPIO_WriteBit(ldac_bank, ldac_pin, level))
#define ndelay(ns) do { \
	for(int i = ((ns) >> 3); i > 0 ; i -- ); \
} while(0)
#define mv2d(mv) (((mv) << 10) / 2500)
#define OUTPUT 0
#define INPUT 1
static void ad5347_Init(int choice)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(db_clock, ENABLE);
	GPIO_InitStructure.GPIO_Pin = db_pins;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = ((choice == OUTPUT) ? GPIO_Mode_Out_PP : GPIO_Mode_IPU);
	GPIO_Init(db_bank, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = channel_select_pins | clr_pin | wr_pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(db_bank, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(ldac_clock, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ldac_pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(ldac_bank, &GPIO_InitStructure);
	
	clr_set(Bit_RESET);
}		
static void ad5347_ChannelSelect(unsigned int channel)
{
	assert(channel < 8);
	int reg = db_bank -> ODR;
	reg &= ~(0x07 << 10);
	reg |= (channel << 10);
	db_bank -> ODR = reg;
}
//level 1 0 1: synchronous
//level 0 1 0: asynchonous
static void ad5347_Ldac(BitAction level)
{
	ldac_set(level);
}
//
static void ad5347_Write(unsigned int mv, unsigned int channel)
{
	int temp_digital = mv2d(mv);
	clr_set(Bit_SET);
	ad5347_Ldac(Bit_RESET);
	wr_set(Bit_RESET);
	//ndelay(10);
	//ad5347_Ldac(Bit_SET);
	int reg = db_bank -> ODR;
	reg &= ~(0x3ff << 0);
	reg |= (temp_digital << 0);
	db_bank -> ODR = reg;
	ad5347_ChannelSelect(channel);
	//ndelay(20);	
	wr_set(Bit_SET);
	//ad5347_Ldac(Bit_RESET);
}
//
static int ad5347_Read(unsigned int channel)
{
	ad5347_ChannelSelect(channel);
	rd_set(Bit_RESET);
	ndelay(30);
	int temp_digital = db_bank -> IDR;
	rd_set(Bit_SET);
	return temp_digital;
}
//
static void ad5347_Clear(void)
{
	clr_set(Bit_RESET);
}

const ad5347_t ad5347 = {
	.init = ad5347_Init,
	.write = ad5347_Write,
	.read = ad5347_Read,
	.clear = ad5347_Clear
};
#if 1
#include "shell/cmd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int cmd_ad5347_func(int argc, char *argv[])
{
	const char * usage = { 							
		" usage:\r\n" 								
		" ad5347 init, chip init \r\n" 				
		" ad5347 write date channel\r\n"		
		" ad5347 read channel\r\n"				
		" ad5347 clear\r\n"
		" .. \r\n"									
	};
	if(argc == 2) {
		if(!strcmp(argv[1],"init")) {
			ad5347_Init(OUTPUT);
			printf("db_bank init pp\r\n");
			printf("ad5347 Init Successful!\r\n");
		}
		if(!strcmp(argv[1],"clear")) {
			ad5347_Clear();
			printf("ad5347 clear Successful!\r\n");
		}
	}
	if(argc == 3) {
		if(!strcmp(argv[1],"read")) {
			ad5347_Init(INPUT);
			printf("db_bank init upload\r\n");
			int data_temp = 0;
			int channel = 0;
			sscanf(argv[2],"%d",&channel);
			data_temp = ad5347_Read(channel);
			printf("Data read from ad5347 is :0x%x\r\n",data_temp & 0x03ff);
		}
	}
	if(argc == 4) {
		if(!strcmp(argv[1],"write")) {
			int mv = 0;
			int channel = 0;
			sscanf(argv[2],"%d",&mv);
			sscanf(argv[3],"%d",&channel);
			//ad5347_Init(OUTPUT);
			printf("db_bank init pp\r\n");
			ad5347_Write(mv, channel);
			printf("mv is %d channel is %d\r\n",mv, channel);
			printf("ad5347 write successful!\r\n");
		}
	}
	if(argc < 2) {
		printf(usage);
		return 0;
	}

	return 0;
}
const cmd_t cmd_ad5347 = {"ad5347", cmd_ad5347_func, "ad5347 cmd"};
DECLARE_SHELL_CMD(cmd_ad5347)
#endif
