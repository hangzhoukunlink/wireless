/*
 *jiamao.gu @2013 first version
 *interface: spi, 40Mhz, 16bit, CPOL=0(sck low idle), 
				CPHA=0(latch at 1st edge/rising edge of sck), 
				Y4 first, X0 last
*/
#include "spi.h"
#include "ch446.h"
#include "stm32f10x.h"
#include "assert.h"
#include <stdio.h>
#define CH446Q 1
#define CH446X
#define ndelay(ns) do { \
	for(int i = ((ns) >> 3); i > 0 ; i -- ); \
} while(0)
#define SPI 0
#define rst_set(level)	(GPIO_WriteBit(GPIOE, GPIO_Pin_1, level))
void ch446_Init(const ch446_t *chip)
{
	#if SPI
	spi_cfg_t cfg = {
		.cpol = 0,
		.cpha = 0,
		.bits = 8,
		.bseq = 1,
		.freq = 1000000,
	};
	chip->bus->init(&cfg);
	#else
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	rst_set(Bit_SET);
	ndelay(100);
	rst_set(Bit_RESET);
	#endif
}
//
#define clk_set(level)	(GPIO_WriteBit(GPIOB, GPIO_Pin_13, level))
#define date_set(level)	(GPIO_WriteBit(GPIOB, GPIO_Pin_15, level))
#define stb_set(level)	(GPIO_WriteBit(GPIOB, GPIO_Pin_9, level))
void ch446_Switch(const ch446_t *chip, int xy, int on)
{
	xy <<= 1; // give up a bit
	for(int i = 0; i < 8; i++) {
		clk_set(Bit_RESET);
		ndelay(20);
		clk_set(Bit_SET);
		if((xy & 0x80) == 0x80) {
			date_set(Bit_SET);
		}
		else {
			date_set(Bit_RESET);
		}
		//printf("0x%x\r\n",xy & 0xff);
		xy <<= 1;
		ndelay(20);
	}
	stb_set(Bit_SET);
	if(on == 1) {
		date_set(Bit_SET);
	}
	else {
		date_set(Bit_RESET);
	}
	ndelay(20);
	stb_set(Bit_RESET);
	ndelay(20);
	date_set(Bit_RESET);
}
//select channel x and channel y connect
void ch446_SelectXY(const ch446_t *chip, int value)
{
	#ifdef CH446Q
	chip->bus->wreg(chip -> idx, value);
	#endif
}
//
void ch446_Reset(void)
{
	rst_set(Bit_SET);
	ndelay(100);
	rst_set(Bit_RESET);
}
//high logic is ok
void ch446_EnableStb(const ch446_t *chip)
{
	spi_cs_set(chip -> stb_pin, 1);
}
void ch446_DisableStb(const ch446_t *chip)
{
	spi_cs_set(chip -> stb_pin, 0);
}
void ch446_Write(const ch446_t *chip, int status, int value)
{
	ch446_SelectXY(chip, value);
	//ch446_SelectXY(chip, value);	
	ndelay(20);	
	if(status == 1) {
		ch446_SelectXY(chip, 0x80);
	}
	else {
		ch446_SelectXY(chip, 0x00);
	}
	ch446_EnableStb(chip);
	ndelay(100);
	ch446_DisableStb(chip);
	ndelay(100);
	//ch446_SelectXY(chip, 0x00);
}
const ch446_func_t ch446 = {
	.init = ch446_Init,
	.write = ch446_Switch,
	.reset = ch446_Reset,
};
#if 1
#include "shell/cmd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const ch446_t sr_ch446 = {
	.bus = &spi2,
	.idx = SPI_CS_DUMMY,
	.stb_pin = SPI_CS_PC3,
};
static int cmd_ch446_func(int argc, char *argv[])
{
	const char * usage = { \
		" usage:\r\n" \
		" ch446 init, chip init \r\n" \
		" ch446 reset \r\n" \
		" ch446 switch on yx \r\n" \
		" ch446 write on yx\r\n" \
		" .. \r\n"
	};
	if(argc == 2) {
		if(!strcmp(argv[1],"init")) {
			ch446_Init(&sr_ch446);
			printf("ch446 Init Successful!\r\n");
		}
		if(!strcmp(argv[1],"reset")) {
			ch446_Reset();
			printf("ch446 Reset Successful!\r\n");
		}
	}
	if(argc == 4) {
		int temp = 0;
		int temp1 = 0;
		sscanf(argv[2],"%d", &temp1);
		sscanf(argv[3],"%x", &temp);
		printf("status is %d\r\n",temp1);
		printf("Switch is 0x%x\r\n", temp);
		if(!strcmp(argv[1],"switch")) {
			if(temp1 != 0) {
				ch446_Switch(&sr_ch446, temp, 1);
			}
			else {
				ch446_Switch(&sr_ch446, temp, 0);
			}
		}
		if(!strcmp(argv[1],"write")) {
			if(temp1 != 0) {
				ch446_Write(&sr_ch446, 1, temp);
			}
			else {
				ch446_Write(&sr_ch446, 0, temp);
			}
		}
	}
	if(argc == 5) {
	}
	if(argc < 2) {
		printf(usage);
		return 0;
	}

	return 0;
}
const cmd_t cmd_ch446 = {"ch446", cmd_ch446_func, "ch446 cmd"};
DECLARE_SHELL_CMD(cmd_ch446)
#endif
