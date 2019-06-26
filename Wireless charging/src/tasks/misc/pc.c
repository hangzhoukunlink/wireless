/*******************************
 *jiamao.gu@2013 initial version
 *note:PB0 TIM3_CH3 PB1 TIM3_Ch4 
********************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <math.h>
#include "stm32f10x.h"
#include "shell/cmd.h"
#include "shell/shell.h"
#include "config.h"
#include "sys/task.h"
#include "ulp_time.h"
#include "pwm.h"
#include "dac.h"
#include "led.h"
#include "uart.h"
#include "sys/malloc.h"
#include "nvm.h"
#include "config.h"
#include "mbi5025.h"
#include "spi.h"
#include "ch446.h"
#include "lpt_ad5347.h"

#define d2mv(d) ((d) * 2500 / 4096)
#define mv2d(mv) ((mv) * 4096 / 2500)
#define OUTPUT 0
#define INPUT 1
#define TIM_pc TIM3
#define DMA_N 128   //每个通道的数据长度
#define DMA_C 2    //ADC通道数
#define OPA_ADC ( 3 - 1 ) //OPA's ADC is the third in the regular convert
#define N 10
#define M 2
#define fre_sample 5000
#define PI 3.1415926
#define NUM_WAVE 360
#define DAC_2 (volatile unsigned *)(DAC_BASE + (unsigned)0x00000014 + DAC_Align_12b_R)
#define INNERDAC 0
typedef struct {
	int max;
	int index;
} max_t;
typedef short q15_t;
typedef enum {
	SINE = 0,
	TRIANGLE,
	SQUARE,
	NONE,
	OTHER,
} type_e;
typedef struct {
	unsigned int value[8];
	float ratio[8];
	float delta_phase[8];
	int phase[8];
} sine_t;
typedef struct {
	unsigned int value[8];
	unsigned int num[8];
} square_t;
typedef struct {
	unsigned int value[8];
	unsigned int num[8];
	float delta[8];
} triangle_t;
typedef struct {
	unsigned int channel;
	unsigned int flag[8]; 
	unsigned int freq[8];
	sine_t sine;
	square_t square;
	triangle_t triangle;
	type_e type[8];
} option_t;
typedef enum {
	AUTOMATIC,
	MAMUAL = !AUTOMATIC,
} pc_shell_t;
pc_shell_t pc_shell = AUTOMATIC;
static signed int pc_cal_data[8][10] __nvm;
void adc_Init();
//2014.1.21 before business
static option_t dither_new = {
				0,
				0,0,0,0,0,0,0,0,
				1,1,1,1,1,1,1,1,
				{
					0,0,0,0,0,0,0,0,
					1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,
					0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
					0,0,0,0,0,0,0,0,
				},
				{
					0,0,0,0,0,0,0,0,
					0,0,0,0,0,0,0,0,
				},
				{
					0,0,0,0,0,0,0,0,
					0,0,0,0,0,0,0,0,
					0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
				},
				SINE,SINE,SINE,SINE,SINE,SINE,SINE,SINE,
				};
//0 ~ +1
static const q15_t SinTabQ15[NUM_WAVE] = {	
		0x4000, 0x411d, 0x423b, 0x4359, 0x4476, 0x4593, 0x46b0, 0x47cc, 
		0x48e8, 0x4a03, 0x4b1d, 0x4c36, 0x4d4e, 0x4e65, 0x4f7b, 0x5090, 
		0x51a4, 0x52b6, 0x53c6, 0x54d6, 0x55e3, 0x56ef, 0x57f9, 0x5901, 
		0x5a07, 0x5b0c, 0x5c0e, 0x5d0e, 0x5e0b, 0x5f07, 0x6000, 0x60f6, 
		0x61ea, 0x62db, 0x63c9, 0x64b5, 0x659e, 0x6684, 0x6766, 0x6846, 
		0x6923, 0x69fc, 0x6ad3, 0x6ba5, 0x6c75, 0x6d41, 0x6e09, 0x6ece, 
		0x6f8f, 0x704d, 0x7106, 0x71bc, 0x726e, 0x731c, 0x73c6, 0x746c, 
		0x750e, 0x75ac, 0x7646, 0x76db, 0x776c, 0x77f9, 0x7882, 0x7906, 
		0x7985, 0x7a00, 0x7a77, 0x7ae9, 0x7b56, 0x7bbf, 0x7c23, 0x7c83, 
		0x7cde, 0x7d34, 0x7d85, 0x7dd1, 0x7e19, 0x7e5c, 0x7e99, 0x7ed2, 
		0x7f07, 0x7f36, 0x7f60, 0x7f85, 0x7fa6, 0x7fc1, 0x7fd8, 0x7fe9, 
		0x7ff6, 0x7ffd, 0x7fff, 0x7ffd, 0x7ff6, 0x7fe9, 0x7fd8, 0x7fc1, 
		0x7fa6, 0x7f85, 0x7f60, 0x7f36, 0x7f07, 0x7ed2, 0x7e99, 0x7e5c, 
		0x7e19, 0x7dd1, 0x7d85, 0x7d34, 0x7cde, 0x7c83, 0x7c23, 0x7bbf, 
		0x7b56, 0x7ae9, 0x7a77, 0x7a00, 0x7985, 0x7906, 0x7882, 0x77f9, 
		0x776c, 0x76db, 0x7646, 0x75ac, 0x750e, 0x746c, 0x73c6, 0x731c, 
		0x726e, 0x71bc, 0x7106, 0x704d, 0x6f8f, 0x6ece, 0x6e09, 0x6d41, 
		0x6c75, 0x6ba5, 0x6ad3, 0x69fc, 0x6923, 0x6846, 0x6766, 0x6684, 
		0x659e, 0x64b5, 0x63c9, 0x62db, 0x61ea, 0x60f6, 0x6000, 0x5f07, 
		0x5e0b, 0x5d0e, 0x5c0e, 0x5b0c, 0x5a07, 0x5901, 0x57f9, 0x56ef, 
		0x55e3, 0x54d6, 0x53c6, 0x52b6, 0x51a4, 0x5090, 0x4f7b, 0x4e65, 
		0x4d4e, 0x4c36, 0x4b1d, 0x4a03, 0x48e8, 0x47cc, 0x46b0, 0x4593, 
		0x4476, 0x4359, 0x423b, 0x411d, 0x4000, 0x3ee2, 0x3dc4, 0x3ca6, 
		0x3b89, 0x3a6c, 0x394f, 0x3833, 0x3717, 0x35fc, 0x34e2, 0x33c9, 
		0x32b1, 0x319a, 0x3084, 0x2f6f, 0x2e5b, 0x2d49, 0x2c39, 0x2b29, 
		0x2a1c, 0x2910, 0x2806, 0x26fe, 0x25f8, 0x24f3, 0x23f1, 0x22f1, 
		0x21f4, 0x20f8, 0x2000, 0x1f09, 0x1e15, 0x1d24, 0x1c36, 0x1b4a, 
		0x1a61, 0x197b, 0x1899, 0x17b9, 0x16dc, 0x1603, 0x152c, 0x145a, 
		0x138a, 0x12be, 0x11f6, 0x1131, 0x1070, 0xfb2, 0xef9, 0xe43, 
		0xd91, 0xce3, 0xc39, 0xb93, 0xaf1, 0xa53, 0x9b9, 0x924, 
		0x893, 0x806, 0x77d, 0x6f9, 0x67a, 0x5ff, 0x588, 0x516, 
		0x4a9, 0x440, 0x3dc, 0x37c, 0x321, 0x2cb, 0x27a, 0x22e, 
		0x1e6, 0x1a3, 0x166, 0x12d, 0xf8, 0xc9, 0x9f, 0x7a, 
		0x59, 0x3e, 0x27, 0x16, 0x9, 0x2, 0x0, 0x2, 
		0x9, 0x16, 0x27, 0x3e, 0x59, 0x7a, 0x9f, 0xc9, 
		0xf8, 0x12d, 0x166, 0x1a3, 0x1e6, 0x22e, 0x27a, 
		0x2cb, 0x321, 0x37c, 0x3dc, 0x440, 0x4a9, 0x516, 0x588, 
		0x5ff, 0x67a, 0x6f9, 0x77d, 0x806, 0x893, 0x924, 0x9b9, 
		0xa53, 0xaf1, 0xb93, 0xc39, 0xce3, 0xd91, 0xe43, 0xef9, 
		0xfb2, 0x1070, 0x1131, 0x11f6, 0x12be, 0x138a, 0x145a, 
		0x152c, 0x1603, 0x16dc, 0x17b9, 0x1899, 0x197b, 0x1a61, 
		0x1b4a, 0x1c36, 0x1d24, 0x1e15, 0x1f09, 0x1fff, 0x20f8, 
		0x21f4, 0x22f1, 0x23f1, 0x24f3, 0x25f8, 0x26fe, 0x2806, 
		0x2910, 0x2a1c, 0x2b29, 0x2c39, 0x2d49, 0x2e5b, 0x2f6f, 
		0x3084, 0x319a, 0x32b1, 0x33c9, 0x34e2, 0x35fc, 0x3717, 
		0x3833, 0x394f, 0x3a6c, 0x3b89, 0x3ca6, 0x3dc4, 0x3ee2, 			
};
q15_t float_to_Q15(float a);
float Q15_to_float(q15_t b);
void arm_sin_q15(q15_t theta, q15_t* pSinVal);
static int phase = 0.0;
static float delta_phase = 0.0;
static float ratio_sin = 0.0;
static float delta_value = 0.0;
static int dither_mode = 0;
static int NUM = 0;
short ADC_ConvertedValue[DMA_N][DMA_C] = {0};
short ADC_ConvertedValue_temp[DMA_N] = {0};
short After_filter[DMA_C] = {0};
static int current_average __nvm = 0;
static int current_dither __nvm = 0;
int fre_pc = 1000, dc_pc = 80;
int dither = 1; //frequence of wave
float fre_dither = 0.0;
static unsigned int channel = 0; //ad5347 channel
//for eight channels
static unsigned int current_average_channel[8] = {0,0,0,0,0,0,0,0};
static int dither_freq[8] = {1,1,1,1,1,1,1,1};
static int option[8] = {0,0,0,0,0,0,0,0};
//sine
static unsigned int dither_sine[8] = {0,0,0,0,0,0,0,0}; //value
static float ratio_sine[8] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
static float delta_phase_sine[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
static int phase_sine[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//square
static unsigned int dither_square[8] = {0,0,0,0,0,0,0,0}; //value
static int NUM_square[8] = {1,1,1,1,1,1,1,1};
//triangle
static unsigned int dither_triangle[8] = {0,0,0,0,0,0,0,0}; //value
static int NUM_triangle[8] = {1,1,1,1,1,1,1,1};
static float delta_triangle[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

/****************************************************************
****************** static local function  ******************
****************************************************************/
static int pc_udelay(int us);
static int pc_mdelay(int ms);
//clock 0 tim1 ch1 PA8
void clock0_Init(int frq, int dc)
{
	pwm_cfg_t cfg;
	const pwm_bus_t *pwm = &pwm11;
	cfg.hz = frq;
	cfg.fs = 100;
	pwm -> init(&cfg);
	pwm -> set(dc);
}
//clock 1 tim3 ch3 PB0
void clock1_Init(int frq, int dc)
{
	pwm_cfg_t cfg;
	const pwm_bus_t *pwm = &pwm33;
	cfg.hz = frq;
	cfg.fs = 100;
	pwm -> init(&cfg);
	pwm -> set(dc);
}
//clock 2 tim4 ch3 PB3
void clock2_Init(int frq, int dc)
{
	pwm_cfg_t cfg;
	const pwm_bus_t *pwm = &pwm43;
	cfg.hz = frq;
	cfg.fs = 100;
	pwm -> init(&cfg);
	pwm -> set(dc);
}

//dds init : TIM2 as the clock source
void dds_Init(int frq)
{
	pwm_cfg_t cfg;
	const pwm_bus_t *pwm = &pwm2;
	cfg.hz = frq;
	cfg.fs = 100;
	pwm->init(&cfg);
}

//dac2 : PA5
void dac2_Init(int option)
{
	dac_ch2.init(NULL);
}

void adc_Init(void)
{
	/*ADC1/2/3 channel0 PA0 ADC1/2/3 channel1 PA1*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); /*72Mhz/8 = 9Mhz*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//DMA Config
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x4001244C; //ADC1的DR地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_ConvertedValue; //不加&应该也可以 数组名 代表地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//外设向内存传数据
	DMA_InitStructure.DMA_BufferSize = DMA_C * DMA_N; //DMA缓存大小 //2 * 128
	DMA_InitStructure.DMA_PeripheralInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //目标内存地址自动后移
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//DMA传送数据尺寸16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//常用循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//禁止内存到内存模式
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, DISABLE);
	DMA_Init(DMA1_Channel1, &DMA_InitStructure); 
	//ADC Config 
	ADC_InitTypeDef ADC_InitStructure;
	ADC_DeInit(ADC1);
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//!!
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//!!
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = DMA_C; //2
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5); //9Mhz/(239.5 + 12.5) = 35.7Khz
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5); //9Mhz/(239.5 + 12.5) = 35.7Khz
	//ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
}

#define A1_PIN	GPIO_Pin_0
#define A1_BANK GPIOC
#define A1_CLOCK	RCC_APB2Periph_GPIOC	
#define B1_PIN	GPIO_Pin_1
#define B1_BANK GPIOC
#define B1_CLOCK	RCC_APB2Periph_GPIOC
#define A2_PIN	GPIO_Pin_10
#define A2_BANK GPIOD
#define A2_CLOCK	RCC_APB2Periph_GPIOD
#define B2_PIN	GPIO_Pin_11
#define B2_BANK GPIOD
#define B2_CLOCK	RCC_APB2Periph_GPIOD
#define INH1_PIN	GPIO_Pin_6
#define INH1_BANK GPIOE
#define INH1_CLOCK	RCC_APB2Periph_GPIOE
#define INH2_PIN	GPIO_Pin_12
#define INH2_BANK GPIOA	
#define INH2_CLOCK	RCC_APB2Periph_GPIOA
void cd4052_Init(void)
{
	RCC_APB2PeriphClockCmd(A1_CLOCK | B1_CLOCK | A2_CLOCK | B2_CLOCK | INH1_CLOCK | INH2_CLOCK, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = A1_PIN | B1_PIN;    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(A1_BANK, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = A2_PIN | B2_PIN;    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(A2_BANK, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = INH1_PIN;    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(INH1_BANK, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = INH2_PIN;    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(INH2_BANK, &GPIO_InitStructure);
}

#define A1_SET(level)	(GPIO_WriteBit(A1_BANK, A1_PIN, level))
#define A2_SET(level)	(GPIO_WriteBit(A2_BANK, A2_PIN, level))
#define B1_SET(level)	(GPIO_WriteBit(B1_BANK, B1_PIN, level))
#define B2_SET(level)	(GPIO_WriteBit(B2_BANK, B2_PIN, level))
#define INH1_SET(level)	(GPIO_WriteBit(INH1_BANK, INH1_PIN, level))
#define INH2_SET(level)	(GPIO_WriteBit(INH2_BANK, INH2_PIN, level))
//LOW ENABLE
//HIGH DISABLE 
int cd4052_channel(int ch_cd4052)
{
	if(ch_cd4052 < 4) {
		INH1_SET(Bit_RESET);
		INH2_SET(Bit_SET);
		switch(ch_cd4052) {
			case 0 :
				A1_SET(Bit_RESET);
				B1_SET(Bit_RESET);
				break;
			case 1 :
				A1_SET(Bit_SET);
				B1_SET(Bit_RESET);
				break;
			case 2 :
				A1_SET(Bit_RESET);
				B1_SET(Bit_SET);
				break;
			case 3 :
				A1_SET(Bit_SET);
				B1_SET(Bit_SET);
				break;
			default : 
				return -1;
		}			
	} else if(ch_cd4052 < 8) {
		INH2_SET(Bit_RESET);
		INH1_SET(Bit_SET);
		switch(ch_cd4052) {
			case 4 :
				A2_SET(Bit_RESET);
				B2_SET(Bit_RESET);
				break;
			case 5 :
				A2_SET(Bit_SET);
				B2_SET(Bit_RESET);
				break;
			case 6 :
				A2_SET(Bit_RESET);
				B2_SET(Bit_SET);
				break;
			case 7 :
				A2_SET(Bit_SET);
				B2_SET(Bit_SET);
				break;
			default : 
				return -1;
		}			
	} else {
		return -1;
	}
	return 0;
}
//flash the eight green led and red led alternately
void led_init(void) 
{	
	for(int i = 0; i < 5; i++) {
		for(int i = 0; i < 8; i++) {
			led_on((led_t)(LED_GREEN1 + 3 * i));
			led_off((led_t)(LED_RED1 + 3 * i));
		}
		pc_mdelay(500);
		for(int i = 0; i < 8; i++) {
			led_off((led_t)(LED_GREEN1 + 3 * i));
			led_on((led_t)(LED_RED1 + 3 * i));
		}
		pc_mdelay(500);
	}
}
void pc_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//占先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;           //响应级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //启动
	NVIC_Init(&NVIC_InitStructure);  	//初始化
	dac2_Init(0);
	adc_Init();
	cd4052_Init();
	shell_mute((const struct console_s *) &uart1);
	//dac_ch2.write((300 * 4095 / 2500));	
	ch446.init();
	ad5347.init(OUTPUT);
	//for test	
	clock1_Init(3000,10);
	//ch446.set(0,1,1);
	//ch446.set(1,1,1);
	//ad5347.write(500,0);
	//ad5347.write(500,1);
	for(int i = 0; i < 4; i++) {
		for(int j = 0; j < 8; j++) {
			if((i % 2))
				led_on((led_t)(LED_GREEN1 + 3 * j));
			else 
				led_on((led_t)(LED_RED1 + 3 * j));
		}
		pc_mdelay(300);
		for(int j = 0; j < 8; j++) {
			if((i % 2))
				led_off((led_t)(LED_GREEN1 + 3 * j));
			else 
				led_off((led_t)(LED_RED1 + 3 * j));
		}
		pc_mdelay(300);
	}
}

/*get the average*/
//DMA_C 2
//DMA_N 128
void filter(void)
{
	short sum = 0;
	for(int i = 0; i < DMA_C; i++) {
		for(int j = 0; j < DMA_N; j++) {
			sum += ADC_ConvertedValue[j][i];
		}
		After_filter[i] = sum / DMA_N;
    }
}
//find the max of a
max_t find_max(int *a)
{
	max_t max = {a[0],0};
	for(int i = 1; i < DMA_N; i++) {
		if(max.max < a[i]) {
			max.max = a[i];
			max.index = i;
		}
	}
	return max;
}
//find the second max of a
max_t find_second_max(int *a)
{
	int max = a[0];
	max_t second = {a[1],1};
	for(int i = 1; i < DMA_N; i++) {
		if(max < a[i]) {
			second.max = max;	//update the max and second max value
			max = a[i];
			second.index = i;
		} else if((a[i] < max) && (a[i] > second.max)) {
			second.max = a[i];
			second.index = i;
		}
	}
	return second;
}
//delete the b from array a
void array_rebuild(int *a, max_t b)
{
	a[b.index] = 0;
}
int* adc(int ch)
{
	int adc_value[2][DMA_N] = {0};
	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	pc_mdelay(10);
	cd4052_channel(ch);
	pc_mdelay(10);
	//1 1 1 1 1 1 1 
	//2 2 2 2 2 2 2
	for(int j = 0; j < DMA_C; j++) {
		for(int k = 0;k < DMA_N; k++) {
			adc_value[j][k] = ADC_ConvertedValue[k][j] * 2500 / 4095;
		}
		//adc_value[j] = adc_value[j] / DMA_N;	//average
	}
	return (ch < 4) ? adc_value[0] : adc_value[1];
}
void pc_Updata(void)
{
	max_t first = {0,0};
	max_t second = {0,0};
	int try = 5;
	for(int i = 0; i < 8; i++) {
		if(((dither_new.flag[i] == 1) && ((dither_new.sine.value[i] > 0) || (dither_new.square.value[i] > 0) || (dither_new.triangle.value[i] > 0))) || (current_average_channel[i] > 0)) { //mean x channel exist 
			/*switch(dither_new.type[i]) {
				case SINE:
				first = find_max(adc(i));
				second = find_second_max(adc(i));
					do {
						if(first.max > second.max * 2) {
							array_rebuild(adc(i),first);
							first = find_max(adc(i));
							second = find_second_max(adc(i));
						} else {
							break;
						}
					} while(try > 0);
					if((first.max < (dither_new.sine.value[i] + current_average_channel[i]) * 11 / 10) && (first.max > (dither_new.sine.value[i] + current_average_channel[i]) * 9 / 10))
						led_on((led_t)(LED_GREEN1 + 3 * i));
					else 
						led_on((led_t)(LED_RED1 + 3 * i));
					break;
				case SQUARE:
					break;
				case TRIANGLE:
					break;
			}*/	
			led_on((led_t)(LED_GREEN1 + 3 * i));
		} else {
			led_off((led_t)(LED_GREEN1 + 3 * i));
		}
	}
}
int main(void)
{
	task_Init();
	pc_Init();
	while(1){
		task_Update();
		pc_Updata();
	} 
}

q15_t float_to_Q15(float a)
{
	//assert(a <= 1);
	short int b = 0;
	b = (short)(a * (1 << 15));
	return b;
}
float Q15_to_float(q15_t b)
{
	float a = 0.0;
	a = (float) b / (1 << 15);
	return a;
}
void arm_sin_q15(q15_t theta, q15_t* pSinVal)
{
	q15_t x0;
	q15_t y0,y1;
	q15_t xSpacing = 0xB6;
	unsigned int i = 0;
	q15_t oneByXSpacing;
	q15_t out;
	unsigned int sign_bits;
	short firstX = 0x8000;
	i = (unsigned int)(theta - firstX) / (unsigned int)xSpacing;
	/* Checking min and max index of table */
	if(i < 0) {
		i = 0;
	}
	else if(i >= 358) {
		i = 358;
	}
	x0 = (q15_t)firstX + ((q15_t)i * xSpacing);
	y0 = SinTabQ15[i];
	y1 = SinTabQ15[i + 1u];
	sign_bits = 8u; //?? 180 / 256
	oneByXSpacing = 0x5A00;//??
	/* Calculation of (theta - x0)/(x1-x0) */
	out =
		(((q15_t) (((int) (theta - x0) * oneByXSpacing) >> 16)) << sign_bits);
	/* Calculation of y0 + (y1 - y0) * ((theta - x0)/(x1-x0)) */
	*pSinVal = y0 + ((q15_t) (((int) (y1 - y0) * out) >> 15));
}
/****************************************************************
****************** static local function  ******************
****************************************************************/
static int pc_udelay(int us)
{
    u16 TIMCounter = 101 - us;
    TIM_Cmd(TIM2, ENABLE);
    TIM_SetCounter(TIM2, TIMCounter);
    while (TIMCounter < 100){
		TIMCounter = TIM_GetCounter(TIM2);
    }
    TIM_Cmd(TIM2, DISABLE);
    return 0;
}

static int pc_mdelay(int ms)
{
    int left;
	time_t deadline = time_get(ms);
	do {
		left = time_left(deadline);
		if(left >= 10) { //system update period is expected less than 10ms
			ulp_update();
		}
	} while(left > 0);
	return 0;
}

#define dac_write(data,channel) (ad5347.write(data,channel))
void TIM2_IRQHandler(void)
{
	static q15_t sin[8] = {0,0,0,0,0,0,0,0};
	static int tri[8] = {0,0,0,0,0,0,0,0};
	static int __NUM_INTERRUPT = 0;
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) {       //判断中断来源
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);            //清除中断标志
		for(int i = 0; i < 8; i++) {
			unsigned int temp = 0;
			if(dither_new.flag[i]) {
				switch(dither_new.type[i]) {
				case SINE:
					dither_new.sine.phase[i] += dither_new.sine.delta_phase[i];
					if(dither_new.sine.phase[i] == 0x7fffffff) {
						dither_new.sine.phase[i] = 0;
					}
					//printf("theta : 0x%x\r\n",theta & 0xffff);
					arm_sin_q15(dither_new.sine.phase[i],&sin[i]);
					//printf("%f\r\n",Q15_to_float(sin));
					//printf("0x%x\r\n",sin);
					temp = (int)(Q15_to_float(sin[i]) * dither_new.sine.ratio[i] * 4095 / 1.8) + current_average_channel[i];
					if(temp == 0) 
						dac_write(0,i);
					else
						dac_write(temp + pc_cal_data[i][temp >> 8],i);
					break;
				case SQUARE:
					if(__NUM_INTERRUPT < dither_new.square.num[i] / 2) {
						temp = current_average_channel[i];
						if(temp ==0)
							dac_write(0,i);
						else 
							dac_write(temp + pc_cal_data[i][temp >> 8],i);
					} else if (__NUM_INTERRUPT < dither_new.square.num[i]) {
						temp = (int)dither_new.square.value[i] + current_average_channel[i];
						if(temp == 0)
							dac_write(0,i);
						else
							dac_write(temp + pc_cal_data[i][temp >> 8],i);
					} else {
						__NUM_INTERRUPT = 0;
					}
					break;
				case TRIANGLE:
					if(__NUM_INTERRUPT < dither_new.triangle.num[i] / 2) {
						tri[i] = (int)(dither_new.triangle.delta[i] * __NUM_INTERRUPT);
						//printf("%d\r\n",tri);				
					} else if(__NUM_INTERRUPT < dither_new.triangle.num[i]) {
						tri[i] = (int)(dither_new.triangle.delta[i] * (dither_new.triangle.num[i] - __NUM_INTERRUPT));
						//printf("%d\r\n",tri);
					} else {
						__NUM_INTERRUPT = 0;
					}
					temp = (int)(tri[i] + current_average_channel[i]);
					if(temp == 0)
						dac_write(0,i);
					else
						dac_write(temp + pc_cal_data[i][temp >> 8],i);
					break;
				/*case NONE:
					temp = (int)current_average_channel[i];
					if(temp == 0)
						dac_write(0,i);
					else 
						dac_write(temp + pc_cal_data[i][temp >> 8],i);
					break;*/
				default:
					break;
				}
			}
		}
	}
	__NUM_INTERRUPT ++;
}
#if 1
#if defined ( __CC_ARM   ) /*------------------RealView Compiler -----------------*/ 
__asm void GenerateSystemReset(void) 
{ 
	MOV R0, #1           //;  
	MSR FAULTMASK, R0    //; FAULTMASK 禁止一切中断产生 
	LDR R0, =0xE000ED0C  //; 
	LDR R1, =0x05FA0004  //;  
	STR R1, [R0]         //;    
	deadloop 
	B deadloop        //;  
} 
#elif (defined (__ICCARM__)) /*------------------ ICC Compiler -------------------*/ 
//#pragma diag_suppress=Pe940
void GenerateSystemReset(void) 
{ 
	__ASM("MOV R0, #1"); 
	__ASM("MSR FAULTMASK, R0"); 
	SCB -> AIRCR = 0x05FA0004; 
	for(;;); 
}
#endif
typedef void (*reset)(void);
static int cmd_pc_func(int argc, char *argv[])
{
	const char *usage = {
		" pc usage : \r\n"
		" \033[31;31mself test : \033[0m\r\n"
		" pc reset \r\n"
		" pc adc\r\n"
		" pc adc all\r\n"
		" pc adc ch\r\n"
		" pc calc i x0 x1 x2 x3 x4 x5 x6 x7\r\n"
		" pc set cd4052 channel \r\n"
		" pc test max \r\n"
		" pc -m : operate manual\r\n"
		" pc -a : operate automatic\r\n"
		" pc set clk0 3000 50 argc = 5\r\n"
		" pc set clk1 3000 50 argc = 5\r\n"
		" pc set clk2 3000 50 argc = 5\r\n"
		" pc set ad5347 1000 0\r\n"
		" pc set current 100: 100ma\r\n"
		" pc set current 100 channel\r\n"
		" pc set dither sine freq current : dds example argc = 6\r\n"
		" pc set dither sine freq current channel\r\n"
		" pc set dither square freq current : square example argc = 6\r\n" 
		" pc set dither square freq current channel\r\n"
		" pc set dither triangle freq current : triangle example argc = 6\r\n"	
		" pc set dither triangle freq current channel\r\n"	
		" pc reset channel 0~7 : reset channel XX current and dither\r\n"		
		" pc current display\r\n"
		" \033[31;31mtest with CVI : \033[0m \r\n" 
		" pc set clk0 clk1 clk2 a1 a2 b1 b2 c1 c2 : argc = 11\r\n"
		" pc set clock x channel y : argc = 6\r\n"
		" pc reset clock x channel y\r\n"
		" 		x < 3, y < 8\r\n"		
		" pc set current freq value : argc = 5\r\n"
		" pc set dither freq value type : argc = 6\r\n"
		"		type 0 : sine wave\r\n"
		"		type 1 : triangle wave\r\n"
		"		type 2 : square wave\r\n"
		"		type 4 : other wave\r\n"
		" ... \r\n"
	};
	if(argc < 2){
		printf("%s", usage);
		return 0;
	}
	if(argc == 2) {
		if(!strcmp(argv[1], "adc")) {
			float adc_value = 0.0;
			DMA_Cmd(DMA1_Channel1, ENABLE);
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
			pc_mdelay(10);
			for(int i = 0; i < DMA_C; i++) {
				for(int j = 0;j < DMA_N; j++) {
					adc_value += ADC_ConvertedValue[j][i] * 2500.0 / 4095;
				}
				adc_value = adc_value / DMA_N;
				printf("%.1f mv\r\n", adc_value);
			}
		}
		if(!strcmp(argv[1], "reset")) {
			GenerateSystemReset();
			ad5347.clear();
			ch446.reset();
		}
		if(!strcmp(argv[1],"-m")) {
			shell_unmute((const struct console_s *) &uart1);
			pc_shell = MAMUAL;
		}
		if(!strcmp(argv[1],"-a")) {
			shell_mute((const struct console_s *) &uart1);
			pc_shell = AUTOMATIC;
		}
	}
	if(argc == 3){
		if(!strcmp(argv[1],"test") && !strcmp(argv[2],"max")) {
			int a[DMA_N] = {2,5,6,8,9,23,19};
			printf("max data is %d %d\r\n",find_max(a).max, find_max(a).index);
			printf("second max data is %d %d\r\n",find_second_max(a).max, find_second_max(a).index);
		}
		if(!strcmp(argv[1],"adc") && !strcmp(argv[2],"all")) {
			float adc_value[2] = {0.0, 0.0};
			pc_mdelay(10);
			for(int i = 0; i < 8; i++) {
				cd4052_channel(i);
				pc_mdelay(10);
				DMA_Cmd(DMA1_Channel1, ENABLE);
				ADC_SoftwareStartConvCmd(ADC1, ENABLE);				
				pc_mdelay(10);
				for(int j = 0; j < DMA_C; j++) {
					for(int k = 0;k < DMA_N; k++) {
						adc_value[j] += ADC_ConvertedValue[k][j] * 2500.0 / 4095;
					}
					adc_value[j] = adc_value[j] / DMA_N;					
				}
				if(i < 4)
					printf("\r\nchannel %d voltage is %.1f mv\r\n", i, adc_value[0]);
				else 
					printf("\r\nchannel %d voltage is %.1f mv\r\n", i, adc_value[1]);
				DMA_Cmd(DMA1_Channel1, DISABLE);
				ADC_SoftwareStartConvCmd(ADC1, DISABLE);			
			}
		}
		if(!strcmp(argv[1],"current") && (!strcmp(argv[2],"dispaly"))) {
			int temp = 0;
			for(int i = 0; i < DMA_N; i++) {
				temp += ADC_ConvertedValue_temp[i];
				printf("%d\t",d2mv(ADC_ConvertedValue_temp[i]));
			}
			temp >>= 10;
			printf("Average is %d\r\n",temp);	
		}
	}
	if(argc == 4){
		if(!strcmp(argv[1],"test") && !strcmp(argv[2],"cd4052")) {
			int ch = 0;
			sscanf(argv[3],"%d",&ch);
			printf("Channel is %d \r\n", ch);
			cd4052_channel(ch);
			if(cd4052_channel(ch) == -1) {
				printf("Oversize 0~8 \r\n");
			} else {
				printf("OK!\r\n");
			}
		}
		if(!strcmp(argv[1],"set") && !strcmp(argv[2],"green")) {
			if(!strcmp(argv[3],"on")) {
				for(int i = 0; i < 8; i++) {
					led_on((led_t)(LED_GREEN1 + 3 * i));
					//led_on((led_t)(LED_RED1 + 3 * i));
				}
			}
			if(!strcmp(argv[3],"off")) {
				for(int i = 0; i < 8; i++) {
					led_off((led_t)(LED_GREEN1 + 3 * i));
					//led_off((led_t)(LED_RED1 + 3 * i));
				}
			}
		}
		if(!strcmp(argv[1],"set") && !strcmp(argv[2],"red")) {
			if(!strcmp(argv[3],"on")) {
				for(int i = 0; i < 8; i++) {
					//led_on((led_t)(LED_GREEN1 + 3 * i));
					led_on((led_t)(LED_RED1 + 3 * i));
				}
			}
			if(!strcmp(argv[3],"off")) {
				for(int i = 0; i < 8; i++) {
					//led_off((led_t)(LED_GREEN1 + 3 * i));
					led_off((led_t)(LED_RED1 + 3 * i));
				}
			}
		}
		if(!strcmp(argv[2],"current")) {
			sscanf(argv[3],"%d",&current_average);
			if(current_average > 2000) {
				printf("Wrong current value, must lower than 2000ma!\r\n");
			}
			else {
				int voltage_temp =(int)(0.1 * current_average * 8);
				printf("%dmv\r\n",voltage_temp);
				int voltage_digital = mv2d(voltage_temp);
				printf("0x%x\r\n",voltage_digital);
				dac_ch2.write(voltage_digital);
			}				
		}
	}
	if(argc == 5){ 
		if(!strcmp(argv[2],"ad5347")) {
			sscanf(argv[3],"%d",&current_average);
			sscanf(argv[4],"%d",&channel);
			ad5347.write(current_average, channel);
			printf("OK\r\n");
		}	
		if(!strcmp(argv[2],"clk0")){
			sscanf(argv[3],"%d",&fre_pc);
			sscanf(argv[4],"%d",&dc_pc);
			clock0_Init(fre_pc,dc_pc);
			if(MAMUAL == pc_shell)
				printf("Clk0 init OK!\r\n");
			printf("OK\r\n");
		}
		if(!strcmp(argv[2],"clk1")){
			sscanf(argv[3],"%d",&fre_pc);
			sscanf(argv[4],"%d",&dc_pc);
			clock1_Init(fre_pc,dc_pc);
			if(MAMUAL == pc_shell)
				printf("Clk1 init OK!\r\n");
			printf("OK\r\n");
		}
		if(!strcmp(argv[2],"clk2")){
			sscanf(argv[3],"%d",&fre_pc);
			sscanf(argv[4],"%d",&dc_pc);
			clock2_Init(fre_pc,dc_pc);
			if(MAMUAL == pc_shell)
				printf("Clk2 init OK!\r\n");
			printf("OK\r\n");
		}
		if(!strcmp(argv[2],"current")) {
			printf("OK\r\n");
			sscanf(argv[4],"%d",&channel);
			if(channel < 8) {
				sscanf(argv[3],"%d",&current_average_channel[channel]);
				//current_average_channel[channel] = current_average_channel[channel] * 4 / 5;
				//dither_new.type[channel] = NONE;
				//dither_new.flag[channel] = 1;
				ch446.set(channel,1,1);
				if(MAMUAL == pc_shell)
					//printf("current_average_channel[%d] is %d\r\n",channel, current_average_channel[channel]);
				if(current_average_channel[channel] < 2000) {
					int temp = (int)current_average_channel[channel];
					int templ = temp % 256;
					int tempr = 256 - templ;
					int delta = (pc_cal_data[channel][temp >> 8] * tempr >> 8) + (pc_cal_data[channel][temp >> 8 + 1] * templ >> 8);
					if((temp + delta) <= 0)
						ad5347.write(0,channel);
					else {
						printf("%d\r\n",(temp + delta) * 4 / 5);
						ad5347.write((temp + delta) * 4 / 5,channel);
					}
				} else {
					if(MAMUAL == pc_shell) {
						printf("Current is oversize\r\n");
						printf("Please 重新输入！！\r\n");
					}
				}
				dds_Init(fre_sample);
			} else {
				if(MAMUAL == pc_shell) {
					printf("Please 重新输入！！\r\n");
				}
			}
		}
	}
	//pc set clock x channel x
	int clock = 0;
	int channel = 0;
	if(argc == 6){
		if(!strcmp(argv[3],"square")) {
			sscanf(argv[4],"%d",&dither);
			sscanf(argv[5],"%d",&current_dither);
			dither_mode = 2; // 2 : square
			//int digital = mv2d(current * 33 * 20 / 1000);
			//printf("0x%x\r\n",digital);
			NUM = fre_sample / dither ;
			printf("%d\r\n",NUM);
			dds_Init(fre_sample);
			printf("Operation Successfully\r\n");
		}
		if(!strcmp(argv[3],"sine")) {
			sscanf(argv[4],"%d",&dither);
			sscanf(argv[5],"%d",&current_dither);
			printf("dither is : %d\r\n",dither);
			dither_mode = 0; //0 sine
			ratio_sin = 1.0 * current_dither / 2500;
			float delta_phase_temp = 2.03 * dither / fre_sample ;
			printf("delta_phase_temp is : %f\r\n",delta_phase_temp);
			delta_phase = float_to_Q15(delta_phase_temp);			
			dds_Init(fre_sample);
		}
		if(!strcmp(argv[3],"triangle")) {
			sscanf(argv[4],"%d",&dither);
			sscanf(argv[5],"%d",&current_dither);
			printf("dither is : %d\r\n",dither);
			dither_mode = 1;//1 triangle
			NUM = fre_sample / dither;
			printf("num of wave is : %d\r\n", NUM);
			float delta_value_temp = 2.0 * current_dither / NUM;
			delta_value = delta_value_temp;
			dds_Init(fre_sample);
		}
		if((!strcmp(argv[2],"clock"))&&(!strcmp(argv[4],"channel"))) {
			sscanf(argv[3],"%d",&clock);
			sscanf(argv[5],"%d",&channel);
			if((clock < 3) &&(channel < 8)) {
				char temp = (channel << 4 | clock);
				ch446.write(temp, 1);
				if(0 == clock) {
					ch446.write(temp + 1, 0);
					ch446.write(temp + 2, 0);
				} else if(1 == clock) {
					ch446.write(temp + 1, 0);
					ch446.write(temp - 1, 0);
				} else if(2 == clock) {
					ch446.write(temp - 2, 0);
					ch446.write(temp - 1, 0);
				}
			} else {
				printf("Clock or Channel oversize max value\r\n");
				printf("Please 重新输入\r\n");
			}
		}
	}
	float delta_phase_temp[8];
	if(argc == 7) {
		if(!strcmp(argv[3],"sine")) {
			printf("OK\r\n");
			sscanf(argv[6],"%d",&dither_new.channel);
			ch446.set(dither_new.channel,1,1);
			if(dither_new.channel < 8) {
				sscanf(argv[4],"%d",&dither_new.freq[dither_new.channel]);
				sscanf(argv[5],"%d",&dither_new.sine.value[dither_new.channel]);
				//dither_new.sine.value[dither_new.channel] = dither_new.sine.value[dither_new.channel] * 4 / 5;
				if(MAMUAL == pc_shell)
					printf("dither_freq[%d] is %d\r\n",dither_new.channel,dither_new.freq[dither_new.channel]);
				dither_new.type[dither_new.channel] = SINE; //0 sine				
				dither_new.sine.ratio[dither_new.channel] = 1.0 * dither_new.sine.value[dither_new.channel] / 2500 ;
				delta_phase_temp[dither_new.channel] = 2.03 * dither_new.freq[dither_new.channel] / fre_sample ;
				if(MAMUAL == pc_shell)	
					printf("delta_phase_temp[%d] is %f\r\n",dither_new.channel,delta_phase_temp[dither_new.channel]);
				dither_new.sine.delta_phase[dither_new.channel] = float_to_Q15(delta_phase_temp[dither_new.channel]);
				dither_new.flag[dither_new.channel] = 1;
				dds_Init(fre_sample);
			} else {
				if(MAMUAL == pc_shell) {
					printf("channel is oversize 7\r\n");
					printf("please 重新输入！！\r\n");
				}
			}
		}
		if(!strcmp(argv[3],"square")) {
			printf("OK\r\n");
			sscanf(argv[6],"%d",&dither_new.channel);
			ch446.set(dither_new.channel,1,1);
			if(dither_new.channel < 8) {
				sscanf(argv[4],"%d",&dither_new.freq[dither_new.channel]);
				sscanf(argv[5],"%d",&dither_new.square.value[dither_new.channel]);
				dither_new.square.value[dither_new.channel] = dither_new.square.value[dither_new.channel] * 4 / 5;
				if(MAMUAL == pc_shell)	
					printf("ditehr_freq[%d] is %d\r\n",dither_new.channel, dither_new.freq[dither_new.channel]);
				dither_new.square.num[dither_new.channel] = fre_sample / dither_new.freq[dither_new.channel];
				if(MAMUAL == pc_shell)	
					printf("NUM_square[%d] is %d\r\n",dither_new.channel,dither_new.square.num[dither_new.channel]);
				dither_new.type[dither_new.channel] = SQUARE;
				dither_new.flag[dither_new.channel] = 1;
				dds_Init(fre_sample);
			} else {
				if(MAMUAL == pc_shell) {
					printf("channel is oversize 7\r\n");
					printf("please 重新输入！！\r\n");
				}
			}
		}
		if(!strcmp(argv[3],"triangle")) {
			printf("OK\r\n");
			sscanf(argv[6],"%d",&dither_new.channel);
			ch446.set(dither_new.channel,1,1);
			if(dither_new.channel < 8) {
				sscanf(argv[4],"%d",&dither_new.freq[dither_new.channel]);
				sscanf(argv[5],"%d",&dither_new.triangle.value[dither_new.channel]) ;
				dither_new.triangle.value[dither_new.channel] = dither_new.triangle.value[dither_new.channel] * 4 / 5;
				if(MAMUAL == pc_shell)
					printf("dither_freq[%d] is %d\r\n",dither_new.channel, dither_new.freq[dither_new.channel]);
				dither_new.triangle.num[dither_new.channel] = fre_sample / dither_new.freq[dither_new.channel];
				if(MAMUAL == pc_shell)	
					printf("NUM_triangle[%d] is %d\r\n",dither_new.channel,dither_new.triangle.num[dither_new.channel]);
				dither_new.type[dither_new.channel] = TRIANGLE;
				dither_new.flag[dither_new.channel] = 1;
				float delta_value_temp = 2.0 * dither_new.triangle.value[dither_new.channel] / dither_new.triangle.num[dither_new.channel]; //step size
				dither_new.triangle.delta[dither_new.channel] = delta_value_temp;
				dds_Init(fre_sample);
			} else {
				if(MAMUAL == pc_shell) {
					printf("channel is oversize 7\r\n");
					printf("please 重新输入！！\r\n");
				}
			}
		}
	}
	if(argc == 11) {
		if((!strcmp(argv[2],"clk0"))&&(!strcmp(argv[3],"clk1"))&&(!strcmp(argv[4],"clk2"))) {
			printf("OK\r\n");
			int clk0 = 0, clk1 = 0, clk2 = 0;
			int dc0 = 0, dc1 = 0, dc2 = 0;
			sscanf(argv[5],"%d",&clk0);
			sscanf(argv[6],"%d",&dc0);
			sscanf(argv[7],"%d",&clk1);
			sscanf(argv[8],"%d",&dc1);
			sscanf(argv[9],"%d",&clk2);
			sscanf(argv[10],"%d",&dc2);
			clock0_Init(clk0,dc0);
			if(MAMUAL == pc_shell)
				printf("clk0 init successful!");
			clock1_Init(clk1,dc1);
			if(MAMUAL == pc_shell)
				printf("clk1 init successful!");
			clock2_Init(clk2,dc2);
			if(MAMUAL == pc_shell)
				printf("clk2 init successful!\n");
		}
	}
	return 0;
}

cmd_t cmd_pc = {"pc", cmd_pc_func, "commands for pc download"};
DECLARE_SHELL_CMD(cmd_pc)

static int cmd_cal_func(int argc, char *argv[])
{
	const char *usage = {
		"CAL <ch> v0 v1 v2 ...	ch = 0..7\n"
		"CAL print <ch> \n"
	};
	
	int ecode = -1;
	if(argc == 3) {
		if(!strcmp(argv[1],"print")) {
			int ch = atoi(argv[2]);
			ch = (ch > 8) ? 8 : ch;
			for(int i = 0; i < 10; i++) {
				printf("%02d ",pc_cal_data[ch][i]);
			}
			printf("\r\n");
			ecode = 0;
		}
	}
	if((argc > 2) && (strcmp(argv[1],"print"))) {
		int ch = atoi(argv[1]);
		if((ch >= 0) && (ch < 8)) {
			int n = argc - 2;
			n = (n > 8) ? 8 : n;
			for(int i = 0; i < n; i ++) {
				pc_cal_data[ch][i] = ((signed char)atoi(argv[i + 2]));
			}
			nvm_save();
			ecode = 0;
			for(int i = 0; i < 8; i++) {
				printf("%02d ",pc_cal_data[ch][i]);
			}
			printf("\r\n");
		}
	}
	if(ecode) {
		printf("%s",usage);
	}
	return 0;
}
cmd_t cmd_cal = {"cal", cmd_cal_func, "valve calibration"};
DECLARE_SHELL_CMD(cmd_cal)
#endif