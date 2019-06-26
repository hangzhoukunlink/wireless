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
#include "led.h"
#include "uart.h"
#include "sys/malloc.h"
#include "config.h"
#include "arm_math.h"
#include "my_key.h"
#include "stm32_dsp.h"
#include "table_fft.h"

#define d2mv(d) ((d) * 2500 / 4096)
#define mv2d(mv) ((mv) * 4096 / 2500)
#define OUTPUT 0
#define INPUT 1
#define TIM_pc TIM3
#define DMA_N 1024   //每个通道的数据长度
#define DMA_C 1    //ADC通道数
#define OPA_ADC ( 3 - 1 ) //OPA's ADC is the third in the regular convert
#define N 10
#define M 2
#define fre_sample 5000
//#define PI 3.1415926
#define PI2 6.2831853
#define NUM_FFT 1024
#define NUM_WAVE 360
#define NPT 1024            /* NPT = No of FFT point*/
void adc_Init();
void my_sine(uint32_t W_freq, q15_t *W_buf, uint32_t W_ampli);
short ADC_ConvertedValue[DMA_N][DMA_C] = {0};	//
short ADC_ConvertedValue_temp[DMA_N] = {0};
short After_filter[DMA_C] = {0};

arm_cfft_radix4_instance_q15 S;
q15_t wBuf_in[NUM_FFT];  //for adc
q15_t wTest_input[NUM_FFT * 2];
q15_t wTest_output[NUM_FFT];

float fTest_input[NUM_FFT * 2];
float fTest_output[NUM_FFT];

long lBUFIN[NPT];         /* Complex input vector */
long lBUFOUT[NPT];        /* Complex output vector */
long lBUFMAG[NPT + NPT/2];/* Magnitude vector */


static uint8_t s_cFlag_10ms_interrup = 0;
void scan_init(int frq);
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
//self clk set for high freq(duty is 50%)
//freq must > 50000
//PA6
static void time3_ch1_Init(int freq)
{
	int f;
	RCC_ClocksTypeDef clks;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/*clock enable*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	RCC_GetClocksFreq(&clks);
	f = clks.PCLK1_Frequency;
	f <<= (clks.HCLK_Frequency == clks.PCLK1_Frequency) ? 0 : 1;

	/* time base configuration */
	TIM_TimeBaseStructure.TIM_Period = f / freq;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;//div - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	//TIM_CtrlPWMOutputs(TIM1, ENABLE);

	/*config pin*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*config ch*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = f / freq / 2;//cfg -> fs >> 1; //default to 50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//TIM_SetCompare1(TIM1, 70);
}

//PB9
//freq >= 50000
static void time4_ch4_Init(int freq)
{
	int f;
	RCC_ClocksTypeDef clks;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/*clock enable*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	RCC_GetClocksFreq(&clks);
	f = clks.PCLK1_Frequency;
	f <<= (clks.HCLK_Frequency == clks.PCLK1_Frequency) ? 0 : 1;

	/* time base configuration */
	TIM_TimeBaseStructure.TIM_Period = f / freq;  //this value must not over 65535(16bit)
	TIM_TimeBaseStructure.TIM_Prescaler = 0;//div - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	//TIM_CtrlPWMOutputs(TIM1, ENABLE);

	/*config pin*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*config ch*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = f / freq / 2;//cfg -> fs >> 1; //default to 50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	//TIM_SetCompare1(TIM1, 70);
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
//clock 2 tim4 ch4 PB9
void clock2_Init(int frq, int dc)
{
	pwm_cfg_t cfg;
	const pwm_bus_t *pwm = &pwm44;
	cfg.hz = frq;
	cfg.fs = 100;
	pwm -> init(&cfg);
	pwm -> set(dc);
}

q15_t float_to_q15(float a)
{
	short int b = 0;
	b = (short)(a * (1 << 15));
	return b;
}

void onesided(long nfill)
{
	uint32_t i;

	lBUFMAG[0] = lBUFMAG[0];
	lBUFMAG[nfill/2] = lBUFMAG[nfill/2];
	for (i = 1; i < nfill / 2; i++) {
		lBUFMAG[i] = lBUFMAG[i] + lBUFMAG[nfill - i];
		lBUFMAG[nfill - i] = 0x0;
	}
}

void findmax(long *buf, int nfill, int *max_index) 
{
	long max = buf[0];
	*max_index = 0;
	for(int i = 1; i < nfill; i++) {
		if(max < buf[i]) {
			max = buf[i];
			*max_index = i;
		}
	}
}

void powerMag(long nfill, char* strPara)
{
	int32_t lX,lY;
	uint32_t i;
	int max_index = 0;
	for (i = 0; i < nfill; i++) {
		lX = (lBUFOUT[i] << 16) >> 16; /* sine_cosine --> cos */
		lY = (lBUFOUT[i] >> 16);   /* sine_cosine --> sin */    
		float X = 1024 * ((float)lX) / 32768;
		float Y = 1024 * ((float)lY) / 32768;
		float Mag = sqrt(X * X + Y * Y) / nfill;
		//float Mag = X * X + Y * Y;
		lBUFMAG[i] = (uint32_t)(Mag * 65536);
	}
	if (strPara == "1SIDED") onesided(nfill);
	
	lBUFMAG[0] = 0;	//clear the average value
	findmax(lBUFMAG, nfill, &max_index);
	if(max_index > nfill / 2) {
		max_index = nfill - max_index;
	}
	float freq = 1.0 * max_index * 6000 / 1024 / 7;
	printf("%d %d %fkhz\r\n",lBUFMAG[max_index], max_index, freq);
        if((freq > 99) && (freq < 102)) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
	} else if((freq > 113) && (freq < 117)) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
	} else if((freq > 128) && (freq < 132)) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
	} else if((freq > 143) && (freq < 147)) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_SET);
	} else {
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
	}
	printf("End fft\r\n");
}

void adc_Init(void)
{
	//RCC_ClocksTypeDef clks;
	//ADC1 + DMA
	/*ADC1/2/3 channel0 PA0 ADC1/2/3 channel1 PA1*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); /*72Mhz/6 = 12Mhz*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);
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
	DMA_InitStructure.DMA_BufferSize = DMA_C * DMA_N; //DMA缓存大小 //2 * 1024
	DMA_InitStructure.DMA_PeripheralInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //目标内存地址自动后移
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//DMA传送数据尺寸16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//常用循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//禁止内存到内存模式
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	//DMA_Init(DMA1_Channel1, &DMA_InitStructure); 
	//ADC Config 
	ADC_InitTypeDef ADC_InitStructure;
	ADC_DeInit(ADC1);
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//!!
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//!!
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = DMA_C; //2
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_1Cycles5); //9Mhz/(239.5 + 12.5) = 35.7Khz
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_1Cycles5); //9Mhz/(239.5 + 12.5) = 35.7Khz
	//ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
	//ADC_ExternalTrigConvCmd(ADC1, DISABLE);
	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
	
	//ADC2 + NO_DMA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	ADC_DeInit(ADC2);
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//!!
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//!!
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = DMA_C; //2
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5); //12Mhz/(239.5 + 12.5) = 35.7Khz
	//ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
	//ADC_ExternalTrigConvCmd(ADC1, DISABLE);
	ADC_DMACmd(ADC2,ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_ResetCalibration(ADC2);
	while (ADC_GetResetCalibrationStatus(ADC2));
	ADC_StartCalibration(ADC2);
	while (ADC_GetCalibrationStatus(ADC2));
	ADC_SoftwareStartConvCmd(ADC2, DISABLE);
	//ADC_ExternalTrigConvCmd(ADC2, DISABLE);
	//ADC3 + DMA
	DMA_DeInit(DMA2_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x40013C4C;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_ConvertedValue_temp;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = DMA_C * DMA_N;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Channel5, &DMA_InitStructure);  
	/* Enable DMA2 channel5 */
	DMA_Cmd(DMA2_Channel5, ENABLE);
	RCC_ClocksTypeDef clks;	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	ADC_DeInit(ADC3);
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//!!
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//!!
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = DMA_C; //2
	ADC_Init(ADC3, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5); //12Mhz/(239.5 + 12.5) = 35.7Khz
	//ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
	//ADC_ExternalTrigConvCmd(ADC1, DISABLE);
	ADC_DMACmd(ADC3,ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	ADC_ResetCalibration(ADC3);
	while (ADC_GetResetCalibrationStatus(ADC3));
	ADC_StartCalibration(ADC3);
	while (ADC_GetCalibrationStatus(ADC3));
	ADC_SoftwareStartConvCmd(ADC3, DISABLE);
	//ADC_ExternalTrigConvCmd(ADC2, DISABLE);
	RCC_GetClocksFreq(&clks);
}

void my_sine(uint32_t W_freq, q15_t *W_buf, uint32_t W_ampli)
{
	for(uint32_t W_i = 0; W_i < NUM_FFT; W_i++) {
		W_buf[W_i] = (q15_t)(W_ampli * (sin(PI2 * W_freq * W_i / NUM_FFT) + 0)) + (q15_t)(0.5 * W_ampli * (sin(PI * W_freq * W_i / NUM_FFT) + 0));
	}
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
	//clock2_Init(100000,50);
	scan_init(100);	//10ms scan
	adc_Init();
	time3_ch1_Init(100000);
	//time4_ch4_Init(30000);
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

int* adc(int ch)
{
	int adc_value[2][DMA_N] = {0};
	DMA_Cmd(DMA1_Channel1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
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
void fft_asm()
{
	cr4_fft_1024_stm32(lBUFOUT, lBUFIN, NPT);
	powerMag(NPT,"2SIDED");
}
void fft()
{
	uint32_t index = 0;
	float max = 0;
	arm_cfft_radix4_instance_f32 arm_cfft_sR_f32_len1024;
	arm_cfft_radix4_init_f32(&arm_cfft_sR_f32_len1024, NUM_FFT, 0, 1);
	//arm_cfft_radix4_init_q15(&S,NUM_FFT, 0, 1);
	//my_sine(2000, wBuf_in, 1024);
	//for(uint32_t W_i = 0; W_i < NUM_FFT; W_i++) {
	//	wTest_input[W_i * 2] = wBuf_in[W_i];
	//}
	printf("Begin fft\r\n");
	arm_cfft_radix4_f32(&arm_cfft_sR_f32_len1024, fTest_input);
	//arm_cfft_radix4_q15(&S,wTest_input);	
	//for(uint32_t W_i = 0; W_i < NUM_FFT; W_i++) {
		//wTest_output[W_i] = wTest_input[W_i * 2 + 2] * wTest_input[W_i * 2 + 2] + wTest_input[W_i * 2 + 1] * wTest_input[W_i * 2 + 1];	//discard the average value or mean value
	//}
	//wTest_input[0] = 0;
	//arm_cmplx_mag_q15(wTest_input,wTest_output,NUM_FFT);
	fTest_input[0] = 0;
	arm_cmplx_mag_f32(fTest_input,fTest_output,NUM_FFT);
	//arm_max_q15(wTest_output,NUM_FFT,&max,&index);
	/*for(int i = 0; i < NUM_FFT; i++) {
		printf("%f\r\n", fTest_output[i]);
	}*/
	arm_max_f32(fTest_output, NUM_FFT, &max, &index);
	if(index > NUM_FFT / 2) {
		index = NUM_FFT - index;
	}
	float freq = 1.0 * index * 6000 / 1024 / 7;
	printf("%f %d %fkhz\r\n",max, index, freq);
	if((freq > 99) && (freq < 102)) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
	} else if((freq > 113) && (freq < 117)) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
	} else if((freq > 128) && (freq < 132)) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
	} else if((freq > 143) && (freq < 147)) {
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_SET);
	} else {
		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);
	}
	printf("End fft\r\n");
}
//key scan init : TIM2 as the clock source
void scan_init(int frq)
{
	pwm_cfg_t cfg;
	const pwm_bus_t *pwm = &pwm2;
	cfg.hz = frq;
	cfg.fs = 100;
	pwm->init(&cfg);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//占先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;           //响应级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //启动
	NVIC_Init(&NVIC_InitStructure);  	//初始化
}
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) { 
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		s_cFlag_10ms_interrup = 1;
	}
}
void pc_Updata(void)
{
	static uint8_t s_cKey = N_key;
	static uint32_t s_wFreq = 100000;	//init 100Khz
	static uint8_t s_cFlag_key_complete = 1;
	/*if(s_cFlag_10ms_interrup) {
		s_cFlag_10ms_interrup = 0;
		s_cKey = key_read();
		if(S_key == s_cKey) {
			printf("single key!\r\n");
			s_wFreq += 5000;
			TIM3 -> ARR = 72000000 / s_wFreq;
			//TIM3 -> EGR = TIM_PSCReloadMode_Immediate;
			s_cFlag_key_complete = 1;
		} else if(D_key == s_cKey) {
			printf("Double key!\r\n");
		} else if(L_key == s_cKey) {
			printf("Long key!\r\n");
		}
	}*/
	if(s_cFlag_key_complete) {
		//s_cFlag_key_complete = 0;
		ADC_SoftwareStartConvCmd(ADC1,ENABLE);
		pc_mdelay(10);
		for(uint32_t W_i = 0; W_i < NUM_FFT; W_i++) {
			//fTest_input[2 * W_i] = 1.0 * ADC_ConvertedValue[W_i][0] * 3300 / 4095;
			//fTest_input[2 * W_i + 1] = 0.0;
			lBUFIN[W_i] = (long)ADC_ConvertedValue[W_i][0];
		}
		fft_asm();
		//ADC_SoftwareStartConvCmd(ADC1,DISABLE);
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
#endif
typedef void (*reset)(void);
static int cmd_fft_func(int argc, char *argv[])
{
		const char *usage = {
		" fft usage : \r\n"
		" fft adc \r\n"
		" fft pwm1 freq duty\r\n"
		" fft fft 1\r\n"
		" ... \r\n"
	};
	if(argc < 2){
		printf("%s", usage);
		return 0;
	}
	if(3 == argc) {
		if(!strcmp(argv[1], "fft")) {
			//ADC_ExternalTrigConvCmd(ADC1, ENABLE);
			ADC_SoftwareStartConvCmd(ADC1,ENABLE);
			pc_mdelay(10);
			uint32_t W_temp = atoi(argv[2]);
			if(1 == W_temp) {
				for(uint32_t W_i = 0; W_i < NUM_FFT; W_i++) {
					wTest_input[2 * W_i] = (q15_t)ADC_ConvertedValue[W_i][0];
					wTest_input[2 * W_i + 1] = 0;
					fTest_input[2 * W_i] = 1.0 * ADC_ConvertedValue[W_i][0] * 3300 / 4095;
					fTest_input[2 * W_i + 1] = 0;
				}				
			} else if(3 == W_temp) {
				for(uint32_t W_i = 0; W_i < NUM_FFT; W_i++) {
					wTest_input[2 * W_i] = (q15_t)ADC_ConvertedValue_temp[W_i];
					wTest_input[2 * W_i + 1] = 0;
				}
			}
			fft();
			ADC_SoftwareStartConvCmd(ADC1,DISABLE);
		}
		if(!strcmp(argv[1], "adc") && !strcmp(argv[2], "1")) {
			//ADC_ExternalTrigConvCmd(ADC1, ENABLE);
			ADC_SoftwareStartConvCmd(ADC1,ENABLE);
			pc_mdelay(10);
			for(uint32_t W_i = 0; W_i < DMA_N; W_i++) {
				printf("%d ",ADC_ConvertedValue[W_i][0]);
				//printf("%d ",ADC_ConvertedValue[W_i][1]);
				printf("\r\n");
			}
		}
		if(!strcmp(argv[1], "adc") && !strcmp(argv[2], "2")) {
			//ADC_ExternalTrigConvCmd(ADC2, ENABLE);
			ADC_SoftwareStartConvCmd(ADC2,ENABLE);
			pc_mdelay(10);
			for(uint32_t W_i = 0; W_i < DMA_N; W_i++) {
				ADC_ConvertedValue_temp[W_i] = ADC_GetConversionValue(ADC3);
			}
			for(uint32_t W_i = 0; W_i < DMA_N; W_i++) {
				printf("%d %d\r\n", W_i, ADC_ConvertedValue_temp[W_i]);
			}
			ADC_SoftwareStartConvCmd(ADC2,DISABLE);
			//ADC_ExternalTrigConvCmd(ADC2, DISABLE);
		}
		if(!strcmp(argv[1], "adc") && !strcmp(argv[2], "3")) {
			//ADC_ExternalTrigConvCmd(ADC2, ENABLE);
			ADC_SoftwareStartConvCmd(ADC3,ENABLE);
			pc_mdelay(10);
			//for(uint32_t W_i = 0; W_i < DMA_N; W_i++) {
				//ADC_ConvertedValue_temp[W_i] = ADC_GetConversionValue(ADC2);
			//}
			for(uint32_t W_i = 0; W_i < DMA_N; W_i++) {
				printf("%d %d\r\n", W_i, ADC_ConvertedValue_temp[W_i]);
			}
			ADC_SoftwareStartConvCmd(ADC3,DISABLE);
			//ADC_ExternalTrigConvCmd(ADC2, DISABLE);
		}
	}
	if(4 == argc) {
		if(!strcmp(argv[1], "pwm3")) {
			uint32_t W_freq = 0;
			uint32_t W_duty = 0;
			W_freq = atoi(argv[2]);
			W_duty = atoi(argv[3]);
			printf("%d,%d\r\n",W_freq,W_duty);
			//clock0_Init(W_freq, W_duty);
			time3_ch1_Init(W_freq);
			printf("PWM3 OK!\r\n");
		}
		if(!strcmp(argv[1], "pwm4")) {
			uint32_t W_freq = 0;
			uint32_t W_duty = 0;
			W_freq = atoi(argv[2]);
			W_duty = atoi(argv[3]);
			printf("%d,%d\r\n",W_freq,W_duty);
			time4_ch4_Init(W_freq);
			//clock2_Init(W_freq,W_duty);
			printf("PWM4 OK!\r\n");
		}
		if(!strcmp(argv[1],"fft") && !strcmp(argv[2],"asm")) {
			ADC_SoftwareStartConvCmd(ADC1,ENABLE);
			pc_mdelay(10);
			for(uint32_t W_i = 0; W_i < DMA_N; W_i++) {
				printf("%d ",ADC_ConvertedValue[W_i][0]);
				printf("\r\n");
			}
			ADC_SoftwareStartConvCmd(ADC1,DISABLE);
			for(uint32_t W_i = 0; W_i < NPT; W_i++) {
				lBUFIN[W_i] = (long)ADC_ConvertedValue[W_i][0];
			}
			fft_asm();
		}
	}
	return 0;
}
cmd_t cmd_fft = {"fft", cmd_fft_func, "commands for fft"};
DECLARE_SHELL_CMD(cmd_fft)
