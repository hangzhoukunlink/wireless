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
#include "can.h"
#include "sys/malloc.h"
#include "mbi5025.h"
#include "config.h"
#include "my_key.h"


#define d2mv(d) ((d) * 2487 / 4096)
#define mv2d(mv) ((mv) * 4096 / 2500)
#define DMA_N 1024   //每个通道的数据长度
#define DMA_C 1    //ADC通道数

static const mbi5025_t segment_mbi5025 = {
		.bus = &spi1,
		.idx = SPI_CS_DUMMY,
		.load_pin = SPI_CS_PA4,
		.oe_pin = SPI_CS_PC4,
};
static const can_bus_t* probe_can_bus = &can1;
//0~9 -> mbi5025, the last one is dp 
static const char led_7[11] = {0xfc,0x60,0xda,0xf2,0x66,0xb6,0xbe,0xe0,0xfe,0xf6,0x01};
short ADC_ConvertedValue[DMA_N][DMA_C] = {0};	//
short ADC_ConvertedValue_temp[DMA_N] = {0};
short After_filter[DMA_C] = {0};
static uint8_t s_cFlag_10ms_interrup = 0;
static uint32_t code_matrix = 0; //get from can message of control card

void scan_init(int frq);
void adc_init(void);
void key_init(void);
void can_init(void);
static int probe_mdelay(int ms);
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

void adc_init(void)
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
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5); //9Mhz/(239.5 + 12.5) = 35.7Khz
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
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

//mbi5024 init
void segment_init(void)
{
	mbi5025_Init(&segment_mbi5025);		//SPI bus,shift register
	mbi5025_EnableOE(&segment_mbi5025); //Enable the OutPin 
}
//segment ++ : for test
void segment_update(void)
{
	static int temp = 0;
	temp++;
	if(temp < 10) {
		mbi5025_WriteByte(&segment_mbi5025, led_7[temp]);
		mbi5025_WriteByte(&segment_mbi5025, 0);
		mbi5025_WriteByte(&segment_mbi5025, 0);
		mbi5025_WriteByte(&segment_mbi5025, 0);
	} else if(temp < 100) {
		mbi5025_WriteByte(&segment_mbi5025, led_7[temp % 10]);
		mbi5025_WriteByte(&segment_mbi5025, led_7[temp / 10]);
		mbi5025_WriteByte(&segment_mbi5025, 0);
		mbi5025_WriteByte(&segment_mbi5025, 0);
	} else if(temp < 1000) {
		mbi5025_WriteByte(&segment_mbi5025, led_7[temp % 10]);
		mbi5025_WriteByte(&segment_mbi5025, led_7[temp / 10 % 10]);
		mbi5025_WriteByte(&segment_mbi5025, led_7[temp / 100]);
		mbi5025_WriteByte(&segment_mbi5025, 0);
	} else if(temp < 10000) {
		mbi5025_WriteByte(&segment_mbi5025, led_7[temp % 10]);
		mbi5025_WriteByte(&segment_mbi5025, led_7[temp / 10 % 10]);
		mbi5025_WriteByte(&segment_mbi5025, led_7[temp / 100 % 10]);
		mbi5025_WriteByte(&segment_mbi5025, led_7[temp / 1000]);
	} else {
		temp = 0;
	}
	spi_cs_set(segment_mbi5025.load_pin, 1);
	spi_cs_set(segment_mbi5025.load_pin, 0);
}
//segment display
//on -> dp, off -> No dp
void segment_display(int code, char on)
{
	if(on) {
		if(0 == code) {
			mbi5025_WriteByte(&segment_mbi5025, 0);
			mbi5025_WriteByte(&segment_mbi5025, led_7[10]);
			mbi5025_WriteByte(&segment_mbi5025, 0);
			mbi5025_WriteByte(&segment_mbi5025, 0);
		} else if(code < 10) {
			mbi5025_WriteByte(&segment_mbi5025, led_7[code]);
			mbi5025_WriteByte(&segment_mbi5025, 0 + led_7[10]);
			mbi5025_WriteByte(&segment_mbi5025, 0);
			mbi5025_WriteByte(&segment_mbi5025, 0);
		} else if(code < 100) {
			mbi5025_WriteByte(&segment_mbi5025, led_7[code % 10]);
			mbi5025_WriteByte(&segment_mbi5025, led_7[code / 10] + led_7[10]);
			mbi5025_WriteByte(&segment_mbi5025, 0);
			mbi5025_WriteByte(&segment_mbi5025, 0);
		} else if(code < 1000) {
			mbi5025_WriteByte(&segment_mbi5025, led_7[code % 10]);
			mbi5025_WriteByte(&segment_mbi5025, led_7[code / 10 % 10] + led_7[10]);
			mbi5025_WriteByte(&segment_mbi5025, led_7[code / 100]);
			mbi5025_WriteByte(&segment_mbi5025, 0);
		} else if(code < 10000) {
			mbi5025_WriteByte(&segment_mbi5025, led_7[code % 10]);
			mbi5025_WriteByte(&segment_mbi5025, led_7[code / 10 % 10] + led_7[10]);
			mbi5025_WriteByte(&segment_mbi5025, led_7[code / 100 % 10]);
			mbi5025_WriteByte(&segment_mbi5025, led_7[code / 1000]);
		}
	} else {
		if(0 == code) {
			mbi5025_WriteByte(&segment_mbi5025, 0);
			mbi5025_WriteByte(&segment_mbi5025, 0);
			mbi5025_WriteByte(&segment_mbi5025, 0);
			mbi5025_WriteByte(&segment_mbi5025, 0);
		} else if(code < 10) {
			mbi5025_WriteByte(&segment_mbi5025, led_7[code]);
			mbi5025_WriteByte(&segment_mbi5025, 0);
			mbi5025_WriteByte(&segment_mbi5025, 0);
			mbi5025_WriteByte(&segment_mbi5025, 0);
		} else if(code < 100) {
			mbi5025_WriteByte(&segment_mbi5025, led_7[code % 10]);
			mbi5025_WriteByte(&segment_mbi5025, led_7[code / 10]);
			mbi5025_WriteByte(&segment_mbi5025, 0);
			mbi5025_WriteByte(&segment_mbi5025, 0);
		} else if(code < 1000) {
			mbi5025_WriteByte(&segment_mbi5025, led_7[code % 10]);
			mbi5025_WriteByte(&segment_mbi5025, led_7[code / 10 % 10]);
			mbi5025_WriteByte(&segment_mbi5025, led_7[code / 100]);
			mbi5025_WriteByte(&segment_mbi5025, 0);
		} else if(code < 10000) {
			mbi5025_WriteByte(&segment_mbi5025, led_7[code % 10]);
			mbi5025_WriteByte(&segment_mbi5025, led_7[code / 10 % 10]);
			mbi5025_WriteByte(&segment_mbi5025, led_7[code / 100 % 10]);
			mbi5025_WriteByte(&segment_mbi5025, led_7[code / 1000]);
		}
	}
	spi_cs_set(segment_mbi5025.load_pin, 1);
	spi_cs_set(segment_mbi5025.load_pin, 0);
}

//set filter of can
void can_init(void)
{
	can_cfg_t cfg_pdi_can = {
		.baud = 500000,
	};
	probe_can_bus -> init(&cfg_pdi_can);	// init can
	can_filter_t filter[] = {
		{
			.id = 0x5a0,
			.mask = 0xffff,
			.flag = 0,
		},
	};
	probe_can_bus -> filt(filter, 1);
}
void probe_Init(void)
{
	scan_init(100);	//10ms scan
	adc_init();
	key_init();
	segment_init();
	can_init();
}

//0 -> No touch
//1 -> touch
int key_adc(void)
{
	int result = 0;
	for(int i = 0; i < DMA_N; i++) {
		result += ADC_ConvertedValue[i][0];
	}
	result >>= 10;	//get average
	if(d2mv(result) > 900) {
		return 1;
	}
	return 0;
}

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
void probe_Updata(void)
{
	static uint8_t s_cKey = N_key;
	static uint8_t time_interrup = 0;
	static uint8_t flag_time_interrup = 0;
	if(s_cFlag_10ms_interrup) {
		s_cFlag_10ms_interrup = 0;
		time_interrup++;
		if(time_interrup > 10) {
			time_interrup = 0;
			flag_time_interrup = 1;	//1s
		}
		s_cKey = key_read();
		if(N_key != s_cKey) {
			printf("key is pressed!\r\n");
		}
		//printf("%d\r\n", d2mv(ADC1 -> DR));
	}
	if(key_adc()) {
		led_on(LED_RED);
		segment_display(code_matrix,1);
	} else {
		led_off(LED_RED);
		segment_display(code_matrix,0);
	}
	if(flag_time_interrup) {
		flag_time_interrup = 0;
		//segment_update();
		//printf("%d\r\n", d2mv(ADC1 -> DR));
	}
}
int main(void)
{
	task_Init();
	probe_Init();
	while(1){
		task_Update();
		probe_Updata();
	} 
}

static int probe_mdelay(int ms)
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
static int cmd_probe_func(int argc, char *argv[])
{
		const char *usage = {
		"probe usage : \r\n"
		"probe dispaly x\r\n"
		"probe key adc\r\n"
	};
	if(argc < 2){
		printf("%s", usage);
		return 0;
	}
	if(argc == 3) {
		if(!strcmp(argv[1],"display")) {
			code_matrix = atoi(argv[2]);
			segment_display(code_matrix,1);
		}
		//PA0 ADC Ch1
		if(!strcmp(argv[1],"key") && !strcmp(argv[2],"adc")) {
			for(int i = 0; i < DMA_N; i++) {
				printf("0x%x %dmv\r\n",ADC_ConvertedValue[i][0],d2mv(ADC_ConvertedValue[i][0]));
			}
		}
	}
	return 0;
}
cmd_t cmd_probe = {"probe", cmd_probe_func, "commands for probe"};
DECLARE_SHELL_CMD(cmd_probe)
