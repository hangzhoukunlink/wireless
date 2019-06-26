/*******************************
 *ccf@2017.3 initial version
 *Functions as followed: 
 *1.Set the filter of can according to encode_read
 *2.Can communication : how to receive??
 *3.Accoding to ID1 Set Voltage and Current(PWM DUTY)
 *4.
 *5.
 *6.Calibration??
 *7.NVM??
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
#include "led.h"
#include "nvm.h"
#include "sys/malloc.h"
#include "config.h"
#include "cwb_drive.h"
#include "dac.h"
#include "filter.h"
#define VEF 3300
#define Vref_PWM 2.500f 
#define D2MV(D) ((D) * VEF / 4095)  //2978 is ref voltage
#undef   __DEBUG__
#define ADC_CHANNEL_NUM   16
#define ADC_SAMPLE_COUNT  8
vu16 AD_Sample_Value[ADC_SAMPLE_COUNT][ADC_CHANNEL_NUM];
void lx_can_choice(int x);
int lx_can_send(void);
struct filter_s  vin_filter_data;
struct filter_s  vout_filter_data;
//golbal var
uint32_t wCanId = 0;
CWB_MISC_T tCwbViMsg __nvm;
CWB_MISC_T tCwbViMsg1;
CWB_MISC_T tCwbViMsg2;
CWB_MISC_T tCwbViMsg3;
CWB_MISC_T tCwbViMsg4;
CWB_MISC_T tCwbViMsgFault;

CWB_MODE_E  tWorkMode  = START;
CWB_MODE_E1 tWorkMode1 = START1;
CWB_MODE_E2 tWorkMode2 = START2;
CWB_MODE_E3 tWorkMode3 = START3;
CWB_MODE_E4 tWorkMode4 = START4;
//START END POLL CFG READ
 
CWB_STATUS_E1 tWorkStatus1 = IDLE1;
CWB_STATUS_E2 tWorkStatus2 = IDLE2;
CWB_STATUS_E3 tWorkStatus3 = IDLE3;
CWB_STATUS_E4 tWorkStatus4 = IDLE4;

ERROR_E1 tError1 = ERROR_NO1;
ERROR_E2 tError2 = ERROR_NO2;
ERROR_E3 tError3 = ERROR_NO3;
ERROR_E4 tError4 = ERROR_NO4;

static uint16_t can_count1 = 0;
static uint16_t can_count3 = 0;


static uint16_t s_chCount1 = 0;
static uint16_t s_chCount2 = 0;
static uint16_t s_chCount3 = 0;
static uint16_t s_chCount4 = 0;




can_msg_t tCwbCanTx = {0, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 0};
can_msg_t tCwbCanTx1 = {0, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 0};
can_msg_t tCwbCanTx2 = {0, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 0};
can_msg_t tCwbCanTx3 = {0, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 0};
can_msg_t tCwbCanTx4 = {0, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 0};
can_msg_t tCwbCanTx5 = {0x53, 8, {1, 2, 3, 4, 5, 6, 7, 8}, 0};
uint8_t   busyflag=0;

can_msg_t tCwbCanTxFault = {0, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 0};
can_msg_t tCwbCanRx = {0, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 0};
can_msg_t tCwbCanCfg[20]; //for recv multi frame
uint8_t chNumCfg = 0;  //count the num of cfg
uint8_t chNumCfgTotal = 0;
uint16_t BASE_MV_IN = 0;
uint16_t BASE_MV_OUT = 0;


static bool s_bFlagCanInterrup = FALSE; // can receive flag
static bool s_bFlagCanScan = FALSE; // for test can communication 



void cwb_update();
void cwb_mode_choose(void);
bool vi_to_can(CWB_MISC_T *tCwbViMsg, CWB_MODE_E tWorkMode);
bool can_to_vi(CWB_MISC_T *tCwbViMsg, CWB_MODE_E tWorkMode);
bool eight_char_to_four_char(char *chBuf_In, uint8_t *chBufOut);
bool bDebug = FALSE;


void GenerateSystemReset(void);
bool check_dut_no_power1(void);
bool check_dut_no_power2(void);
bool check_dut_no_power3(void);
bool check_dut_no_power4(void);


static int  canerror=0;
static int  canerror1=0;
static int  canerror3=0;
#define  RX_SZ 32
static char rx[RX_SZ]="";
static int  s_idx=0;

int uart_Update(void)
{  
    int ret,ready=0;
    while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE)){
        while(1){
	        ret = (int) USART_GetFlagStatus(USART3, USART_FLAG_RXNE);
		if(ret) break;
        }
        rx[s_idx] =  USART_ReceiveData(USART3);
         
        if(rx[s_idx] == '\n'){ 
             ready = 1;
             s_idx=0;
                     
         }
        s_idx++;
        if(s_idx >= RX_SZ)
            s_idx= 0;
    }
	    if(ready){             
	    ready = 0; 
	    s_idx = 0;
	    if(!strncmp(rx,"aaaa",4)){
		    canerror=0x10;
	    }else if(!strncmp(rx,"bbbb",4)){
		    canerror=0x20;
	    }else if(!strncmp(rx,"cccc",4)){
		    canerror=0x30;
	    }else if(!strncmp(rx,"dddd",4)){
		    canerror=0x40;
	    }else if(!strncmp(rx,"eeee",4)){
		    canerror=0x50;
	    }else if(!strncmp(rx,"ffff",4)){
		    canerror=0x60;
	    }
	    else if(!strncmp(rx,"gggg",4)){
		    canerror=0x70;
	    }
	  else if(!strncmp(rx,"hhhh",4)){
		    canerror=0x80;
	  }else if(!strncmp(rx,"iiii",4)){
		    canerror=0x90;
	  }
	  }
  return 0;
  
}



typedef enum {
    AUTOMATIC,
    MAMUAL = !AUTOMATIC,
} pc_shell_t;
pc_shell_t pc_shell = AUTOMATIC;

uint32_t cwb_mdelay(uint32_t wMs)
{
    uint32_t wLeft;
    time_t tDeadline = time_get(wMs);
    do {
            wLeft = time_left(tDeadline);
            if(wLeft >= 10){ //system update period is expected less than 10ms
                ulp_update();
            }
           
    } while(wLeft > 0);
    return 0;
}

uint32_t lxall_mdelay(uint32_t wMs)
{
    uint32_t wLeft;
    time_t tDeadline = time_get(wMs);
    do {
            wLeft = time_left(tDeadline);
            if(wLeft >= 10){ //system update period is expected less than 10ms
                ulp_update();
            }
            if(END == tWorkMode){ //no need delay complete
                return 1;
            }
           
    } while(wLeft > 0);
    return 0;
}


uint32_t lx1_mdelay(uint32_t wMs)
{
    uint32_t wLeft;
    time_t tDeadline = time_get(wMs);
    do {
            wLeft = time_left(tDeadline);
            if(wLeft >= 10){ //system update period is expected less than 10ms
                ulp_update();
            }
            if(END1 == tWorkMode1){ //no need delay complete
                return 1;
            }
    } while(wLeft > 0);
    return 0;
}

uint32_t lx2_mdelay(uint32_t wMs)
{
    uint32_t wLeft;
    time_t tDeadline = time_get(wMs);
    do {
            wLeft = time_left(tDeadline);
            if(wLeft >= 10){ //system update period is expected less than 10ms
                ulp_update();
            }
            if(END2 == tWorkMode2){ //no need delay complete
                return 1;
            }
    } while(wLeft > 0);
    return 0;
}

uint32_t lx3_mdelay(uint32_t wMs)
{
    uint32_t wLeft;
    time_t tDeadline = time_get(wMs);
    do {
            wLeft = time_left(tDeadline);
            if(wLeft >= 10){ //system update period is expected less than 10ms
                ulp_update();
            }
            if(END3 == tWorkMode3){ //no need delay complete
                return 1;
            }
    } while(wLeft > 0);
    return 0;
}

uint32_t lx4_mdelay(uint32_t wMs)
{
    uint32_t wLeft;
    time_t tDeadline = time_get(wMs);
    do {
            wLeft = time_left(tDeadline);
            if(wLeft >= 10){ //system update period is expected less than 10ms
                ulp_update();
            }
            if(END4 == tWorkMode4){ //no need delay complete
                return 1;
            }
    } while(wLeft > 0);
    return 0;
}




void lx_can_a1()
{
       lx_can_choice(CHANNEL_A1);
       lx1_mdelay(1000);
       canerror1=canerror;
       printf("canerror1 is %d\n",canerror1);
}
void lx_can_c1()
{
       lx_can_choice(CHANNEL_C1);
       lx3_mdelay(1000);
       canerror3=canerror;
       printf("canerror3 is %d\n",canerror3);
}

static void ADC_init(void)
{
  /*	ADC1
        PC0:CHANNEL0	PC1:CHANNEL1	PC2:CHANNEL2      PC3:CHANNEL3
        PA2:CHANNEL_I   PA3:CHANNEL_V   PA4:CHANNEL_PWM
  */
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
    
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入引脚
    GPIO_Init(GPIOC, &GPIO_InitStructure);
     
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入引脚
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 |GPIO_Pin_5 |GPIO_Pin_6 | GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入引脚
    GPIO_Init(GPIOA, &GPIO_InitStructure);
        
/*	Initializes the ADCx peripheral		*/
    ADC_InitTypeDef ADC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); /*72Mhz/6 = 12Mhz, note: 14MHz at most*/
    ADC_DeInit(ADC1);
	
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; 	//ADC工作模式:ADC1和ADC2工作在独立模式
    ADC_InitStructure.ADC_ScanConvMode =ENABLE; 		//模数转换工作在扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 	//模数转换工作在连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //外部触发转换关闭
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//ADC数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = ADC_CHANNEL_NUM; 	//顺序进行规则转换的ADC通道的数目
    ADC_Init(ADC1, &ADC_InitStructure); 			//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器
        
    //设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
    //ADC1,ADC通道x,规则采样顺序值为y,采样时间为239.5周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0,  9,  ADC_SampleTime_239Cycles5 );//PA0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1,  10, ADC_SampleTime_239Cycles5 );//PA1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2,  11, ADC_SampleTime_239Cycles5 );//PA2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3,  12, ADC_SampleTime_239Cycles5 );//PA3
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  13, ADC_SampleTime_239Cycles5 );//PA4
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5,  14, ADC_SampleTime_239Cycles5 );//PA5
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6,  15, ADC_SampleTime_239Cycles5 );//PA6
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7,  16, ADC_SampleTime_239Cycles5 );//PA7
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8,  7,  ADC_SampleTime_239Cycles5 );//PB0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9,  8,  ADC_SampleTime_239Cycles5 );//PB1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1,  ADC_SampleTime_239Cycles5 );//PC0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2,  ADC_SampleTime_239Cycles5 );//PC1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3,  ADC_SampleTime_239Cycles5 );//PC2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4,  ADC_SampleTime_239Cycles5 );//PC3
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 5,  ADC_SampleTime_239Cycles5 );//PC4
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 6,  ADC_SampleTime_239Cycles5 );//PC5
    // 开启ADC的DMA支持（要实现DMA功能，还需独立配置DMA通道等参数）
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE); //使能指定的ADC1       
    ADC_ResetCalibration(ADC1); //复位指定的ADC1的校准寄存器        
    while(ADC_GetResetCalibrationStatus(ADC1)); //获取ADC1复位校准寄存器的状态,设置状态则等待
    ADC_StartCalibration(ADC1); //开始指定ADC1的校准状态
    while(ADC_GetCalibrationStatus(ADC1)); //获取指定ADC1的校准程序,设置状态则等待
//    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/************************************** 
 *	configure DMA function 
 **************************************/
static void DMA_ADC_CHANNEL1_init(void)
{
  /*	configure DMA channel1, data from ADC_>DR to DMA buffer	*/
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Channel1); //将DMA的通道1寄存器重设为缺省值
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //DMA外设ADC基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Sample_Value; //DMA内存基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //内存作为数据传输的目的地
    DMA_InitStructure.DMA_BufferSize = ADC_SAMPLE_COUNT*ADC_CHANNEL_NUM; //DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //数据宽度为16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //数据宽度为16位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //工作在循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA通道 x拥有高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //DMA通道x没有设置为内存到内存传输
    DMA_Init(DMA1_Channel1, &DMA_InitStructure); //根据DMA_InitStruct中指定的参数初始化DMA的通道
}


static void lx_LED_Init()
{       
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
        GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
        GPIO_ResetBits(GPIOE, GPIO_Pin_8| GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15 );

	//GPIO_ResetBits(GPIOE, GPIO_Pin_11);
}

static void lx_RELAY_Init()
{
	
  
	GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5 |  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOF, &GPIO_InitStructure);
        GPIO_ResetBits(GPIOF, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5 |  GPIO_Pin_11 | GPIO_Pin_12);
        GPIO_SetBits(GPIOF, GPIO_Pin_14|GPIO_Pin_15);
        
        
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOG, &GPIO_InitStructure);
        GPIO_ResetBits(GPIOG, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6);
        GPIO_SetBits(GPIOG, GPIO_Pin_0|GPIO_Pin_1);
        
        
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
        GPIO_ResetBits(GPIOD,GPIO_Pin_12);
        
       
}


static void El_CURRENT_Init()
{
	/*	PB6:CURRENT_A1	PB7:CURRENT_A2	PB8:CURRENT_B1	PB9:CURRENT_B2    */
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO , ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
        
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);        
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_Cmd(TIM4, DISABLE);
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Period = 500;
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
        TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;


	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
        TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_Cmd(TIM4, ENABLE);
       
        
        /*	PC6:CURRENT_C1	PC7:CURRENT_C2	PC8:CURRENT_D1	PC9:CURRENT_D2    */
       
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
        
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
        GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
        
        
	TIM_Cmd(TIM3, DISABLE);
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = 500;
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
        
        TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
        TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
        
        
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;


	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
        TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_Cmd(TIM3, ENABLE);
       
        
}
void cwb_light_after_power(void)
{
    for(uint8_t chIndex = 0; chIndex < 5; chIndex++) {
        led_off(LED_YELLOW1);
        led_off(LED_YELLOW2);
        led_off(LED_YELLOW3);
        led_off(LED_YELLOW4);
        led_on(LED_RED1);
        led_on(LED_RED2);
        led_on(LED_RED3);
        led_on(LED_RED4);
        cwb_mdelay(300);
        led_off(LED_YELLOW1);
        led_off(LED_YELLOW2);
        led_off(LED_YELLOW3);
        led_off(LED_YELLOW4);
        led_on(LED_GREEN1);
        led_on(LED_GREEN2);
        led_on(LED_GREEN3);
        led_on(LED_GREEN4);
        cwb_mdelay(300);
    }
        led_off(LED_YELLOW1);
        led_off(LED_YELLOW2);
        led_off(LED_YELLOW3);
        led_off(LED_YELLOW4);
}

void __sys_update(void)
{
    cwb_update();
    uart_Update();
}

void power_choose(power x)
{
    switch(x) { 
                case VCC_INL: 
                        GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);  
                        break; 
                case VCC_INH: 
                        GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET); 
                        break; 
                default: 
                        return; 
                } 
}
void ACC_choose(status_t x)
{
    switch(x) { 
                case OFF1: 
                        
                        GPIO_WriteBit(GPIOD, GPIO_Pin_11, Bit_RESET); 
                        break;
                case OFF2: 
                       
                        GPIO_WriteBit(GPIOD, GPIO_Pin_9,  Bit_RESET); 
                        break;
                 
                case ON1: 
                        
                        GPIO_WriteBit(GPIOD, GPIO_Pin_11, Bit_SET); 
                        break; 
                case ON2: 
                        
                        GPIO_WriteBit(GPIOD, GPIO_Pin_9,  Bit_SET);  
                        break; 
                default: 
                        return; 
                } 
}
int lx_set_current( int channel, float operation)
{   
    float currenta1set,currenta2set,currentb1set,currentb2set,currentc1set,currentc2set,currentd1set,currentd2set;
    int regva1set,regva2set,regvb1set,regvb2set,regvc1set,regvc2set,regvd1set,regvd2set;
    
     switch(channel){
        case CHANNEL_A1:
             currenta1set=operation;
             regva1set =(int)((0.5*currenta1set/Vref_PWM)*500);             
             TIM_SetCompare1(TIM4, regva1set);
             
             break;
        case CHANNEL_A2:
             currenta2set=operation;
             regva2set =(int)((0.5*currenta2set/Vref_PWM)*500);                      
             TIM_SetCompare2(TIM4, regva2set);
             
             break;
        case CHANNEL_B1:
             currentb1set=operation;
             regvb1set =(int)((0.5*currentb1set/Vref_PWM)*500);                      
             TIM_SetCompare3(TIM4, regvb1set);
             
             break;
        case CHANNEL_B2:
             currentb2set=operation;
             regvb2set =(int)((0.5*currentb2set/Vref_PWM)*500);                      
             TIM_SetCompare4(TIM4, regvb2set);
             
             break;
        case CHANNEL_C1:
             currentc1set=operation;
             regvc1set =(int)((0.5*currentc1set/Vref_PWM)*500);                      
             TIM_SetCompare1(TIM3, regvc1set);
             
             break;
        case CHANNEL_C2:
             currentc2set=operation;
             regvc2set =(int)((0.5*currentc2set/Vref_PWM)*500);                      
             TIM_SetCompare2(TIM3, regvc2set);
             
             break;
        case CHANNEL_D1:
             currentd1set=operation;
             regvd1set =(int)((0.5*currentd1set/Vref_PWM)*500);                      
             TIM_SetCompare3(TIM3, regvd1set);
             
             break;
        case CHANNEL_D2:
             currentd2set=operation;
             regvd2set =(int)((0.5*currentd2set/Vref_PWM)*500);                      
             TIM_SetCompare4(TIM3, regvd2set);
             
             break;
        default:
                return -1;
        
          
     }
    return 0;
    
}


void power_on(power_set x,led_status_t status){
  
    BitAction ba; 
        switch(status) { 
                case OFF: 
                        ba = Bit_RESET; 
                        break; 
                case ON: 
                        ba = Bit_SET; 
                        break; 
                default: 
                        return; 
        } 
    switch(x) { 
                case  a:    
                        GPIO_WriteBit(GPIOG, GPIO_Pin_4, ba);
                       
                        break; 
                case  b: 
                        GPIO_WriteBit(GPIOG, GPIO_Pin_3, ba);
                        
                        break; 
                case  c: 
                        GPIO_WriteBit(GPIOG, GPIO_Pin_2, ba);
                        
                        break; 
                case  d: 
                        GPIO_WriteBit(GPIOF, GPIO_Pin_11, ba);
                        
                        break; 
                default: 
                        return ; 
        } 
}


void lx_hub_load(power_set x,led_status_t status){
  
    BitAction ba; 
        switch(status) { 
                case OFF: 
                        ba = Bit_RESET; 
                        break; 
                case ON: 
                        ba = Bit_SET; 
                        break; 
                default: 
                        return; 
        } 
    switch(x) { 
                case  a:    
                        GPIO_WriteBit(GPIOF, GPIO_Pin_4, ba);
                        GPIO_WriteBit(GPIOF, GPIO_Pin_5, ba);
                        break; 
                case  b: 
                        GPIO_WriteBit(GPIOF, GPIO_Pin_2, ba);
                        GPIO_WriteBit(GPIOF, GPIO_Pin_3, ba);
                        break; 
                case  c: 
                        GPIO_WriteBit(GPIOG, GPIO_Pin_6, ba);
                        GPIO_WriteBit(GPIOG, GPIO_Pin_5, ba);
                        break; 
                case  d: 
                        GPIO_WriteBit(GPIOF, GPIO_Pin_1, ba);
                        GPIO_WriteBit(GPIOF, GPIO_Pin_0, ba);
                        break; 
                default: 
                        return ; 
        } 
}

static void short_protect_enable(power_set x,led_status_t status){
  
    BitAction ba; 
        switch(status) { 
                case OFF: 
                        ba = Bit_RESET; 
                        break; 
                case ON: 
                        ba = Bit_SET; 
                        break; 
                default: 
                        return; 
        } 
    switch(x) { 
                case  a:    
                        GPIO_WriteBit(GPIOF, GPIO_Pin_14, ba);
                        
                        break; 
                case  b: 
                       GPIO_WriteBit(GPIOF, GPIO_Pin_15, ba);
                        
                        break; 
                case  c: 
                       
                        GPIO_WriteBit(GPIOG, GPIO_Pin_0, ba);
                        break; 
                case  d: 
                        GPIO_WriteBit(GPIOG, GPIO_Pin_1, ba);
                        
                        break; 
                default: 
                        return ; 
      
    } 
}



static void hud_down_select(hub_down x)
{
  
        switch(x) { 
                case  down1:    
                        GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_RESET);
                        
                        break; 
                case  down2: 
                        GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_SET);
                        
                        break; 
                default: 
                        return ; 
        }   
}


static void DMA_ADC_ENABLE(void)
{
     ADC_init();
     DMA_ADC_CHANNEL1_init();
     DMA_Cmd(DMA1_Channel1, ENABLE); 	        //启动DMA通道
     ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//启动ADC转换


}

static int get_adc(Different_ADC_selection x)
{
    uint32_t hwSum = 0;
    uint32_t BASE  = 0;
    for(uint8_t chIndex = 0; chIndex < 8; chIndex++){
      uint32_t a=0;
      for(uint8_t i=0;i<8;i++)
      {
        
        a += AD_Sample_Value[i][x];
        
      }
        a /=8;
        hwSum += a;
    }
    
    BASE = D2MV(hwSum >> 3);
   // #ifdef __DEBUG__
         ///  printf("cur in base is %d\n", BASE);
  //  #endif
    return  BASE;
}

static float input_voltage(int x)
{
    float b=0;
    if(x < 550){
        
     b=0;
    } 
    else
      b=5.1*(x-500)/1000.0;
    #ifdef __DEBUG__
           printf("vol in base is %.2fV\n", b);
    #endif
    return  b;
        
}

static float input_current(int x)
{
    float b=0;  
    if(x < 530){
        
     b=0;
    } 
    else
    b=(x-500)/1000.0;
    #ifdef __DEBUG__
           printf("cur in base is %.2fA\n", b);
    #endif
    return  b;
        
}

static float output_voltage(int x)
{
    float b=0;
    if(x < 530){
        
     b=0;
    } 
    else
    b=3.0*(x-500)/1000.0;
    #ifdef __DEBUG__
              printf("vol out base is %.2fV\n", b);
    #endif
    return  b;
        
}

static float output_current(int x)
{
    float b=0;
    if(x < 1100){
        
     b=0;
    } 
    else
    b=(x-1000)/1000.0;
    #ifdef __DEBUG__
               printf("cur out base is %.2fA\n", b);
    #endif
    return  b;
        
}

static float output_current1(int x)
{
    float b=0;
    if(x < 1100){
        
     b=0;
    } 
    else
    b=(x-1000)/500.0;
    #ifdef __DEBUG__
               printf("cur out base is %.2fA\n", b);
    #endif
    return  b;
        
}

//if(tCwbViMsg.downselect == 1)
int cwb_test_busy1(void)
{   
     uint32_t cur_aver=1;
 
     if(tCwbViMsg.Volinset == 5.0 )
        cur_aver=1;
     else if(tCwbViMsg.Volinset == 12.0 )
        cur_aver=2;
     if((tWorkStatus1 != BUSY1) && (busyflag == 1)){
                     
          lxall_mdelay(1500);
          s_chCount1 = 0;
        
     }
     if(BUSY1 == tWorkStatus1){  
        if(ERROR_NO1 != tError1){//if any error before start, can not start test
            lx_can_a1();
            tWorkStatus1 = FAIL1;
	    s_chCount1 = 0;            
            power_on(a,OFF);   //产品断电
            lx_hub_load(a,OFF);
            tWorkMode1 = END1;
		#ifdef __DEBUG__
			printf("tError is %d\n", tError1);
		#endif
            return tError1;
        }
      
        led_off(LED_YELLOW1);
	#ifdef __DEBUG__
		printf("In busy! Current count1 is %d, all test time is %f\n", s_chCount1, tCwbViMsg.fTimesSet);
	#endif
        if(s_chCount1 < tCwbViMsg.fTimesSet*10){
            s_chCount1++;
            can_count1++;
            if(can_count1>=20)
            {
               lx_can_choice(CHANNEL_A1);
               lx1_mdelay(5);
               canerror1=canerror;
               lx_can_send();  
               can_count1=0;
            }
            //////////////////////////////////////////////////////
            if(tError1 != ERROR_NO1){
                lx_can_a1();
                tWorkStatus1 = FAIL1;
		s_chCount1 = 0;                
                power_on(a,OFF);   //产品断电
                lx_hub_load(a,OFF);
                tWorkMode1 = END1;               
                return tError1;
            } 
        } else {
            s_chCount1 = 0;  //reach the due to time
            tWorkMode1 = END1;
            power_on(a,OFF);   //产品断电
            lx_hub_load(a,OFF);
            if(tError1 != ERROR_NO1){
                tWorkStatus1 = FAIL1;
               
            } else {
                tWorkStatus1 = PASS1;
            }
            return tError1;
        }
        if(END1 == tWorkMode1) {
            lx_can_a1();
            tWorkStatus1 = FAIL1;
            s_chCount1 = 0;         
            power_on(a,OFF);   //产品断电
            lx_hub_load(a,OFF);
		#ifdef __DEBUG__
			printf("break END!\n");
		#endif
            return tError1;
        }
        power_on(a,ON);   //产品供电
        lx_hub_load(a,ON);//产品老化使能 
        lx1_mdelay(50);
        lx_set_current(CHANNEL_A1,tCwbViMsg.fCurOutSet);
        lx1_mdelay(450);
        
        tCwbViMsg1.fVinGet =input_voltage(get_adc(INUA)) ;
        tCwbViMsg1.fVoutGet=output_voltage(get_adc(OUTUA))+tCwbViMsg.downSetVol;
        tCwbViMsg1.fVoutGet1=0;
        tCwbViMsg1.fCinGet =input_current(get_adc(INIA));
        tCwbViMsg1.fCoutGet=output_current1(get_adc(OUTIA)) ;
        tCwbViMsg1.fCoutGet1=0;

          
           if(input_voltage(get_adc(INUA)) > (tCwbViMsg.Volinset+0.5)){
               tCwbViMsg.fCountGet11++;
               if(tCwbViMsg.fCountGet11 > tCwbViMsg.fCountVolLimit+1) {
                 printf("Error1 : DUT VOL IN HIGH1!\n");
                 lx_can_a1();
                 tError1=ERROR_VOL_IN_HIGH1; 
                 tWorkStatus1 = FAIL1;                 
                 power_on(a,OFF);   //产品断电
                 lx_hub_load(a,OFF);
                 s_chCount1 = 0;
                 return tError1;
               }
           }
           if(input_voltage(get_adc(INUA)) < (tCwbViMsg.Volinset-1.0)){
               tCwbViMsg.fCountGet11++;
               if(tCwbViMsg.fCountGet11 > tCwbViMsg.fCountVolLimit+1){
                  printf("Error1 :  short!\n");
                  lx_can_a1();
                  tError1 = ERROR_SHORT1;
                  tWorkStatus1 = FAIL1;               
                  power_on(a,OFF);   //产品断电
                  lx_hub_load(a,OFF);
                  s_chCount1 = 0;
                  //tWorkMode1 = END1;
                  return tError1; 
               }
           }
           if(output_voltage(get_adc(OUTUA))+tCwbViMsg.downSetVol < tCwbViMsg.downVolLimitL){ // get_adc(OUTUA)   filt(&vout1_filter_data,get_adc(OUTUA) );  
              // tCwbViMsg.fCountGet1 =0;
           
               tCwbViMsg.fCountGet12++;
               if(tCwbViMsg.fCountGet12 > tCwbViMsg.fCountVolLimit+1){
                   printf("Error1 : DUT VOL OUT LOW\n");
                   lx_can_a1();  
                   tError1 = ERROR_VOL_OUT_LOW1;                 
                   tWorkStatus1 = FAIL1;                  
                   power_on(a,OFF);   //产品断电
                   lx_hub_load(a,OFF);
                   s_chCount1 = 0;
                  // tWorkMode1 = END1;
                   return tError1; 
               }
           } 
           if(output_voltage(get_adc(OUTUA)) > tCwbViMsg.downVolLimitH){      
               tCwbViMsg.fCountGet12++;
               if(tCwbViMsg.fCountGet12 > tCwbViMsg.fCountVolLimit+1){
                   printf("Error1 : DUT VOL OUT HIGH\n");
                   lx_can_a1();
                   tError1 = ERROR_VOL_OUT_HIGH1;  
                   tWorkStatus1 = FAIL1;                 
                   power_on(a,OFF);   //产品断电
                   lx_hub_load(a,OFF);
                   s_chCount1 = 0;
                   //tWorkMode1 = END1;
                   return tError1; 
               }
           }
           if( input_current(get_adc(INIA)) <= (tCwbViMsg.fCurOutSet-0.15)/cur_aver){   
              // tCwbViMsg.fCountGet1 =0;
          
               tCwbViMsg.fCountGet13++;
               if(tCwbViMsg.fCountGet13 >= tCwbViMsg.fCountVolLimit+1){
                   printf("Error1 : DUT CUR IN LOW\n");
                   lx_can_a1();
                   tError1 = ERROR_CUR_IN1;  
                   tWorkStatus1 = FAIL1;                  
                   power_on(a,OFF);   //产品断电
                   lx_hub_load(a,OFF);
                   s_chCount1 = 0;
                  // tWorkMode1 = END1;
                   return tError1;
               }
           }
           if( input_current(get_adc(INIA)) >= (tCwbViMsg.fCurOutSet+1.2)/cur_aver){   
              // tCwbViMsg.fCountGet1 =0;
          
               tCwbViMsg.fCountGet13++;
               if(tCwbViMsg.fCountGet13 > tCwbViMsg.fCountVolLimit+1){
                   printf("Error1 : DUT CUR IN HIGH1\n");
                   lx_can_a1();
                   tError1 = ERROR_CUR_IN_HIGH1;  
                   tWorkStatus1 = FAIL1;                
                   power_on(a,OFF);   //产品断电
                   lx_hub_load(a,OFF);
                   s_chCount1 = 0;
                  // tWorkMode1 = END1;
                   return tError1;
               }
           }
           if( output_current1(get_adc(OUTIA)) <= (tCwbViMsg.fCurOutSet-0.2)){   
             //  tCwbViMsg.fCountGet1 =0;
          
               tCwbViMsg.fCountGet14++;
               if(tCwbViMsg.fCountGet14 > tCwbViMsg.fCountVolLimit+1){
                   printf("Error1 : DUT CUR OUT LOW\n");
                   lx_can_a1();
                   tError1 = ERROR_CUR_OUT1;  
                   tWorkStatus1 = FAIL1;                  
                   power_on(a,OFF);   //产品断电
                   lx_hub_load(a,OFF);
                   s_chCount1 = 0;
                  // tWorkMode1 = END1;
                   return tError1;
               }
           }
           if( output_current1(get_adc(OUTIA)) >= (tCwbViMsg.fCurOutSet+0.2)){   
             //  tCwbViMsg.fCountGet1 =0;
          
               tCwbViMsg.fCountGet14++;
               if(tCwbViMsg.fCountGet14 > tCwbViMsg.fCountVolLimit+1){
                   printf("Error1 : DUT CUR OUT HIGH\n");
                   lx_can_a1();
                   tError1 = ERROR_CUR_OUT_HIGH1;  
                   tWorkStatus1 = FAIL1;                 
                   power_on(a,OFF);   //产品断电
                   lx_hub_load(a,OFF);
                   s_chCount1 = 0;
                  // tWorkMode1 = END1;
                   return tError1;
               }
           }
           lx1_mdelay(1000);
           ///////////////////////////////////////////////////
         
        
    }
     if((tWorkStatus2 != BUSY2) && (busyflag == 1)){
                     
          lxall_mdelay(1500);
          s_chCount2 = 0;
         // if(tError1 == ERROR_NO1){
         // canerror1=canerror;/////////////////////////////////////////
         // }
     }
     if(BUSY2 == tWorkStatus2){ 
        if(ERROR_NO2 != tError2){//if any error before start, can not start test
            tWorkStatus2 = FAIL2;
	    s_chCount2 = 0;
            power_on(b,OFF);   //产品断电
            lx_hub_load(b,OFF);
            tWorkMode2 = END2;
		#ifdef __DEBUG__
			printf("tError is %d\n", tError2);
		#endif
            return tError2;
        }
      
        led_off(LED_YELLOW2);
	#ifdef __DEBUG__
		printf("In busy! Current count is %d, all test time is %f\n", s_chCount2, tCwbViMsg.fTimesSet);
	#endif
        if(s_chCount2 < tCwbViMsg.fTimesSet*10){
            s_chCount2++;
            if(tError2 != ERROR_NO2){
                tWorkStatus2 = FAIL2;
		s_chCount2 = 0;
                power_on(b,OFF);   //产品断电
                lx_hub_load(b,OFF);
                tWorkMode2 = END2;
                return tError2;
            } 
        } else {
            s_chCount2 = 0;  //reach the due to time
            tWorkMode2 = END2;
            power_on(b,OFF);   //产品断电
            lx_hub_load(b,OFF);
            if(tError2 != ERROR_NO2){
                tWorkStatus2= FAIL2;
            } else {
                tWorkStatus2 = PASS2;
            }
            return tError2;
        }
        if(END2 == tWorkMode2) {
       
            tWorkStatus2 = FAIL2;
            s_chCount2 = 0;
            power_on(b,OFF);   //产品断电
            lx_hub_load(b,OFF);
		#ifdef __DEBUG__
			printf("break END2!\n");
		#endif
            return tError2;
        }
        power_on(b,ON);   //产品供
        lx_hub_load(b,ON);//产品老化使能 
        lx2_mdelay(50);
        lx_set_current(CHANNEL_B1,tCwbViMsg.fCurOutSet);
        lx2_mdelay(450);
        
        tCwbViMsg2.fVinGet =input_voltage(get_adc(INUB)) ;
        tCwbViMsg2.fVoutGet=output_voltage(get_adc(OUTUB))+tCwbViMsg.downSetVol;
        tCwbViMsg2.fVoutGet1 =0;
        tCwbViMsg2.fCinGet =input_current(get_adc(INIB));
        tCwbViMsg2.fCoutGet=output_current(get_adc(OUTIB)) ;
        tCwbViMsg2.fCoutGet1 =0;

      
           if(input_voltage(get_adc(INUB)) > (tCwbViMsg.Volinset+0.5)){
               tCwbViMsg.fCountGet21++;
               if(tCwbViMsg.fCountGet21 >= tCwbViMsg.fCountVolLimit) {
                 printf("Error2 : DUT VOL IN HIGH2!\n");
                 tError2=ERROR_VOL_IN_HIGH2; 
                 tWorkStatus2 = FAIL2;
                 power_on(b,OFF);   //产品断电
                 lx_hub_load(b,OFF);
                 s_chCount2 = 0;
                // tWorkMode2 = END2;
                 return tError2;
               }
           }
           if(input_voltage(get_adc(INUB)) < (tCwbViMsg.Volinset-1.0)){
               tCwbViMsg.fCountGet21++;
               if(tCwbViMsg.fCountGet21 >= tCwbViMsg.fCountVolLimit){
                  printf("Error2 : DUT short!\n");
                  tError2 = ERROR_SHORT2;
                  tWorkStatus2 = FAIL2;
                  power_on(b,OFF);   //产品断电
                  lx_hub_load(b,OFF);
                  s_chCount2 = 0;
                  //tWorkMode2 = END2;
                  return tError2; 
               }
           }
           if(output_voltage(get_adc(OUTUB))+tCwbViMsg.downSetVol < tCwbViMsg.downVolLimitL ){      
              // tCwbViMsg.fCountGet2 =0;
          
               tCwbViMsg.fCountGet22++;
               if(tCwbViMsg.fCountGet22 >= tCwbViMsg.fCountVolLimit){
                   printf("Error2 : DUT VOL OUT LOW\n");
                   tError2 = ERROR_VOL_OUT_LOW2;  
                   tWorkStatus2 = FAIL2;
                    power_on(b,OFF);   //产品断电
                   lx_hub_load(b,OFF);
                   s_chCount2 = 0;
                   //tWorkMode2 = END2;
                   return tError2; 
               }
           } 
           if(output_voltage(get_adc(OUTUB)) > tCwbViMsg.downVolLimitH){      
               tCwbViMsg.fCountGet22++;
               if(tCwbViMsg.fCountGet22 >= tCwbViMsg.fCountVolLimit){
                   printf("Error2 : DUT VOL OUT HIGH\n");
                   tError2 = ERROR_VOL_OUT_HIGH2;  
                   tWorkStatus2 = FAIL2;
                   power_on(b,OFF);   //产品断电
                   lx_hub_load(b,OFF);
                   s_chCount2 = 0;
                   //tWorkMode2 = END2;
                   return tError2; 
               }
           }
           
           if( input_current(get_adc(INIB)) <= (tCwbViMsg.fCurOutSet-0.15)/cur_aver){   
              // tCwbViMsg.fCountGet2 =0;
             
          
               tCwbViMsg.fCountGet23++;
               if(tCwbViMsg.fCountGet23 >= tCwbViMsg.fCountVolLimit){
                   printf("Error2 : DUT CUR IN LOW\n");
                   tError2 = ERROR_CUR_IN2;  
                   tWorkStatus2 = FAIL2;
                   power_on(b,OFF);   //产品断电
                   lx_hub_load(b,OFF);
                   s_chCount2 = 0;
                   //tWorkMode2 = END2;
                   return tError2;
               }
           }
           if( input_current(get_adc(INIB)) >= (tCwbViMsg.fCurOutSet+0.15)/cur_aver){   
              // tCwbViMsg.fCountGet2 =0;
             
          
               tCwbViMsg.fCountGet23++;
               if(tCwbViMsg.fCountGet23 >= tCwbViMsg.fCountVolLimit){
                   printf("Error2 : DUT CUR IN HIGH2\n");
                   tError2 = ERROR_CUR_IN_HIGH2;  
                   tWorkStatus2 = FAIL2;
                   power_on(b,OFF);   //产品断电
                   lx_hub_load(b,OFF);
                   s_chCount2 = 0;
                   //tWorkMode2 = END2;
                   return tError2;
               }
           }
           if( output_current(get_adc(OUTIB)) <= (tCwbViMsg.fCurOutSet-0.1)){   
              // tCwbViMsg.fCountGet2 =0;
             
         
               tCwbViMsg.fCountGet24++;
               if(tCwbViMsg.fCountGet24 >= tCwbViMsg.fCountVolLimit){
                   printf("Error2 : DUT CUR OUT LOW\n");
                   tError2 = ERROR_CUR_OUT2;  
                   tWorkStatus2 = FAIL2;
                   power_on(b,OFF);   //产品断电
                   lx_hub_load(b,OFF);
                   s_chCount2 = 0;
                   //tWorkMode2 = END2;
                   return tError2;
               }
           }
           if( output_current(get_adc(OUTIB)) >= (tCwbViMsg.fCurOutSet+0.2)){   
              // tCwbViMsg.fCountGet2 =0;
             
         
               tCwbViMsg.fCountGet24++;
               if(tCwbViMsg.fCountGet24 >= tCwbViMsg.fCountVolLimit){
                   printf("Error2 : DUT CUR OUT HIGH\n");
                   tError2 = ERROR_CUR_OUT_HIGH2;  
                   tWorkStatus2 = FAIL2;
                   power_on(b,OFF);   //产品断电
                   lx_hub_load(b,OFF);
                   s_chCount2 = 0;
                   //tWorkMode2 = END2;
                   return tError2;
               }
           }
   
        
        lx2_mdelay(1000);

        
        
    }
     if((BUSY3 != tWorkStatus3) && (busyflag == 1)){
                    
        lxall_mdelay(1500);
        s_chCount3 = 0;
    }  
     if(BUSY3 == tWorkStatus3){ 
        if(ERROR_NO3 != tError3){//if any error before start, can not start test
            lx_can_c1();
            tWorkStatus3 = FAIL3;
	    s_chCount3 = 0;           
            power_on(c,OFF);   //产品断电
            lx_hub_load(c,OFF);
            tWorkMode3 = END3;
		#ifdef __DEBUG__
			printf("tError is %d\n", tError3);
		#endif
            return tError3;
        }
      
        led_off(LED_YELLOW3);
	#ifdef __DEBUG__
		printf("In busy! Current count3 is %d, all test time is %f\n", s_chCount3, tCwbViMsg.fTimesSet);
	#endif
        if(s_chCount3 < tCwbViMsg.fTimesSet*10){
            s_chCount3++;
            can_count3++;
             if(can_count3>=25)
            {
               lx_can_choice(CHANNEL_C1);
               lx3_mdelay(5);
               canerror3=canerror;
               lx_can_send();
               can_count3=0;
            }
            if(tError3 != ERROR_NO3){
                lx_can_c1();
                tWorkStatus3 = FAIL3;
		s_chCount3 = 0;
                tWorkMode3 = END3;               
                power_on(c,OFF);   //产品断电
                lx_hub_load(c,OFF);
                return tError3;
            } 
        } else {
            s_chCount3 = 0;  //reach the due to time
            tWorkMode3 = END3;
            power_on(c,OFF);   //产品断电
            lx_hub_load(c,OFF);
            if(tError3 != ERROR_NO3){
                tWorkStatus3 = FAIL3;
            } else {
                tWorkStatus3 = PASS3;
            }
            return tError3;
        }
        if(END3 == tWorkMode3) {
            lx_can_c1();
            tWorkStatus3 = FAIL3;
            s_chCount3 = 0;
            
            power_on(c,OFF);   //产品断电
            lx_hub_load(c,OFF);
		#ifdef __DEBUG__
			printf("break END3!\n");
		#endif
            return tError3;
        }
        power_on(c,ON);   //产品供电
        lx_hub_load(c,ON);//产品老化使能 
        lx3_mdelay(50);
        lx_set_current(CHANNEL_C1,tCwbViMsg.fCurOutSet);
        lx3_mdelay(450);
        
        tCwbViMsg3.fVinGet =input_voltage(get_adc(INUC)) ;
        tCwbViMsg3.fVoutGet=output_voltage(get_adc(OUTUC))+tCwbViMsg.downSetVol;
        tCwbViMsg3.fVoutGet1=0;
        tCwbViMsg3.fCinGet =input_current(get_adc(INIC));
        tCwbViMsg3.fCoutGet=output_current1(get_adc(OUTIC)) ;
        tCwbViMsg3.fCoutGet1=0;


      
           if(input_voltage(get_adc(INUC)) > (tCwbViMsg.Volinset+0.5)){
               tCwbViMsg.fCountGet31++;
               if(tCwbViMsg.fCountGet31 >= tCwbViMsg.fCountVolLimit) {
                 printf("Error3 : DUT VOL IN HIGH3!\n");
                 lx_can_c1();
                 tError3=ERROR_VOL_IN_HIGH3; 
                 tWorkStatus3 = FAIL3;                
                 power_on(c,OFF);   //产品断电
                 lx_hub_load(c,OFF);
                 s_chCount3 = 0;
                // tWorkMode3 = END3;
                 return tError3;
               }
           }
           if(input_voltage(get_adc(INUC)) < (tCwbViMsg.Volinset-1.0)){
               tCwbViMsg.fCountGet31++;
               if(tCwbViMsg.fCountGet31 >= tCwbViMsg.fCountVolLimit){
                  printf("Error3 : DUT short!\n");
                  lx_can_c1();
                  tError3 = ERROR_SHORT3;
                  tWorkStatus3 = FAIL3;
                  power_on(c,OFF);   //产品断电
                  lx_hub_load(c,OFF);
                  s_chCount3 = 0;
                 // tWorkMode3 = END3;
                  return tError3; 
               }
           }
           if(output_voltage(get_adc(OUTUC))+tCwbViMsg.downSetVol < tCwbViMsg.downVolLimitL ){      
          
               tCwbViMsg.fCountGet32++;
               if(tCwbViMsg.fCountGet32 >= tCwbViMsg.fCountVolLimit){
                   printf("Error3 : DUT VOL OUT LOW\n");
                   lx_can_c1();
                   tError3 = ERROR_VOL_OUT_LOW3;  
                   tWorkStatus3 = FAIL3;                  
                   power_on(c,OFF);   //产品断电
                   lx_hub_load(c,OFF);
                   s_chCount3 = 0;
                  // tWorkMode3 = END3;
                   return tError3; 
               }
           } 
           if(output_voltage(get_adc(OUTUC)) > tCwbViMsg.downVolLimitH){      
               tCwbViMsg.fCountGet32++;
               if(tCwbViMsg.fCountGet32 >= tCwbViMsg.fCountVolLimit){
                   printf("Error3 : DUT VOL OUT HIGH\n");
                   lx_can_c1();
                   tError3 = ERROR_VOL_OUT_HIGH3;  
                   tWorkStatus3 = FAIL3;                   
                   power_on(c,OFF);   //产品断电
                   lx_hub_load(c,OFF);
                   s_chCount3 = 0;
                  // tWorkMode3 = END3;
                   return tError3; 
               }
           }
           if( input_current(get_adc(INIC)) <= (tCwbViMsg.fCurOutSet-0.15)/cur_aver){   
              // tCwbViMsg.fCountGet3 =0;
             
          
               tCwbViMsg.fCountGet33++;
               if(tCwbViMsg.fCountGet33 >= tCwbViMsg.fCountVolLimit){
                   printf("Error3 : DUT CUR IN LOW\n");
                   lx_can_c1();
                   tError3 = ERROR_CUR_IN3;  
                   tWorkStatus3 = FAIL3;                  
                   power_on(c,OFF);   //产品断电
                   lx_hub_load(c,OFF);
                   s_chCount3 = 0;
                   //tWorkMode3 = END3;
                   return tError3;
               }
           }
           if( input_current(get_adc(INIC)) >= (tCwbViMsg.fCurOutSet+1.2)/cur_aver){   
              // tCwbViMsg.fCountGet3 =0;
             
          
               tCwbViMsg.fCountGet33++;
               if(tCwbViMsg.fCountGet33 >= tCwbViMsg.fCountVolLimit){
                   printf("Error3 : DUT CUR IN HIHG3\n");
                   lx_can_c1();
                   tError3 = ERROR_CUR_IN_HIGH3;  
                   tWorkStatus3 = FAIL3;
                   power_on(c,OFF);   //产品断电
                   lx_hub_load(c,OFF);
                   s_chCount3 = 0;
                   //tWorkMode3 = END3;
                   return tError3;
               }
           }
           if( output_current1(get_adc(OUTIC)) <= (tCwbViMsg.fCurOutSet-0.2)){   
              // tCwbViMsg.fCountGet3 =0;
         
               tCwbViMsg.fCountGet34++;
               if(tCwbViMsg.fCountGet34 >= tCwbViMsg.fCountVolLimit){
                   printf("Error3 : DUT CUR OUT LOW\n");
                   lx_can_c1();
                   tError3 = ERROR_CUR_OUT3;  
                   tWorkStatus3 = FAIL3;                  
                   power_on(c,OFF);   //产品断电
                   lx_hub_load(c,OFF);
                   s_chCount3 = 0;
                   //tWorkMode3 = END3;
                   return tError3;
               }
           }
           if( output_current1(get_adc(OUTIC)) >= (tCwbViMsg.fCurOutSet+0.2)){   
              // tCwbViMsg.fCountGet3 =0;
         
               tCwbViMsg.fCountGet34++;
               if(tCwbViMsg.fCountGet34 >= tCwbViMsg.fCountVolLimit){
                   printf("Error3 : DUT CUR OUT HIGH\n");
                   lx_can_c1();
                   tError3 = ERROR_CUR_OUT_HIGH3;  
                   tWorkStatus3 = FAIL3;
                   
                   power_on(c,OFF);   //产品断电
                   lx_hub_load(c,OFF);
                   s_chCount3 = 0;
                   //tWorkMode3 = END3;
                   return tError3;
               }
           }
       
        
       lx3_mdelay(1000);
       //////////////////////////////////////////////////////////////
     
        
    }
     if((tWorkStatus4 != BUSY4) && (busyflag == 1)){       
                    
          lxall_mdelay(1500);
          s_chCount4 = 0;
          
      } 
     if(BUSY4 == tWorkStatus4){ 
        if(ERROR_NO4 != tError4){//if any error before start, can not start test
            tWorkStatus4 = FAIL4;
	    s_chCount4 = 0;
            power_on(d,OFF);   //产品断电
            lx_hub_load(d,OFF);
            tWorkMode4 = END4;
		#ifdef __DEBUG__
			printf("tError is %d\n", tError4);
		#endif
            return tError4;
        }
      
        led_off(LED_YELLOW4);
	#ifdef __DEBUG__
		printf("In busy! Current count is %d, all test time is %f\n", s_chCount4, tCwbViMsg.fTimesSet);
	#endif
        if(s_chCount4 < tCwbViMsg.fTimesSet*10){
            s_chCount4++;
            if(tError4 != ERROR_NO4){
                tWorkStatus4 = FAIL4;
		s_chCount4 = 0;
                power_on(d,OFF);   //产品断电
                lx_hub_load(d,OFF);
                tWorkMode4 = END4;
                return tError4;
            } 
        } else {
            s_chCount4 = 0;  //reach the due to time
            tWorkMode4 = END4;
            power_on(d,OFF);   //产品断电
            lx_hub_load(d,OFF);
            if(tError4 != ERROR_NO4){
                tWorkStatus4 = FAIL4;
            } else {
                tWorkStatus4 = PASS4;
            }
            return tError4;
        }
        if(END4 == tWorkMode4) {
       
            tWorkStatus4 = FAIL4;
            s_chCount4 = 0;
            power_on(d,OFF);   //产品断电
            lx_hub_load(d,OFF);
		#ifdef __DEBUG__
			printf("break END4!\n");
		#endif
            return tError4;
        }
        power_on(d,ON);   //产品供电
        lx_hub_load(d,ON);//产品老化使能  
        lx4_mdelay(50);
        lx_set_current(CHANNEL_D1,tCwbViMsg.fCurOutSet);
        lx4_mdelay(450);
        
        tCwbViMsg4.fVinGet =input_voltage(get_adc(INUD)) ;
        tCwbViMsg4.fVoutGet=output_voltage(get_adc(OUTUD))+tCwbViMsg.downSetVol;
        tCwbViMsg4.fVoutGet1 = 0;
        tCwbViMsg4.fCinGet =input_current(get_adc(INID));
        tCwbViMsg4.fCoutGet=output_current(get_adc(OUTID)) ;
        tCwbViMsg4.fCoutGet1 = 0;

        if(input_voltage(get_adc(INUD)) > (tCwbViMsg.Volinset+0.5)){
               tCwbViMsg.fCountGet41++;
               if(tCwbViMsg.fCountGet41 >= tCwbViMsg.fCountVolLimit) {
                 printf("Error4 : DUT VOL IN HIGH1!\n");
                 tError4=ERROR_VOL_IN_HIGH4; 
                 tWorkStatus4 = FAIL4;
                 power_on(d,OFF);   //产品断电
                 lx_hub_load(d,OFF);
                 s_chCount4 = 0;
                 //tWorkMode4 = END4;
                 return tError4;
               }
         }
        if(input_voltage(get_adc(INUD)) < (tCwbViMsg.Volinset-1.0)){
               tCwbViMsg.fCountGet41++;
               if(tCwbViMsg.fCountGet41 >= tCwbViMsg.fCountVolLimit){
                  printf("Error4 : DUT short!\n");
                  tError4 = ERROR_SHORT4;
                  tWorkStatus4 = FAIL4;
                  power_on(d,OFF);   //产品断电
                  lx_hub_load(d,OFF);
                  s_chCount4 = 0;
                 // tWorkMode4 = END4;
                  return tError4; 
               }
         }
        if(output_voltage(get_adc(OUTUD))+tCwbViMsg.downSetVol < tCwbViMsg.downVolLimitL ){      
             //  tCwbViMsg.fCountGet4 =0;
       
               tCwbViMsg.fCountGet42++;
               if(tCwbViMsg.fCountGet42 >= tCwbViMsg.fCountVolLimit){
                   printf("Error4 : DUT VOL OUT LOW\n");
                   tError4 = ERROR_VOL_OUT_LOW4;  
                   tWorkStatus4 = FAIL4;
                   power_on(d,OFF);   //产品断电
                   lx_hub_load(d,OFF);
                   s_chCount4 = 0;
                   //tWorkMode4 = END4;
                   return tError4; 
               }
          } 
         if(output_voltage(get_adc(OUTUD)) > tCwbViMsg.downVolLimitH){      
               tCwbViMsg.fCountGet42++;
               if(tCwbViMsg.fCountGet42 >= tCwbViMsg.fCountVolLimit){
                   printf("Error4 : DUT VOL OUT HIGH\n");
                   tError4 = ERROR_VOL_OUT_HIGH4;  
                   tWorkStatus4 = FAIL4;
                   power_on(d,OFF);   //产品断电
                   lx_hub_load(d,OFF);
                   s_chCount4 = 0;
                   //tWorkMode4 = END4;
                   return tError4; 
               }
           }
          if( input_current(get_adc(INID)) <= (tCwbViMsg.fCurOutSet-0.15)/cur_aver){   
             //  tCwbViMsg.fCountGet4 =0;
             
               tCwbViMsg.fCountGet43++;
               if(tCwbViMsg.fCountGet43 >= tCwbViMsg.fCountVolLimit){
                   printf("Error4 : DUT CUR IN LOW\n");
                   tError4 = ERROR_CUR_IN4;  
                   tWorkStatus4 = FAIL4;
                   power_on(d,OFF);   //产品断电
                   lx_hub_load(d,OFF);
                   s_chCount4 = 0;
                   //tWorkMode4 = END4;
                   return tError4;
               }
          }
          if( input_current(get_adc(INID)) >= (tCwbViMsg.fCurOutSet+0.15)/cur_aver){   
             //  tCwbViMsg.fCountGet4 =0;
             
               tCwbViMsg.fCountGet43++;
               if(tCwbViMsg.fCountGet43 >= tCwbViMsg.fCountVolLimit){
                   printf("Error4 : DUT CUR IN HIGH4\n");
                   tError4 = ERROR_CUR_IN_HIGH4;  
                   tWorkStatus4 = FAIL4;
                   power_on(d,OFF);   //产品断电
                   lx_hub_load(d,OFF);
                   s_chCount4 = 0;
                   //tWorkMode4 = END4;
                   return tError4;
               }
          }
          if( output_current(get_adc(OUTID)) <= (tCwbViMsg.fCurOutSet-0.1)){   
             //  tCwbViMsg.fCountGet4 =0;
        
               tCwbViMsg.fCountGet44++;
               if(tCwbViMsg.fCountGet44 >= tCwbViMsg.fCountVolLimit){
                   printf("Error4 : DUT CUR OUT LOW\n");
                   tError4 = ERROR_CUR_OUT4;  
                   tWorkStatus4 = FAIL4;
                   power_on(d,OFF);   //产品断电
                   lx_hub_load(d,OFF);
                   s_chCount4 = 0;
                   //tWorkMode4 = END4;
                   return tError4;
               }
           }
           if( output_current(get_adc(OUTID)) >= (tCwbViMsg.fCurOutSet+0.2)){   
               tCwbViMsg.fCountGet44++;
               if(tCwbViMsg.fCountGet44 >= tCwbViMsg.fCountVolLimit){
                   printf("Error4 : DUT CUR OUT HIGH\n");
                   tError4 = ERROR_CUR_OUT_HIGH4;  
                   tWorkStatus4 = FAIL4;
                   power_on(d,OFF);   //产品断电
                   lx_hub_load(d,OFF);
                   s_chCount4 = 0;
                   //tWorkMode4 = END4;
                   return tError4;
               }
           }  
        lx4_mdelay(1000);       
    }
      
    

    return tError1; 

}



int cwb_test_busy(void)
{ 
   if(tCwbViMsg.downselect == 1)  
   { 
     cwb_test_busy1();
     
   }
   else if(tCwbViMsg.downselect == 2)
   { 
     // cwb_test_busy2();
     
   }
     return 0;
   
}
int lx_can_send(void)
{
  if(tCwbViMsg.productselect==0){
    for(int i=0;i<3 ;i++){
       int s=0;
       char ax1[]="can once send 535 00 00 00 00 00 80 00 00 on";
       s=strlen(ax1); 
       #ifdef __DEBUG__
       printf("%d\r\n",s); 
       #endif
       for(int i=0;i<s;i++)                 
          uart3.putchar(ax1[i]);
       uart3.putchar('\r');
       cwb_mdelay(20);
       char ax2[]="can once send 325 00 00 00 00 00 02 00 00 on";                 
       s=strlen(ax2);                     
       for(int i=0;i<s;i++)                 
          uart3.putchar(ax2[i]);
       uart3.putchar('\r');
       cwb_mdelay(20);
       char ax3[]="can once send 310 00 00 00 00 80 00 00 00 on";                 
       s=strlen(ax3);   
       for(int i=0;i<s;i++)                 
          uart3.putchar(ax3[i]);
       uart3.putchar('\r');  
       cwb_mdelay(20);
       char ax4[]="can once send 311 00 00 00 00 00 02 00 00 on";;                 
       s=strlen(ax4);    
       for(int i=0;i<s;i++)                 
          uart3.putchar(ax4[i]);
       uart3.putchar('\r');                     
       cwb_mdelay(20);
       
      }
  }else if(tCwbViMsg.productselect==1){
    for(int i=0;i<3 ;i++){
       int s=0;
       char ax1[]="can once send 3AA 02 00 00 00 40 00 00 00 on";
       s=strlen(ax1); 
       #ifdef __DEBUG__
       printf("%d\r\n",s); 
       #endif
       for(int i=0;i<s;i++)                 
          uart3.putchar(ax1[i]);
       uart3.putchar('\r');
       cwb_mdelay(20);
      
      }
  }
  return 1;
}
void lx_can_choice(int x)
{
     switch(x) { 
                case CHANNEL_A1: 
                        GPIO_WriteBit(GPIOF, GPIO_Pin_12, Bit_RESET);  
                        break; 
                case CHANNEL_C1: 
                        GPIO_WriteBit(GPIOF, GPIO_Pin_12, Bit_SET); 
                        break; 
                default: 
                        return; 
                } 

}
int cwb_test_end(void)
{
  if(END == tWorkMode){
       if((END1 == tWorkMode1) && (END2 == tWorkMode2) && (END3 == tWorkMode3) && (END4 == tWorkMode4)){
        // lx_set_current(0);//电流设为零
       }
       if(END1 == tWorkMode1){
        
       
     
         if(FAIL1 == tWorkStatus1) { 
            led_off(LED_YELLOW1); //must add, else red will light continue
            led_on(LED_RED1);
            cwb_mdelay(1);
         } else if(PASS1 == tWorkStatus1) {
            led_off(LED_YELLOW1);
            led_on(LED_GREEN1);
            cwb_mdelay(1);
         } 
      }
      if(END2 == tWorkMode2) {
       
         if(FAIL2 == tWorkStatus2) { 
            led_off(LED_YELLOW2); //must add, else red will light continue
            led_on(LED_RED2);
            cwb_mdelay(1);
         } else if(PASS2 == tWorkStatus2) {
            led_off(LED_YELLOW2);
            led_on(LED_GREEN2);
            cwb_mdelay(1);
         } 
      }
      if(END3 == tWorkMode3) {
       
       
         if(FAIL3 == tWorkStatus3) { 
            led_off(LED_YELLOW3); //must add, else red will light continue
            led_on(LED_RED3);
            cwb_mdelay(1);
         } else if(PASS3 == tWorkStatus3) {
            led_off(LED_YELLOW3);
            led_on(LED_GREEN3);
            cwb_mdelay(1);
         } 
      }
      if(END4 == tWorkMode4) {
        
       
         if(FAIL4 == tWorkStatus4) { 
            led_off(LED_YELLOW4); //must add, else red will light continue
            led_on(LED_RED4);
            cwb_mdelay(1);
         } else if(PASS4 == tWorkStatus4) {
            led_off(LED_YELLOW4);
            led_on(LED_GREEN4);
            cwb_mdelay(1);
         } 

      }    
  }
  return tError1;
}
//
bool cwb_mode_poll(void)
{
   if(POLL == tWorkMode) {
       
        tWorkMode = START;
        if(POLL1 == tWorkMode1) {
            tWorkMode1 = START1;//*********注意***********
            #ifdef __DEBUG__
                    printf("In poll!\r\n");
                    printf("Error1 code is %d\r\n", tError1);
            #endif
            tCwbCanTx1.id = tCwbCanRx.id;
            tCwbCanTx1.data[0] = (uint8_t)tError1;
            tCwbCanTx1.data[2] = (uint8_t)tWorkStatus1;
            tCwbCanTx1.data[3]=  (uint8_t)canerror1;
            cwb_can_write(&tCwbCanTx1);         
             if(FAIL1 == tWorkStatus1) {
                led_off(LED_YELLOW1);
                led_on(LED_RED1);
                cwb_mdelay(1);
            } else if(PASS1 == tWorkStatus1) {
                led_off(LED_YELLOW1);
                led_on(LED_GREEN1);
                cwb_mdelay(1);
            } else {
               
            }
        }else if(POLL2 == tWorkMode2) {
            tWorkMode2 = START2;//*********注意***********
            #ifdef __DEBUG__
                    printf("In poll!\r\n");
                    printf("Error2 code is %d\r\n", tError2);
            #endif
            tCwbCanTx2.id = tCwbCanRx.id;
            tCwbCanTx2.data[0] = (uint8_t)tError2;
            tCwbCanTx2.data[2] = (uint8_t)tWorkStatus2;
            cwb_can_write(&tCwbCanTx2);         
             if(FAIL2 == tWorkStatus2) {
                led_off(LED_YELLOW2);
                led_on(LED_RED2);
                cwb_mdelay(1);
            } else if(PASS2 == tWorkStatus2) {
                led_off(LED_YELLOW2);
                led_on(LED_GREEN2);
                cwb_mdelay(1);
            } else {
               
            }
        }else if(POLL3 == tWorkMode3) {
            tWorkMode3 = START3;//*********注意***********
            #ifdef __DEBUG__
                    printf("In poll!\r\n");
                    printf("Error3 code is %d\r\n", tError3);
            #endif
            tCwbCanTx3.id = tCwbCanRx.id;
            tCwbCanTx3.data[0] = (uint8_t)tError3;
            tCwbCanTx3.data[2] = (uint8_t)tWorkStatus3;
            tCwbCanTx3.data[3]=  (uint8_t)canerror3;
            cwb_can_write(&tCwbCanTx3);         
            if(FAIL3== tWorkStatus3){
                led_off(LED_YELLOW3);
                led_on(LED_RED3);
                cwb_mdelay(1);
            } else if(PASS3 == tWorkStatus3) {
                led_off(LED_YELLOW3);
                led_on(LED_GREEN3);
                cwb_mdelay(1);
            } else {
               
            }
        }else if(POLL4 == tWorkMode4) {
            tWorkMode4 = START4;//*********注意***********
            #ifdef __DEBUG__
                    printf("In poll!\r\n");
                    printf("Error4 code is %d\r\n", tError4);
            #endif
            tCwbCanTx4.id = tCwbCanRx.id;
            tCwbCanTx4.data[0] = (uint8_t)tError4;
            tCwbCanTx4.data[2] = (uint8_t)tWorkStatus4;
            cwb_can_write(&tCwbCanTx4);         
            if(FAIL4== tWorkStatus4){
                led_off(LED_YELLOW4);
                led_on(LED_RED4);
                cwb_mdelay(1);
            } else if(PASS4 == tWorkStatus4) {
                led_off(LED_YELLOW4);
                led_on(LED_GREEN4);
                cwb_mdelay(1);
            } else {
               
            }
        }
   }
    return TRUE;
}
//
bool cwb_mode_cfg(void)
{
    if(CFG == tWorkMode) {
        printf("In cfg!\r\n");
         printf("The chNumCfg message is %d\n", chNumCfg);
        if((chNumCfgTotal + 1) == chNumCfg) { //recv over 12 bytes; 1 -> c 
           
            nvm_save();
            chNumCfg = 0;
            if(can_to_vi(&tCwbViMsg, tWorkMode)) { //maybe cfg error
                if((ERROR_CFG1 != tError1) && (ERROR_CFG2 != tError2) && (ERROR_CFG3 != tError3) && (ERROR_CFG4 != tError4) ) {
                    tCwbCanTx.data[0] = 0x1;
                    nvm_save();
                    printf("no error\n");
                }
                else {
                    tCwbCanTx.data[0] = 0x2; //cfg error
                    printf(" error\n");
                }

            }
            tCwbCanTx.id = tCwbCanRx.id;
            cwb_can_write(&tCwbCanTx);
        }
        tWorkMode = START;
        tWorkMode1 = START1;
        tWorkMode2 = START2;
        tWorkMode3 = START3;
        tWorkMode4 = START4;
    }
    
    return TRUE;
}
//
bool cwb_mode_read(void)
{
   if(READ == tWorkMode) {
      if(READ1 == tWorkMode1){
        tWorkMode1 = START1;
        vi_to_can(&tCwbViMsg1, READ);
        cwb_can_write(&tCwbCanTx);    //no checksum
        cwb_mdelay(2);
        return TRUE;
       
       }
       else if(READ2 == tWorkMode2){
        tWorkMode2 = START2;
        vi_to_can(&tCwbViMsg2, READ);
        cwb_can_write(&tCwbCanTx);    //no checksum
        cwb_mdelay(2);
        return TRUE;
       
       }else if(READ3 == tWorkMode3){
        tWorkMode3 = START3;
        vi_to_can(&tCwbViMsg3, READ);
        cwb_can_write(&tCwbCanTx);    //no checksum
        cwb_mdelay(2);
        return TRUE;
       
       }else if(READ4 == tWorkMode4){
        tWorkMode4 = START4;
        vi_to_can(&tCwbViMsg4, READ);
        cwb_can_write(&tCwbCanTx);    //no checksum
        cwb_mdelay(2);
        return TRUE;
       
       }
    
   }
   return TRUE;
}
//
bool cwb_mode_fault(void)
{
    if(FAULT == tWorkMode) {
	tWorkMode = START;
        if(FAULT1 == tWorkMode1) {
          #ifdef __DEBUG__
          printf("Error code is %d\r\n", tCwbCanRx.data[1]);
          #endif
          if( tCwbCanRx.data[1] == 1){
             tWorkMode1 = START1;
             led_off(LED_YELLOW1);
             return TRUE;
          }else if( tCwbCanRx.data[1] == 2){
             tWorkMode1 = START1;
             led_on(LED_GREEN1);
             return TRUE;
          }   
        }else if(FAULT2 == tWorkMode2){
          if( tCwbCanRx.data[1] == 1){
             tWorkMode2 = START2;
             led_off(LED_YELLOW2);
             return TRUE;
          }else if( tCwbCanRx.data[1] == 2){
             tWorkMode2 = START2;
             led_on(LED_GREEN2);
             return TRUE;
          }   
        }else if(FAULT3 == tWorkMode3){
           if( tCwbCanRx.data[1] == 1){
             tWorkMode3 = START3;
             led_off(LED_YELLOW3);
             return TRUE;
           }else if( tCwbCanRx.data[1] == 2){
             tWorkMode3 = START3;
             led_on(LED_GREEN3);
             return TRUE;
           }   
        }else if(FAULT4 == tWorkMode4){
           if( tCwbCanRx.data[1] == 1){
             tWorkMode4 = START4;
             led_off(LED_YELLOW4);
             return TRUE;
           }else if( tCwbCanRx.data[1] == 2){
             tWorkMode4 = START4;
             led_on(LED_GREEN4);
             return TRUE;
          }   
        }  
    }
    
    return TRUE;
}
//step1 set the spv
bool cwb_mode_hand1(void)
{
    if(HAND1 == tWorkMode) {
      
      
	#ifdef __DEBUG__
		printf("In hand1, set power 12.5v, wait for check voltage!\n");
	#endif
         tWorkMode = START;  
         
	if(tCwbViMsg.Volinset == 5.0) {
             power_choose(VCC_INL);
             if(HAND11 == tWorkMode1){
                
                tWorkMode1 = START1;
                power_on(a,ON);  
                tCwbCanTx1.id = tCwbCanRx.id;
                tCwbCanTx1.data[0] = 1;
                cwb_can_write(&tCwbCanTx1);    //no checksum
                cwb_mdelay(2);
                ACC_choose(ON1);           
             }else if(HAND12 == tWorkMode2){
                tWorkMode2 = START2;
                tCwbCanTx2.id = tCwbCanRx.id;
                tCwbCanTx2.data[0] = 1;
                cwb_can_write(&tCwbCanTx2);    //no checksum
                cwb_mdelay(2);
                power_on(b,ON);
             }else if(HAND13 == tWorkMode3){
                
                tWorkMode3 = START3;
                power_on(c,ON);
                tCwbCanTx3.id = tCwbCanRx.id;
                tCwbCanTx3.data[0] = 1;
                cwb_can_write(&tCwbCanTx3);    //no checksum
                cwb_mdelay(2);
                ACC_choose(ON2);
             }else if(HAND14 == tWorkMode4){
                tWorkMode4 = START4; 
                tCwbCanTx4.id = tCwbCanRx.id;
                tCwbCanTx4.data[0] = 1;
                cwb_can_write(&tCwbCanTx4);    //no checksum
                cwb_mdelay(2);
                power_on(d,ON);
             }
        } 
        else if(tCwbViMsg.Volinset == 12.0){
             power_choose(VCC_INH);
            if(HAND11 == tWorkMode1){
                printf("In hand1 power\n");
                tWorkMode1 = START1;
                power_on(a,ON);
                #ifdef __DEBUG__
                printf("power on1!\n");
                #endif
                tCwbCanTx1.id = tCwbCanRx.id;
                tCwbCanTx1.data[0] = 1;
                cwb_can_write(&tCwbCanTx1);    //no checksum
                cwb_mdelay(2);
                
                ACC_choose(ON1);  
             }else if(HAND12 == tWorkMode2){
                tWorkMode2 = START2;               
                tCwbCanTx2.id = tCwbCanRx.id;
                tCwbCanTx2.data[0] = 1;
                cwb_can_write(&tCwbCanTx2);    //no checksum
                cwb_mdelay(2);
                power_on(b,ON);
                
             }else if(HAND13 == tWorkMode3){
                printf("In hand3 power\n");
                tWorkMode3 = START3;
                power_on(c,ON);                
                tCwbCanTx3.id = tCwbCanRx.id;                
                tCwbCanTx3.data[0] = 1;
                cwb_can_write(&tCwbCanTx3);    //no checksum
                cwb_mdelay(2);               
                ACC_choose(ON2);  
             }else if(HAND14 == tWorkMode4){
                tWorkMode4 = START4;
                #ifdef __DEBUG__
                printf("power on4!\n");
                #endif
                tCwbCanTx4.id = tCwbCanRx.id;
                tCwbCanTx4.data[0] = 1;
                cwb_can_write(&tCwbCanTx4);    //no checksum
                cwb_mdelay(2);
                power_on(d,ON);
                
             }
             
        }       
        else {
             return FALSE;
        }

     
        
    }

    return TRUE;
}
//step2 judge target
bool cwb_mode_hand(void)
{     if(HAND == tWorkMode) {
        
        tWorkMode = START;
             
        if(HAND01 == tWorkMode1){
          printf("In wakeing1\n");
          tWorkStatus1 = IDLE1;//清除上一次的状态
          tWorkMode1 = START1;
          lx_can_choice(CHANNEL_A1);
          
          cwb_mdelay(100);
          
          if(lx_can_send()){
            tCwbCanTx1.data[0] = 1;
          } else {
            tCwbCanTx1.data[0] = 2;
            led_on(LED_RED1);
          }          
          
          tCwbCanTx1.id = tCwbCanRx.id;
          cwb_can_write(&tCwbCanTx1);    
          cwb_mdelay(2);         
          
           return TRUE;
        }
        else if(HAND02 == tWorkMode2){
           
           tWorkStatus2 = IDLE2; //清除上一次的状态
           tWorkMode2 = START2;
           
          if(lx_can_send()){
            tCwbCanTx2.data[0] = 1;
          } else {
            tCwbCanTx2.data[0] = 2;
            led_on(LED_RED2);
          }
        
          tCwbCanTx2.id = tCwbCanRx.id;
          cwb_can_write(&tCwbCanTx2);    
          cwb_mdelay(2);
          
          return TRUE;
        }
        else if(HAND03 == tWorkMode3){
          printf("In wakeing3\n");
          tWorkStatus3 = IDLE3;//清除上一次的状态
          tWorkMode3 = START3;
          lx_can_choice(CHANNEL_C1);
         
          cwb_mdelay(100);
          
          if(lx_can_send()){
            tCwbCanTx3.data[0] = 1;
          } else {
            tCwbCanTx3.data[0] = 2;
            led_on(LED_RED3);
          }
      
         
          tCwbCanTx3.id = tCwbCanRx.id;
          cwb_can_write(&tCwbCanTx3);    
          cwb_mdelay(2);
          
          return TRUE;
        }
        else if(HAND04 == tWorkMode4){
          tWorkStatus4 = IDLE4;
          tWorkMode4 = START4;
         
          if(lx_can_send()){
            tCwbCanTx4.data[0] = 1;
          } else {
            tCwbCanTx4.data[0] = 2;
            led_on(LED_RED4);
          }
        
           tCwbCanTx4.id = tCwbCanRx.id;
           cwb_can_write(&tCwbCanTx4);    
           cwb_mdelay(2);          
           
            return TRUE;
        }
      
       
        
       
              
        //power_off_all();//关闭所有电源输入
    }
    return TRUE;
}


void test_update(void)
{
   
    cwb_mode_hand1(); //step1 set spv
    cwb_mode_hand();  //step2 judge target
    cwb_test_busy();  //test begin : no led
    cwb_test_end();   //test end : light red or green
}



void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    if(CAN_MessagePending(CAN1, CAN_FIFO0) > 0) {
        s_bFlagCanInterrup = TRUE;
        s_bFlagCanScan = TRUE;
        cwb_can_read(&tCwbCanRx);
       
    }
}
//
//CANID[11:8] [7:0]
//function  address
void cwb_mode_choose(void)
{
    if(s_bFlagCanInterrup){
        s_bFlagCanInterrup = FALSE;
        switch(tCwbCanRx.id >> 8 & 0xF){ //增加功能位
            case START://0
                 tWorkMode = START;
                 busyflag =1; //允许进入测试过程
                 tCwbViMsg.fCountGet11 =0;//错误清零
                 tCwbViMsg.fCountGet12 =0;//错误清零
                 tCwbViMsg.fCountGet13 =0;
                 tCwbViMsg.fCountGet14 =0;
                 
                 tCwbViMsg.fCountGet21 =0;//错误清零
                 tCwbViMsg.fCountGet22 =0;//错误清零
                 tCwbViMsg.fCountGet23 =0;
                 tCwbViMsg.fCountGet24 =0;
                 
                 tCwbViMsg.fCountGet31 =0;//错误清零
                 tCwbViMsg.fCountGet32 =0;//错误清零
                 tCwbViMsg.fCountGet33 =0;
                 tCwbViMsg.fCountGet34 =0;
                 
                 tCwbViMsg.fCountGet41 =0;//错误清零
                 tCwbViMsg.fCountGet42 =0;//错误清零
                 tCwbViMsg.fCountGet43 =0;
                 tCwbViMsg.fCountGet44 =0;
                 if( tCwbCanRx.data[0] == 1){//一拖四：DATA[0]指的是哪個工位，第一個工位 
                   tWorkMode1 = START1;
                   tError1 = ERROR_NO1;
                   tWorkStatus1 = BUSY1; 
                                  
                   
                   tCwbCanTx1.id = tCwbCanRx.id;
                   tCwbCanTx1.data[0] = 1;  //response
                   cwb_can_write(&tCwbCanTx1);
                   #ifdef __DEBUG__
                   printf("Receive Start1, begin test!\n");
                   #endif
                 }else if( tCwbCanRx.data[0] == 2){
                   tWorkMode2 = START2;
                   tError2 = ERROR_NO2;
                   tWorkStatus2 = BUSY2;
                 
                   tCwbCanTx2.id = tCwbCanRx.id;
                   tCwbCanTx2.data[0] = 1;  //response
                   cwb_can_write(&tCwbCanTx2);
                   
                 }else if( tCwbCanRx.data[0] == 3){
                   tWorkMode3 = START3;
                   tError3 = ERROR_NO3;
                   tWorkStatus3 = BUSY3;
                  
                   tCwbCanTx3.id = tCwbCanRx.id;
                   tCwbCanTx3.data[0] = 1;  //response
                   cwb_can_write(&tCwbCanTx3);
                   #ifdef __DEBUG__
                   printf("Receive Start3, begin test!\n");
                   #endif
                 }else if( tCwbCanRx.data[0] == 4){
                   tWorkMode4 = START4;
                   tError4 = ERROR_NO4;
                   tWorkStatus4 = BUSY4;
                   
                   tCwbCanTx4.id = tCwbCanRx.id;
                   tCwbCanTx4.data[0] = 1;  //response
                   cwb_can_write(&tCwbCanTx4);
                   
                 }		            
                chNumCfg = 0;
                break;
            case END://1
                tWorkMode = END;
                busyflag =0; 
                //lx_set_current(0);
                if( tCwbCanRx.data[0] == 1){
                   ACC_choose(OFF1);                            
                   tWorkMode1 = END1;          
                   power_on(a,OFF);   //产品断电电
                   lx_hub_load(a,OFF);//产品去除老化 
                   lx_set_current(CHANNEL_A1,0);
                   tCwbCanTx1.id = tCwbCanRx.id;
                   tCwbCanTx1.data[0] = 1;  //response
                   cwb_can_write(&tCwbCanTx1);  
                   printf("Receive end1, end test!\n");
                }else if( tCwbCanRx.data[0] == 2){
                  
                   tWorkMode2 = END2;
                   power_on(b,OFF);   //产品断电电
                   lx_hub_load(b,OFF);//产品去除老化 
                   lx_set_current(CHANNEL_B1,0);
                   tCwbCanTx2.id = tCwbCanRx.id;
                   tCwbCanTx2.data[0] = 1;  //response
                   cwb_can_write(&tCwbCanTx2); 
                 //  printf("Receive end2, end test!\n");
                }else if( tCwbCanRx.data[0] == 3){
                   ACC_choose(OFF2); 
                   tWorkMode3 = END3;
                   power_on(c,OFF);   //产品断电电
                   lx_hub_load(c,OFF);//产品去除老化 
                   lx_set_current(CHANNEL_C1,0);
                   tCwbCanTx3.id = tCwbCanRx.id;
                   tCwbCanTx3.data[0] = 1;  //response
                   cwb_can_write(&tCwbCanTx3); 
                   printf("Receive end3, end test!\n");
                }else if( tCwbCanRx.data[0] == 4){
                 
                   tWorkMode4 = END4;
                   power_on(d,OFF);   //产品断电电
                   lx_hub_load(d,OFF);//产品去除老化 
                   lx_set_current(CHANNEL_D1,0);
                   tCwbCanTx4.id = tCwbCanRx.id;
                   tCwbCanTx4.data[0] = 1;  //response
                   cwb_can_write(&tCwbCanTx4); 
                   //printf("Receive end4, end test!\n");
                }  
                              
                chNumCfg = 0;
                break;
            case POLL://2
                tWorkMode = POLL;
                if( tCwbCanRx.data[0] == 1){
                   tWorkMode1 = POLL1;                
                }else if( tCwbCanRx.data[0] == 2){
                   tWorkMode2 = POLL2;                
                }else if( tCwbCanRx.data[0] == 3){
                   tWorkMode3 = POLL3;                
                }else if( tCwbCanRx.data[0] == 4){
                   tWorkMode4 = POLL4;                
                }  
                chNumCfg = 0;
                #ifdef __DEBUG__
			printf("Will in poll!\n");
		#endif
                break;
            case CFG://3
                tWorkMode  = CFG; 
                tWorkMode1 = CFG1;
                tWorkMode2 = CFG2;
                tWorkMode3 = CFG3;
                tWorkMode4 = CFG4;
                tError1=ERROR_NO1;
                tError2=ERROR_NO2;
                tError3=ERROR_NO3;
                tError4=ERROR_NO4;
                
                if(chNumCfg > 20){    //can not over 20, in fact
                    chNumCfg = 0;
                }
                tCwbCanCfg[chNumCfg].id = tCwbCanRx.id;
                tCwbCanCfg[chNumCfg].dlc = tCwbCanRx.dlc;
                memcpy(tCwbCanCfg[chNumCfg].data, tCwbCanRx.data, tCwbCanRx.dlc);
                tCwbCanCfg[chNumCfg].flag = tCwbCanRx.flag;
                //can_msg_print(&tCwbCanCfg[chNumCfg], "\n");
                if(0 == chNumCfg) {
                    chNumCfgTotal = tCwbCanCfg[0].data[7] + 1; //1 -> last CRC bit
			
				printf("The chNumCfgTotal message is %d\n", chNumCfgTotal);
			
                }
                chNumCfg++;
                break;
            case READ://4
                tWorkMode = READ;
                if( tCwbCanRx.data[0] == 1){
                   tWorkMode1 = READ1;                
                }else if( tCwbCanRx.data[0] == 2){
                   tWorkMode2 = READ2;                
                }else if( tCwbCanRx.data[0] == 3){
                   tWorkMode3 = READ3;                
                }else if( tCwbCanRx.data[0] == 4){
                   tWorkMode4 = READ4;                
                }  
                chNumCfg = 0;
                break;
            case HAND://5
                tWorkMode = HAND;              
                if( tCwbCanRx.data[0] == 1){
                   tWorkMode1 = HAND01; 
                   //printf("Receive hand1!\n");
                }else if( tCwbCanRx.data[0] == 2){
                   tWorkMode2 = HAND02;
                  // printf("Receive hand2!\n");
                }else if( tCwbCanRx.data[0] == 3){
                   tWorkMode3 = HAND03;
                  //printf("Receive hand3!\n");
                }else if( tCwbCanRx.data[0] == 4){
                   tWorkMode4 = HAND04;
                  // printf("Receive hand4!\n");
                }  
                chNumCfg = 0;
                break;
            case FAULT://6
                tWorkMode = FAULT; 
                if( tCwbCanRx.data[0] == 1){
                   tWorkMode1 = FAULT1;  
                   tCwbCanTx1.id = tCwbCanRx.id;
                   cwb_can_write(&tCwbCanTx1); 
                   printf("Receive FAULT1!\n");
                }else if( tCwbCanRx.data[0] == 2){
                   tWorkMode2 = FAULT2;
                   tCwbCanTx2.id = tCwbCanRx.id;
                   cwb_can_write(&tCwbCanTx2);
                   printf("Receive FAULT2!\n");
                }else if( tCwbCanRx.data[0] == 3){
                   tWorkMode3 = FAULT3;
                   tCwbCanTx3.id = tCwbCanRx.id;
                   cwb_can_write(&tCwbCanTx3); 
                   printf("Receive FAULT3!\n");
                }else if( tCwbCanRx.data[0] == 4){
                   tWorkMode4 = FAULT4; 
                   tCwbCanTx4.id = tCwbCanRx.id;
                   cwb_can_write(&tCwbCanTx4);
                   printf("Receive FAULT4!\n");
                }  
                break;
            case HAND1: //power on7
                tWorkMode = HAND1;
                busyflag =0; 
                s_chCount1 = 0;//测试次数清零
                s_chCount2 = 0;//测试次数清零
                s_chCount3 = 0;
                s_chCount4 = 0;
                can_count1 = 0;
                can_count3 = 0;             
                led_off(LED_YELLOW1); 
                led_off(LED_YELLOW2); 
                led_off(LED_YELLOW3); 
                led_off(LED_YELLOW4); 
                
                 tCwbViMsg.fCountGet11 =0;//错误清零
                 tCwbViMsg.fCountGet12 =0;//错误清零
                 tCwbViMsg.fCountGet13 =0;
                 tCwbViMsg.fCountGet14 =0;
                 
                 tCwbViMsg.fCountGet21 =0;//错误清零
                 tCwbViMsg.fCountGet22 =0;//错误清零
                 tCwbViMsg.fCountGet23 =0;
                 tCwbViMsg.fCountGet24 =0;
                 
                 tCwbViMsg.fCountGet31 =0;//错误清零
                 tCwbViMsg.fCountGet32 =0;//错误清零
                 tCwbViMsg.fCountGet33 =0;
                 tCwbViMsg.fCountGet34 =0;
                 
                 tCwbViMsg.fCountGet41 =0;//错误清零
                 tCwbViMsg.fCountGet42 =0;//错误清零
                 tCwbViMsg.fCountGet43 =0;
                 tCwbViMsg.fCountGet44 =0;
                if( tCwbCanRx.data[0] == 1){
                   tWorkMode1 = HAND11;
                   GPIO_WriteBit(GPIOF, GPIO_Pin_14, Bit_RESET);//短路复位
                   cwb_mdelay(2);
                   GPIO_WriteBit(GPIOF, GPIO_Pin_14, Bit_SET);
                   
                }else if( tCwbCanRx.data[0] == 2){
                   tWorkMode2 = HAND12;
                   GPIO_WriteBit(GPIOF, GPIO_Pin_15, Bit_RESET);
                   cwb_mdelay(2);
                   GPIO_WriteBit(GPIOF, GPIO_Pin_15, Bit_SET);
                  
                }else if( tCwbCanRx.data[0] == 3){
                   tWorkMode3 = HAND13;
                   GPIO_WriteBit(GPIOG, GPIO_Pin_0, Bit_RESET);
                   cwb_mdelay(2);
                   GPIO_WriteBit(GPIOG, GPIO_Pin_0, Bit_SET);
                   
                }else if( tCwbCanRx.data[0] == 4){
                   tWorkMode4 = HAND14;
                   GPIO_WriteBit(GPIOG, GPIO_Pin_1, Bit_RESET);
                   cwb_mdelay(2);
                   GPIO_WriteBit(GPIOG, GPIO_Pin_1, Bit_SET);
                  
                }
                break;
            default:
                chNumCfg = 0;
        }
    }
}

//

//
#define RATIO 0
#define VIN 1
#define VOUT 2
#define CIN 3
#define COUT 4
#define VOUT1 5
#define COUT1 6
bool vi_to_can(CWB_MISC_T *tCwbViMsg, CWB_MODE_E tWorkMode)
{
    if(NULL == tCwbViMsg){
        return FALSE;
    }
    if(READ == tWorkMode){
        tCwbCanTx.id = tCwbCanRx.id;
        switch(tCwbCanRx.data[1]){
            case RATIO:
                tCwbViMsg -> fRatioGet = 80.1; 
                memcpy(&tCwbCanTx.data, &tCwbViMsg -> fRatioGet,sizeof(tCwbViMsg -> fRatioGet));
	
			//printf("Ratio is %f\r\n", *(float *)tCwbCanTx.data);
	
                break;
            case VIN:
                
                memcpy(&tCwbCanTx.data, &tCwbViMsg -> fVinGet, sizeof(tCwbViMsg -> fVinGet));
	
			//printf("Input voltage is %f\r\n", *(float *)tCwbCanTx.data);
		
                break;
            case VOUT:
               
                memcpy(&tCwbCanTx.data, &tCwbViMsg -> fVoutGet, sizeof(tCwbViMsg -> fVoutGet));
		
			//printf("Output voltage is %f\r\n", *(float *)tCwbCanTx.data);
	
                break;
            case CIN:
                
                memcpy(&tCwbCanTx.data, &tCwbViMsg -> fCinGet, sizeof(tCwbViMsg -> fCinGet));
		
			//printf("Input current is %f\r\n", *(float *)tCwbCanTx.data);
		
                break;
            case COUT:
                
                memcpy(&tCwbCanTx.data, &tCwbViMsg -> fCoutGet, sizeof(tCwbViMsg -> fCoutGet));
		
			//printf("Output current is %f\r\n", *(float *)tCwbCanTx.data);
		
                break;
            case VOUT1:
                
                memcpy(&tCwbCanTx.data, &tCwbViMsg -> fVoutGet1, sizeof(tCwbViMsg -> fVoutGet1));
		
			//printf("Input current is %f\r\n", *(float *)tCwbCanTx.data);
		
                break;
            case COUT1:
                
                memcpy(&tCwbCanTx.data, &tCwbViMsg -> fCoutGet1, sizeof(tCwbViMsg -> fCoutGet1));
		
			//printf("Output current is %f\r\n", *(float *)tCwbCanTx.data);
		
                break;
            default:
                break;
        }
    } else if(FAULT == tWorkMode) {
            tCwbCanTxFault.id = tCwbCanRx.id;
            switch(tCwbCanRx.data[0]) {
                case VIN:
                    //tCwbViMsgFault -> fVinGet = 12.5; // for test
                    memcpy(&tCwbCanTxFault.data, &tCwbViMsg -> fVinGet, sizeof(tCwbViMsg -> fVinGet));
			#ifdef __DEBUG__
				printf("Fault Input voltage is %f\r\n", *(float *)tCwbCanTxFault.data);
			#endif
                    break;
                case VOUT:
                    //tCwbViMsgFault -> fVoutGet = 11.5; // for test
                    memcpy(&tCwbCanTxFault.data, &tCwbViMsg -> fVoutGet, sizeof(tCwbViMsg -> fVoutGet));
			#ifdef __DEBUG__
				printf("Fault Output voltage is %f\r\n", *(float *)tCwbCanTxFault.data);
			#endif
                    break;
                case CIN:
                    //tCwbViMsgFault -> fCinGet = 20.5; // for test
                    memcpy(&tCwbCanTxFault.data, &tCwbViMsg -> fCinGet, sizeof(tCwbViMsg -> fCinGet));
			#ifdef __DEBUG__
				printf("Fault Input current is %f\r\n", *(float *)tCwbCanTxFault.data);
			#endif
                    break;
                case COUT:
                    //tCwbViMsgFault -> fCoutGet = 30.5; // for test
                    memcpy(&tCwbCanTxFault.data, &tCwbViMsg -> fCoutGet, sizeof(tCwbViMsg -> fCoutGet));
			#ifdef __DEBUG__
				printf("Fault Output current is %f\r\n", *(float *)tCwbCanTxFault.data);
			#endif
                    break;
                default:
                    break;
            }
      }
    return TRUE;
}
//
bool eight_char_to_four_char(char *chBufIn, uint8_t *chBufOut)
{
    if((NULL == chBufIn) || (NULL == chBufOut)) {
        return FALSE;
    }
    for(uint8_t chIndex = 0; chIndex < 8; chIndex++) {
        if((chBufIn[chIndex]<= 9) && (chBufIn[chIndex]>= 0)) {
            chBufIn[chIndex]+= '0';
        } else if((chBufIn[chIndex] <= 0xf) && (chBufIn[chIndex] >= 0xa)) {
            chBufIn[chIndex] = chBufIn[chIndex] - 10 + 'a';
        }
    }
    sscanf(chBufIn, "%2x%2x%2x%2x", &chBufOut[0], &chBufOut[1], &chBufOut[2], &chBufOut[3]);
    return TRUE;
}
//
bool can_to_vi(CWB_MISC_T *tCwbViMsg, CWB_MODE_E tWorkMode)
{
    if(NULL == tCwbViMsg) {
        return FALSE;
    }
    uint8_t chBuf[4];
    memset(chBuf, 0, sizeof(chBuf));
    if(CFG == tWorkMode) {
        eight_char_to_four_char(tCwbCanCfg[1].data, chBuf);
        tCwbViMsg -> Volinset = *(float *)chBuf;//产品供电选择5v或者12v
        printf("VOL0 SET is %f\n", tCwbViMsg -> Volinset);
        
        eight_char_to_four_char(tCwbCanCfg[2].data, chBuf);
        tCwbViMsg -> fCurOutSet = *(float *)chBuf;//负载老化的电流
        printf("CUR SET is %f\n", tCwbViMsg -> fCurOutSet);
                
        eight_char_to_four_char(tCwbCanCfg[3].data, chBuf);
        tCwbViMsg -> fTimesSet = *(float *)chBuf;//负载时间（单位：分钟）
        printf("TIEM SET is %f\n", tCwbViMsg -> fTimesSet);
        
        eight_char_to_four_char(tCwbCanCfg[4].data, chBuf);
        tCwbViMsg -> downselect = *(float *)chBuf;//一个还是两个下行口
        printf("DOWN select %f\n", tCwbViMsg -> downselect);
        
        eight_char_to_four_char(tCwbCanCfg[5].data, chBuf);
        tCwbViMsg -> downVolLimitH = *(float *)chBuf;   //下行口的电压输出口上限
        printf("DWON HIGH LIMIT  %f\n", tCwbViMsg -> downVolLimitH);
                
        eight_char_to_four_char(tCwbCanCfg[6].data, chBuf);
        tCwbViMsg -> downVolLimitL = *(float *)chBuf;  //下行口的电压输出下限
        printf("DWON LOW LIMIT is %f\n", tCwbViMsg -> downVolLimitL);
        
        eight_char_to_four_char(tCwbCanCfg[7].data, chBuf);
        tCwbViMsg -> fCountVolLimit = *(float *)chBuf;//允许误差次数
        printf("ERROR COUNT LIMIT is %f\n", tCwbViMsg -> fCountVolLimit);
        
        
        eight_char_to_four_char(tCwbCanCfg[8].data, chBuf);
        tCwbViMsg -> fCurOutSet1 = *(float *)chBuf;//负载老化的电流
        printf("CUR1 SET is %f\n", tCwbViMsg -> fCurOutSet1);
        
        
        eight_char_to_four_char(tCwbCanCfg[9].data, chBuf);
        tCwbViMsg -> downSetVol = *(float *)chBuf;//负载老化的补偿电压
        printf("VOL SET is %f\n", tCwbViMsg -> downSetVol);

        eight_char_to_four_char(tCwbCanCfg[10].data, chBuf);
        tCwbViMsg -> productselect = *(float *)chBuf;//负载产品的选择
        printf("VOL SET is %f\n", tCwbViMsg -> downSetVol);

        tCwbViMsg -> hwCheckSum = tCwbCanCfg[11].data[6] * 16 + tCwbCanCfg[11].data[7];//CHECK_SUM
        printf("CFG Check_Sum recv is %d\n", tCwbViMsg -> hwCheckSum);
        tCwbViMsg -> hwCheckSumCalc = 0;
        for(uint8_t chIndex = 0; chIndex < chNumCfgTotal; chIndex++) {
            for(uint8_t chNum = 0; chNum < 8; chNum++) {
                uint8_t chTmp = 0;
                if((tCwbCanCfg[chIndex].data[chNum] <= '9') && (tCwbCanCfg[chIndex].data[chNum] >= '0')) {
                    chTmp = tCwbCanCfg[chIndex].data[chNum] - '0';
                } else if((tCwbCanCfg[chIndex].data[chNum] <= 'f') && (tCwbCanCfg[chIndex].data[chNum] >= 'a')) {
                    chTmp = tCwbCanCfg[chIndex].data[chNum] - 'a' + 10;
                } else if(tCwbCanCfg[chIndex].data[chNum] <= 0xf) {
                    chTmp = tCwbCanCfg[chIndex].data[chNum];
                }
                tCwbViMsg -> hwCheckSumCalc += chTmp;
            }
        }
        printf("Calc the checksum is %d\r\n", tCwbViMsg -> hwCheckSumCalc);
        if(tCwbViMsg -> hwCheckSum != tCwbViMsg -> hwCheckSumCalc) {
            tError1 = ERROR_CFG1;
            tError2 = ERROR_CFG2;
            tError3 = ERROR_CFG3;
            tError4 = ERROR_CFG4;
            printf("config   error\r\n");
        }
    }
    return TRUE;
}



 
void TIM2_IRQHandler(void)
{
    static uint32_t s_wCount = 0;
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET){ 
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        
       
        s_wCount++;
       // get_outadc(); 
    if(24000 == s_wCount) { //judge once every 5s
            s_wCount = 0;
            //printf("%d\n", s_bFlagCanScan);
            if((BUSY1 == tWorkStatus1) || (BUSY2 == tWorkStatus2) || (BUSY3 == tWorkStatus3) || (BUSY4 == tWorkStatus4)) { //only in busy mode
                if(s_bFlagCanScan) {  //if exist can frame
                    s_bFlagCanScan = FALSE; //clear can flag
                } else {  //if no can frame, means communication error
                    printf("Can communication error!Stop Test!\n");
                    tWorkMode = END;
                    tWorkMode1 = END1;
                    tWorkMode2 = END2;
                    tWorkMode3 = END3;
                    tWorkMode4 = END4;
                    tError1 = ERROR_CAN1;
                    tError2 = ERROR_CAN2;
                    tError3 = ERROR_CAN3;
                    tError4 = ERROR_CAN4;
                }
            }
        }
    } 
}

//check dut target or not
//read the out of dut to judge the target


static void cwb_init(void)
{    
  
  
    uart_cfg_t cf3;
    cf3.baud = 115200;
    uart3.init(&cf3);             
    cwb_scan_init(10); //100ms scan   
    lx_LED_Init();
    lx_RELAY_Init();
    lx_drive_init();
    DMA_ADC_ENABLE();
    El_CURRENT_Init();
    cwb_can_init();        
    //shell_mute((const struct console_s *) &uart1);
    //printf("%d\r\n", encode_read()); //read the encode
    can_filter_set(wCanId);
    if(can_filter_set(encode_read())) {
        
    } else {
        printf("ENCODE Oversize 60!\r\n");
        
    }
    printf("cwb sw v1.0, build: %s %s\n\r", __DATE__, __TIME__);
    cwb_light_after_power(); 
    led_on(LED_GREEN1);
    led_on(LED_GREEN2);
    led_on(LED_GREEN3);
    led_on(LED_GREEN4);
   
}

void cwb_update(void)
{
    cwb_mode_choose();
    cwb_mode_poll();
    cwb_mode_read();
    cwb_mode_cfg();
    cwb_mode_fault();  
}
int main(void)
{
    task_Init();
    cwb_init();
    while(1){
        task_Update();
        test_update();
        //cwb_can_write(&tCwbCanTx5);
        //cwb_mdelay(2);
    } 
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
void hardfault(void)
{
    int *p = (int *)1;
    *p = 5;
    printf("%d345\n", *p);
}
#endif
#endif
//typedef void (*reset)(void);
typedef void (*fault)(void);
static int cmd_lx_func(int argc, char *argv[])
{
    const char *usage = {
        "lx  usage : \n"
        "lx  reset\r\n"
        "lx  nvm\r\n"
        "lx  in     a/b/c/d"   
        "lx  mode   x\r\n"
        "lx  set    current            0.5-2.1A\n"
        "lx  ACC    on1/on2/off1/off2 \n"
        "lx  canx    CHANNEL_A1/CHANNEL_C1       \n"  
        "lx  power  choose             VCC_INL / VCC_INH\n"
        "lx  power  on/off             a/b/c/d\n"
        "lx  load   enable/disable     a/b/c/d\n" 
        "lx  short  enable/disable     a/b/c/d\n" 
        "lx  hub    select             down1/down2 \n"
        "选择通道2时读取ADC时先选择lx  hub    select    down1 \n"
        "lx  get    adc                INUA/INUB/INUC/INUD   INIA/INIB/INIC/INID   OUTUA1/OUTUB1/OUTUC1/OUTUD1   OUTIA1/OUTIB1/OUTIC1/OUTID1 \
                                       OUTUA2/OUTUB2/OUTUC2/OUTUD2    OUTIA2/OUTIB2/OUTIC2/OUTID2  \n"        
        "lx  led    on/off             LED_GREEN1 / LED_GREEN2 / LED_GREEN3 / LED_GREEN4 / LED_RED1 / LED_RED2 /  LED_RED3 /  LED_RED4  \n"          
        "lx  filter                    x\r\n"
        "lx  load   current            a/b/c/d\n"
        "lx  light  on/off \n"  
        "lx  can    MP5/PEPS/DoorStsFrntLe/BCM_PowerMode\n" 
    };
    if(argc < 2){
        printf("%s", usage);
        return 0;
    }
    if(2 == argc) {
        if(!strcmp(argv[1], "reset")) {
            GenerateSystemReset();
        }
        else if(!strcmp(argv[1], "fault")) {
            
            hardfault();
        
        }else if(!strcmp(argv[1], "settime")) {
           printf("s_chCount1=%d\r\n",s_chCount1); 
           printf("s_chCount3=%d\r\n",s_chCount3);   
           
        
        } else if(!strcmp(argv[1], "nvm")) {
            
           tCwbViMsg.Volinset = 12.0;
           tCwbViMsg.fCurOutSet = 1.0;
           tCwbViMsg.fTimesSet = 30;
           tCwbViMsg.downselect = 1;
           tCwbViMsg.downVolLimitH = 5.5;
           tCwbViMsg.downVolLimitL = 4.5;
           tCwbViMsg.fCountVolLimit = 4;
           nvm_save();
        
        } else if(!strcmp(argv[1], "-m")) {
            shell_unmute((const struct console_s *) &uart1);
            pc_shell = MAMUAL;
        } else if(!strcmp(argv[1], "-a")) {
            shell_mute((const struct console_s *) &uart1);
            pc_shell = AUTOMATIC;
        }else if(!strcmp(argv[1], "workmode")) {
           printf("%d,%d,%d,%d\r\n",tWorkMode1,tWorkMode2,tWorkMode3,tWorkMode4);
        }else if(!strcmp(argv[1], "workstatus")) {
           printf("%d,%d,%d,%d\r\n",tWorkStatus1,tWorkStatus2,tWorkStatus3,tWorkStatus4);
        }else if(!strcmp(argv[1], "pwm")) {
            TIM_SetCompare1(TIM3, 250);
            TIM_SetCompare2(TIM3, 250);
            TIM_SetCompare3(TIM3, 250);
            TIM_SetCompare4(TIM3, 250);
          
        }
    }
    if(3 == argc){
            
    if(!strcmp(argv[1], "light")) {
            int s=0;
            if(!strcmp(argv[2], "on")){
                 char ax[]="can light on";
                 s=strlen(ax);               
                 for(int i=0;i<s;i++)
                    uart3.putchar(ax[i]);
                 uart3.putchar('\r');
                                      
               
            } else if(!strcmp(argv[2], "off")){
                 char ax[]="can light off";                 
                 s=strlen(ax); 
                 
                 for(int i=0;i<s;i++)                 
                    uart3.putchar(ax[i]);
                 uart3.putchar('\r');                     
               
            }  
     
            
            
            
     }else  if(!strcmp(argv[1], "read")) {
            
            if(!strcmp(argv[2], "can")){
               printf("%x\r\n",canerror);      
                                      
               
            }
            
            
            
     } 
    else if(!strcmp(argv[1], "in")) {
            if(!strcmp(argv[2], "a")){
                 if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3) ==1)
                   printf("HIGH1!\r\n");
                 else
                   printf("LOW1!\r\n");
            }else if(!strcmp(argv[2], "b")){
                 if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2) ==1)
                   printf("HIGH2!\r\n");
                 else
                   printf("LOW2!\r\n");
              
              
            } else if(!strcmp(argv[2], "c")){
                 if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_1) ==1)
                   printf("HIGH3!\r\n");
                 else
                   printf("LOW3!\r\n");
            }else if(!strcmp(argv[2], "d")){
                 
                 if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_0) ==1)
                   printf("HIGH4!\r\n");
                 else
                   printf("LOW4!\r\n");
            }
       
     }
    else if(!strcmp(argv[1], "ACC")) {
            if(!strcmp(argv[2], "on1")){
                 ACC_choose(ON1);
                 printf("operation success!\r\n");
            } else if(!strcmp(argv[2], "off1")){
                 ACC_choose(OFF1);                  
                 printf("operation success!\r\n");
            }else if(!strcmp(argv[2], "on2")){
                 
                 ACC_choose(ON2);
                 printf("operation success!\r\n");
            } else if(!strcmp(argv[2], "off2")){
                                  
                 ACC_choose(OFF2);
                 printf("operation success!\r\n");
            }  
            
     }else if(!strcmp(argv[1], "current")) {
            if(!strcmp(argv[2], "set")){
              lx_set_current(CHANNEL_A1,1.0);
               
            } 
            
     }
     else if(!strcmp(argv[1], "canx")) {
            if(!strcmp(argv[2], "CHANNEL_A1")){
                 lx_can_choice(CHANNEL_A1);
                 printf("operation success!\r\n");
               
            } else if(!strcmp(argv[2], "CHANNEL_C1")){
                 lx_can_choice(CHANNEL_C1);                  
                 printf("operation success!\r\n");
            } 
     }
     else if(!strcmp(argv[1], "can")) {
            int s=0;
            if(!strcmp(argv[2], "MP5")){
                 char ax[]="can once send 535 00 00 00 00 00 80 00 00 on";
                 s=strlen(ax); 
                 printf("%d\r\n",s);                 
                 for(int i=0;i<s;i++)
                    uart3.putchar(ax[i]);
                 uart3.putchar('\r');                                     
               
            } else if(!strcmp(argv[2], "PEPS")){
                 char ax[]="can once send 325 00 00 00 00 00 02 00 00 on";                 
                 s=strlen(ax); 
                 printf("%d\r\n",s);                      
                 for(int i=0;i<s;i++)                 
                    uart3.putchar(ax[i]);
                 uart3.putchar('\r');                     
               
            } else if(!strcmp(argv[2], "DoorStsFrntLe")){
                 char ax[]="can once send 310 00 00 00 00 80 00 00 00 on";                 
                 s=strlen(ax); 
                 printf("%d\r\n",s);     
                 for(int i=0;i<s;i++)                 
                    uart3.putchar(ax[i]);
                 uart3.putchar('\r');                     
               
            } else if(!strcmp(argv[2], "BCM_PowerMode")){
                 char ax[]="can once send 311 00 00 00 00 00 02 00 00 on";;                 
                 s=strlen(ax); 
                 printf("%d\r\n",s);     
                 for(int i=0;i<s;i++)                 
                    uart3.putchar(ax[i]);
                 uart3.putchar('\r');                     
               
            } else if(!strcmp(argv[2], "all")){
                 char ax1[]="can once send 535 00 00 00 00 00 80 00 00 on";
                 s=strlen(ax1); 
                 printf("%d\r\n",s);                 
                 for(int i=0;i<s;i++)                 
                    uart3.putchar(ax1[i]);
                 uart3.putchar('\r');
                 cwb_mdelay(2);
                 char ax2[]="can once send 325 00 00 00 00 00 02 00 00 on";                 
                 s=strlen(ax2); 
                 printf("%d\r\n",s);                      
                 for(int i=0;i<s;i++)                 
                    uart3.putchar(ax2[i]);
                 uart3.putchar('\r');
                 cwb_mdelay(2);
                 char ax3[]="can once send 310 00 00 00 00 80 00 00 00 on";                 
                 s=strlen(ax3); 
                 printf("%d\r\n",s);     
                 for(int i=0;i<s;i++)                 
                    uart3.putchar(ax3[i]);
                 uart3.putchar('\r');  
                 cwb_mdelay(2);
                 char ax4[]="can once send 311 00 00 00 00 00 02 00 00 on";;                 
                 s=strlen(ax4); 
                 printf("%d\r\n",s);     
                 for(int i=0;i<s;i++)                 
                    uart3.putchar(ax4[i]);
                 uart3.putchar('\r');                     
                 cwb_mdelay(2);
                 printf("operation success!\r\n");
            }     
          
      } 
    else if(!strcmp(argv[1], "filter")) {
            sscanf(argv[2], "%d", &wCanId);
            if(can_filter_set(wCanId)) {
            } else {
		#ifdef __DEBUG__
			printf("ID oversize!\r\n");
		#endif
            }
      }  else if(!strcmp(argv[1], "read") && !strcmp(argv[2], "sn")) {
            printf("The flash size is %dKB\r\n", *(uint16_t *)0x1ffff7e0);
            printf("UID0 : is %d\r\n", *(uint32_t *)(0x1ffff7e8));
            printf("UID1 : is %d\r\n", *(uint32_t *)(0x1ffff7ec));
            printf("UID2 : is %d\r\n", *(uint32_t *)(0x1ffff7f0));
         
        
      } else if(!strcmp(argv[1], "mode")){
            if(!strcmp(argv[2], "start")) {
                busyflag=1;
                tWorkMode = START;
                tWorkMode1 = START1;
              //  tWorkMode2 = START2;
                tWorkMode3 = START3;
               // tWorkMode4 = START4;               
                tWorkStatus1 = BUSY1;
               // tWorkStatus2 = BUSY2;
                tWorkStatus3 = BUSY3;
               // tWorkStatus4 = BUSY4;
                tError1=ERROR_NO1; 
               // tError2=ERROR_NO2;
                tError3=ERROR_NO3;
               // tError4=ERROR_NO4;
          
                                  
		#ifdef __DEBUG__
			printf("In start mode!\r\n");
		#endif
            } else if(!strcmp(argv[2], "end")) {
                busyflag=0;
                tWorkMode1 = END1;
                tWorkMode2 = END2;
                tWorkMode3 = END3;
                tWorkMode4 = END4;
		#ifdef __DEBUG__
			printf("In end mode!\r\n");
		#endif
            } else if(!strcmp(argv[2], "poll")) {
                tWorkMode = POLL;
		#ifdef __DEBUG__
			printf("In poll mode!\r\n");
		#endif
            } else if(!strcmp(argv[2], "cfg")) {
                tWorkMode = CFG;
		#ifdef __DEBUG__
			printf("In cfg mode!\r\n");
		#endif
            } else if(!strcmp(argv[2], "read")) {
                tWorkMode = READ;
		#ifdef __DEBUG__
			printf("In read mode!\r\n");
		#endif
            }
        }
    }
    if(4 == argc) {
        
         if(!strcmp(argv[1], "load")&& !strcmp(argv[2], "current")) {
            if(!strcmp(argv[3], "a")){
               power_on(a,ON);
               lx_hub_load(a,ON);
               
               lx_set_current(CHANNEL_A1,2.0);
               lx_set_current(CHANNEL_A2,0.0);
              printf("operation success!\r\n");
            }
            else if(!strcmp(argv[3], "b")){
               power_on(b,ON);
               lx_hub_load(b,ON);
               lx_set_current(CHANNEL_B1,1.0);
               lx_set_current(CHANNEL_B2,1.0);
              printf("operation success!\r\n");
            }
            else if(!strcmp(argv[3], "c")){
               power_on(c,ON);
               lx_hub_load(c,ON);
               lx_set_current(CHANNEL_C1,2.0);
               lx_set_current(CHANNEL_C2,0.0);
              printf("operation success!\r\n");
            }
            else if(!strcmp(argv[3], "d")){
               power_on(d,ON);
               lx_hub_load(d,ON);
               lx_set_current(CHANNEL_D1,1.0);
               lx_set_current(CHANNEL_D2,1.0);
              printf("operation success!\r\n");
              
            }        
          
        }
        else if(!strcmp(argv[1], "power")) {
          if( !strcmp(argv[2], "on")){
            if(!strcmp(argv[3], "a")){
              power_on(a,ON);
              printf("operation success!\r\n");
            }
            else if(!strcmp(argv[3], "b")){
              power_on(b,ON);
              printf("operation success!\r\n");
            }
            else if(!strcmp(argv[3], "c")){
              power_on(c,ON);
              printf("operation success!\r\n");
            }
            else if(!strcmp(argv[3], "d")){
              power_on(d,ON);
              printf("operation success!\r\n");
              
            }
          }
          else if( !strcmp(argv[2], "off")){
            if(!strcmp(argv[3], "a")){
               power_on(a,OFF);
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "b")){
               power_on(b,OFF);
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "c")){
               power_on(c,OFF);
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "d")){
               power_on(d,OFF);
               printf("operation success!\r\n"); 
            }
          
          }
           else if( !strcmp(argv[2], "choose")){
            if(!strcmp(argv[3], "VCC_INL")){
               power_choose(VCC_INL);
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "VCC_INH")){
               power_choose(VCC_INH);
               printf("operation success!\r\n"); 
            }
              
           }
        }
        else if(!strcmp(argv[1], "load") ) {
             if( !strcmp(argv[2], "enable")){
            if(!strcmp(argv[3], "a")){
              lx_hub_load(a,ON);
              printf("operation success!\r\n");
            }
            else if(!strcmp(argv[3], "b")){
              lx_hub_load(b,ON);
              printf("operation success!\r\n");
            }
            else if(!strcmp(argv[3], "c")){
              lx_hub_load(c,ON);
              printf("operation success!\r\n");
            }
            else if(!strcmp(argv[3], "d")){
              lx_hub_load(d,ON);
              printf("operation success!\r\n");
              
            }
          }
           else if( !strcmp(argv[2], "disable")){
            if(!strcmp(argv[3], "a")){
               lx_hub_load(a,OFF);
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "b")){
               lx_hub_load(b,OFF);
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "c")){
                lx_hub_load(c,OFF);
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "d")){
               lx_hub_load(d,OFF);
               printf("operation success!\r\n"); 
            }
          
          }
          }
        else if(!strcmp(argv[1], "short") ) {
             if( !strcmp(argv[2], "enable")){
            if(!strcmp(argv[3], "a")){
              short_protect_enable(a,ON);
              printf("operation success!\r\n");
            }
            else if(!strcmp(argv[3], "b")){
              short_protect_enable(b,ON);
              printf("operation success!\r\n");
            }
            else if(!strcmp(argv[3], "c")){
              short_protect_enable(c,ON);
              printf("operation success!\r\n");
            }
            else if(!strcmp(argv[3], "d")){
              short_protect_enable(d,ON);
              printf("operation success!\r\n");
              
            }
          }
           else if( !strcmp(argv[2], "disable")){
            if(!strcmp(argv[3], "a")){
               short_protect_enable(a,OFF);
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "b")){
               short_protect_enable(b,OFF);;
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "c")){
               short_protect_enable(c,OFF);;
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "d")){
               short_protect_enable(d,OFF);;
               printf("operation success!\r\n"); 
            }
          
          }
          }
        else if(!strcmp(argv[1], "hub") && !strcmp(argv[2], "select")){
            if(!strcmp(argv[3], "down1")){
               hud_down_select(down1);
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "down2")){
               hud_down_select(down2);
               printf("operation success!\r\n"); 
            }         
       }
       else if(!strcmp(argv[1], "led") && !strcmp(argv[2], "on")){
            if(!strcmp(argv[3], "LED_GREEN1")){
               led_on(LED_GREEN1);
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "LED_GREEN2")){
               led_on(LED_GREEN2);
               printf("operation success!\r\n"); 
            }
             else if(!strcmp(argv[3], "LED_GREEN3")){
               led_on(LED_GREEN3);
               printf("operation success!\r\n"); 
            }     
             else if(!strcmp(argv[3], "LED_GREEN4")){
               led_on(LED_GREEN4);
               printf("operation success!\r\n"); 
            }     
             else if(!strcmp(argv[3], "LED_RED1")){
               led_on(LED_RED1);
               printf("operation success!\r\n"); 
            }     
             else if(!strcmp(argv[3], "LED_RED2")){
               led_on(LED_RED2);
               printf("operation success!\r\n"); 
            }     
             else if(!strcmp(argv[3], "LED_RED3")){
               led_on(LED_RED3);
               printf("operation success!\r\n"); 
            }     
             else if(!strcmp(argv[3], "LED_RED4")){
               led_on(LED_RED4);
               printf("operation success!\r\n"); 
            }     
            
       }
       else if(!strcmp(argv[1], "led") && !strcmp(argv[2], "off")){
            if(!strcmp(argv[3], "LED_GREEN1")){
               led_off(LED_GREEN1);
               printf("operation success!\r\n"); 
            }
            else if(!strcmp(argv[3], "LED_GREEN2")){
               led_off(LED_GREEN2);
               printf("operation success!\r\n"); 
            }
             else if(!strcmp(argv[3], "LED_GREEN3")){
               led_off(LED_GREEN3);
               printf("operation success!\r\n"); 
            }     
             else if(!strcmp(argv[3], "LED_GREEN4")){
               led_off(LED_GREEN4);
               printf("operation success!\r\n"); 
            }     
             else if(!strcmp(argv[3], "LED_RED1")){
               led_off(LED_RED1);
               printf("operation success!\r\n"); 
            }     
             else if(!strcmp(argv[3], "LED_RED2")){
               led_off(LED_RED2);
               printf("operation success!\r\n"); 
            }     
             else if(!strcmp(argv[3], "LED_RED3")){
               led_off(LED_RED3);
               printf("operation success!\r\n"); 
            }     
             else if(!strcmp(argv[3], "LED_RED4")){
               led_off(LED_RED4);
               printf("operation success!\r\n"); 
            }     
            
       }
        else if(!strcmp(argv[1], "get") && !strcmp(argv[2], "adc")){
               Different_ADC_selection a;
               if(!strcmp(argv[3], "INUA")){
                    a=INUA;
                 input_voltage(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "INUB")){
                    a=INUB;
                    input_voltage(get_adc(a));
               printf("operation success!  %d \r\n",a);
               }     
               else if(!strcmp(argv[3], "INUC")){
                    a=INUC;
                   input_voltage(get_adc(a));
               printf("operation success!  %d \r\n",a);
               }  
               else if(!strcmp(argv[3], "INUD")){
                    a=INUD;
                   input_voltage(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "INIA")){
                    a=INIA;
                    input_current(get_adc(a));
               printf("operation success!  %d \r\n",a);
               }
               else if(!strcmp(argv[3], "INIB")){
                    a=INIB;
                    input_current(get_adc(a));
               printf("operation success!  %d \r\n",a);
               }
               else if(!strcmp(argv[3], "INIC")){
                    a=INIC;
                    input_current(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "INID")){
                    a=INID;
                    input_current(get_adc(a));
               printf("operation success!  %d \r\n",a);
               }
               else if(!strcmp(argv[3], "OUTUA1")){
                    a=OUTUA;
                    output_voltage(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTUB1")){
                    a=OUTUB;
                    output_voltage(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTUC1")){
                    a=OUTUC;
                    output_voltage(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTUD1")){
                    a=OUTUD;
                    output_voltage(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTIA1")){
                    a=OUTIA;
                    output_current1(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTIB1")){
                    a=OUTIB;
                    output_current(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTIC1")){
                    a=OUTIC;
                    output_current1(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTID1")){
                    a=OUTID;
                    output_current(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTUA2")){
                    a=OUTUA;
                    output_voltage(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTUB2")){
                    a=OUTUB;
                    output_voltage(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTUC2")){
                    a=OUTUC;
                    output_voltage(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTUD2")){
                    a=OUTUD;
                    output_voltage(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTIA2")){
                    a=OUTIA;
                    output_current(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTIB2")){
                    a=OUTIB;
                    output_current(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTIC2")){
                    a=OUTIC;
                    output_current(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               else if(!strcmp(argv[3], "OUTID2")){
                    a=OUTID;
                    output_current(get_adc(a));
               printf("operation success!  %d \r\n",a); 
               }
               
               printf("\r\n"); 
            
            
            
       }
        } 
    return 0;
}
cmd_t cmd_lx = {"lx", cmd_lx_func, "commands for lx"};
DECLARE_SHELL_CMD(cmd_lx)


