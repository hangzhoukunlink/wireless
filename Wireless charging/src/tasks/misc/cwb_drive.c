

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
#include "sys/malloc.h"
#include "config.h"
#include "cwb_drive.h"

static const can_bus_t *s_ptCwb_Can_Bus = &can1;

void time3_init(uint32_t wFrq)
{
    pwm_cfg_t tCfg;
    const pwm_bus_t *ptPwm33 = &pwm33;
    tCfg.hz = wFrq;
    tCfg.fs = 100;
    ptPwm33 -> init(&tCfg);
    
    const pwm_bus_t *ptPwm34 = &pwm34;
    tCfg.hz = wFrq;
    tCfg.fs = 100;
    ptPwm34 -> init(&tCfg);
}
//ISET tim3_ch3 PB0

//PV tim3_ch4 PB1
void pv_set(uint32_t wDc)
{
    const pwm_bus_t *ptPwm = &pwm34;
    ptPwm -> set(wDc);
}

//LED_GREEN Bit_SET
void product_set_led(led_t tColor, led_status_t tStatus)
{
    BitAction ba;
    switch(tStatus) {
        case OFF:
            ba = Bit_RESET;
            break;
        case ON:
            ba = Bit_SET;
            break;
        default:
            return ;
    }
    
    switch(tColor) {
        case LED_GREEN:
            GPIO_WriteBit(GPIOE, GPIO_Pin_11, ba);
            //GPIO_WriteBit(GPIOD, GPIO_Pin_0, ba);
            break;
        case LED_RED:
            GPIO_WriteBit(GPIOE, GPIO_Pin_9, ba);
            //GPIO_WriteBit(GPIOD, GPIO_Pin_2, ba);
            break;
        case LED_YELLOW:
            GPIO_WriteBit(GPIOE, GPIO_Pin_9, ba);
            GPIO_WriteBit(GPIOE, GPIO_Pin_11, ba);
        default:
            break;
    }
}

void lx_drive_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);
    //ID
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOE, &GPIO_InitStructure);    
    //MCU_SI0   MCU_SI1  MCU_SI2   MCU_SI3   MCU_ACC_CK1  MCU_ACC_CK2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_10  ;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);    
    //MCU_SW   MCU_ACC1   MCU_ACC2 
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_SetBits(GPIOD, GPIO_Pin_13);
    GPIO_ResetBits(GPIOD, GPIO_Pin_9 | GPIO_Pin_11);
    //MMCU_CT1  MMCU_CT2  MMCU_CT3  MMCU_CT4
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 );
   
   
}

//enable or disable spv
void cwb_en_spv_set(bool bStatus)
{
    (FALSE != bStatus) ? GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_SET) : GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET);
}
//enable or disable eload
void cwb_en_eload_set(bool bStatus)
{
    (FALSE != bStatus) ? GPIO_WriteBit(GPIOE, GPIO_Pin_1, Bit_SET) : GPIO_WriteBit(GPIOE, GPIO_Pin_1, Bit_RESET);
}
//sig_out
void cwb_isg_set(bool bStatus)
{
    (FALSE != bStatus) ? GPIO_WriteBit(GPIOE, GPIO_Pin_0, Bit_SET) : GPIO_WriteBit(GPIOE, GPIO_Pin_0, Bit_RESET);
}
//max return 59
uint8_t encode_read(void)
{
    
    ENCODE_STATUS_T tEncode_Value;
    memset(&tEncode_Value, 0 ,sizeof(tEncode_Value));
    tEncode_Value.BYTE_BIT.BIT1 = (GPIOE -> IDR & 1 );
    tEncode_Value.BYTE_BIT.BIT2 = (GPIOE -> IDR & (1 << 1)) >> 1;
    tEncode_Value.BYTE_BIT.BIT3 = (GPIOE -> IDR & (1 << 2)) >> 2;
    tEncode_Value.BYTE_BIT.BIT4 = (GPIOE -> IDR & (1 << 3)) >> 3;
    tEncode_Value.BYTE_BIT.BIT5 = (GPIOE -> IDR & (1 << 4)) >> 4;
    tEncode_Value.BYTE_BIT.BIT6 = (GPIOE -> IDR & (1 << 5)) >> 5;
    tEncode_Value.BYTE_BIT.BIT7 = (GPIOE -> IDR & (1 << 6)) >> 6;
    tEncode_Value.BYTE_BIT.BIT8 = (GPIOE -> IDR & (1 << 7)) >> 7;
    
   // printf("Encode Low is %d\r\n", tEncode_Value.BYTE_HALF.BYTEL);
  //  printf("Encode High is %d\r\n", tEncode_Value.BYTE_HALF.BYTEH);
    printf("ID is %d\r\n", tEncode_Value.BYTE);
    
   
    return  tEncode_Value.BYTE;
}

//set filter of can
void cwb_can_init(void)
{
    can_cfg_t tCfg_pdi_can = {
        .baud = 20000,
    };
    s_ptCwb_Can_Bus -> init(&tCfg_pdi_can);  // init can
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_SetPriority(SysTick_IRQn, 0);
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}
//
void cwb_can_handler(void)
{
    can_msg_t tMsg;
    s_ptCwb_Can_Bus -> recv(&tMsg);
}
//two id : one is code; other is broadcast
bool can_filter_set(uint32_t wId)
{
      if(wId > 60) {
        return FALSE;
      }
    can_filter_t tFilter[] = {
        {
            .id = wId,
            .mask = 0xff,//匹配的can帧才能接受；
            .flag = 0,
        },
        {
            .id = 0x0000,
            .mask = 0xff,
            .flag = 0,
        },
        {
            .id = 0x100,
            .mask = 0x1ff,
            .flag = 0,
        },
    };
    s_ptCwb_Can_Bus -> filt(tFilter, 3);
    
    return TRUE;
}
//
bool cwb_can_read(can_msg_t *ptMsg)
{
    if(s_ptCwb_Can_Bus -> recv(ptMsg)) {
        return FALSE;
    }
    return TRUE;
}
//
bool cwb_can_write(can_msg_t *ptMsg)
{
    if(s_ptCwb_Can_Bus -> send(ptMsg)) {
        return FALSE;
    }
    return TRUE;
}

void cwb_scan_init(uint32_t wFrq)
{
    pwm_cfg_t tCfg;
    const pwm_bus_t *ptPwm = &pwm2;
    tCfg.hz = wFrq;
    tCfg.fs = 100;
    ptPwm->init(&tCfg);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//
void cwb_adc_init(void)
{
    //RCC_ClocksTypeDef clks;
    //ADC1/2/_ch0 PA0 ADC1/2/_ch1 PA1 ADC1/2/_ch2 PA2 ADC1/2/_ch3 PA3
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); /*72Mhz/6 = 12Mhz*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 |GPIO_Pin_5 |GPIO_Pin_6 | GPIO_Pin_7 ;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //ADC INIT, ADC1 INJECTED CH, CONT SCAN MODE
    ADC_InitTypeDef ADC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); /*72Mhz/6 = 12Mhz, note: 14MHz at most*/
    ADC_DeInit(ADC1);
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
    ADC_InitStructure.ADC_NbrOfChannel = 0;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_InjectedSequencerLengthConfig(ADC1, 4); //!!!length must be configured at first
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 9, ADC_SampleTime_239Cycles5); //12Mhz / (12.5 + 239.5) = 47Khz
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 10, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_2, 11, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_3, 12, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_4, 13, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_5, 14, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_6, 15, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_7, 16, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 7, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_9, 8, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_10,1, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_11,2, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_12,3, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_13,4, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_14,5, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_15,6, ADC_SampleTime_239Cycles5);
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
    ADC_AutoInjectedConvCmd(ADC1, ENABLE); //!!!must be set because inject channel do not support CONT mode independently
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1)); //WARNNING: DEAD LOOP!!!
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
