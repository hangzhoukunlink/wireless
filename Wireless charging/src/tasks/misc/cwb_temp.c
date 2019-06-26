/*******************************
 *jiamao.gu@2015.3 initial version
 *Functions as followed: 
 *1.Set the filter of can according to encode_read
 *2.Can communication : how to receive??
 *3.Accoding to ID1 Set Voltage and Current(PWM DUTY)
 *4.Measure Input Voltage and Current Converted into can message ID2
 *5.Measure Output Voltage and Current Converted into can message ID3
 *6.Calibration??
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
#include "sys/malloc.h"
#include "config.h"
#include "cwb_drive.h"

#define D2MV(D) ((D) * 2978 / 65535)
#define MV2D(MV) ((MV) * 65535 / 2978)
#define DMA_N 1   //length of data
#define DMA_C 4    //num of channel

uint8_t chCan_Id = 0;
CWB_MISC_T tCwb_Vi_Msg;
can_msg_t tCwb_Can_Tx = {0, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 0};
can_msg_t tCwb_Can_Rx = {0, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 0};
static uint8_t s_chFlag_10ms_Interrup = 0;
CWB_MODE_E can_to_viset(can_msg_t *ptCan_Msg);  //ID1
CWB_MODE_E viout_to_can(CWB_MISC_T *ptVi_Msg);  //ID2
CWB_MODE_E viout_to_can(CWB_MISC_T *ptVi_Msg);  //ID3
uint32_t cwb_mdelay(uint32_t wMs);
void cwb_adc_init(void);

uint32_t cwb_mdelay(uint32_t wMs)
{
    uint32_t wLeft;
    time_t tDeadline = time_get(wMs);
    do {
        wLeft = time_left(tDeadline);
        if(wLeft >= 10) { //system update period is expected less than 10ms
            ulp_update();
        }
    } while(wLeft > 0);
    return 0;
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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;    
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
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5); //12Mhz / (12.5 + 239.5) = 47Khz
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5);
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
    ADC_AutoInjectedConvCmd(ADC1, ENABLE); //!!!must be set because inject channel do not support CONT mode independently
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1)); //WARNNING: DEAD LOOP!!!
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
//
uint16_t adc_pa0_get(void)
{
    return ADC1->JDR1 << 1;
}

uint16_t adc_pa1_get(void)
{
    return ADC1->JDR2 << 1;
}

uint16_t adc_pa2_get(void)
{
    return ADC1->JDR3 << 1;
}

uint16_t adc_pa3_get(void)
{
    return ADC1->JDR4 << 1;
}

uint16_t adc_nul_get(void)
{
    return 0;
}
//
uint16_t adc_filter(uint8_t chChannel)
{
    uint32_t wResult = 0;
    for(int i = 0; i < DMA_N; i++) {
        //wResult += hwAdc_Converted_Value[i][chChannel];
    }
    return wResult >> 6;    //get average;
}
//
#define MAX_VOL 20000   //mv
#define MAX_CUR 50000   //ma
CWB_MODE_E can_to_viset(can_msg_t *ptCan_Msg)
{
    if(NULL == ptCan_Msg) {
        return ERROR_MODE;
    }
    can_msg_print(ptCan_Msg, "\n");
    tCwb_Vi_Msg.hwVol_Set = ptCan_Msg -> data[0] * 10000 + ptCan_Msg -> data[1] * 1000 + ptCan_Msg -> data[3] * 100;  //mv
    tCwb_Vi_Msg.hwCur_Set = ptCan_Msg -> data[4] * 10000 + ptCan_Msg -> data[5] * 1000 + ptCan_Msg -> data[7] * 100;  //ma
    if((tCwb_Vi_Msg.hwVol_Set > MAX_VOL) || (tCwb_Vi_Msg.hwCur_Set > MAX_CUR)) {
        return OVER_MODE;
    }
    return COMMON_MODE;
} 
//
CWB_MODE_E viin_to_can(CWB_MISC_T *ptVi_Msg)
{
    if(NULL == ptVi_Msg) {
        return ERROR_MODE;
    }
    if((ptVi_Msg -> hwVin_Get > MAX_VOL) || (ptVi_Msg -> hwCur_Set > MAX_CUR)) {
        return OVER_MODE;
    } else {
        tCwb_Can_Tx.id = chCan_Id + 60;
        tCwb_Can_Tx.data[0] = ptVi_Msg -> hwVin_Get / 10000;   //shi wei
        tCwb_Can_Tx.data[1] = ptVi_Msg -> hwVin_Get % 10000 / 1000;    //ge wei
        tCwb_Can_Tx.data[2] = '.';
        tCwb_Can_Tx.data[3] = ptVi_Msg -> hwVin_Get % 10000 % 1000 / 100; //xiaoshuwei
        tCwb_Can_Tx.data[4] = ptVi_Msg -> hwCin_Get / 10000;
        tCwb_Can_Tx.data[5] = ptVi_Msg -> hwCin_Get % 10000 / 1000;
        tCwb_Can_Tx.data[6] = '.';
        tCwb_Can_Tx.data[7] = ptVi_Msg -> hwCin_Get % 10000 % 1000 / 100;
    }
    return COMMON_MODE;
}
//
CWB_MODE_E viout_to_can(CWB_MISC_T *ptVi_Msg)
{
    if(NULL == ptVi_Msg) {
        return ERROR_MODE;
    }
    if((ptVi_Msg -> hwVin_Get > MAX_VOL) || (tCwb_Vi_Msg.hwCur_Set > MAX_CUR)) {
        return OVER_MODE;
    } else {
        tCwb_Can_Tx.id = chCan_Id + 120;
        tCwb_Can_Tx.data[0] = ptVi_Msg -> hwVout_Get / 10000;   //shi wei
        tCwb_Can_Tx.data[1] = ptVi_Msg -> hwVout_Get % 10000 / 1000;    //ge wei
        tCwb_Can_Tx.data[2] = '.';
        tCwb_Can_Tx.data[3] = ptVi_Msg -> hwVout_Get % 100; //xiaoshuwei
        tCwb_Can_Tx.data[4] = ptVi_Msg -> hwCout_Get / 10000;
        tCwb_Can_Tx.data[5] = ptVi_Msg -> hwCout_Get % 10000 / 1000;
        tCwb_Can_Tx.data[6] = '.';
        tCwb_Can_Tx.data[7] = ptVi_Msg -> hwCout_Get % 100;
    }
    return COMMON_MODE;
}
//
bool cwb_can_send(uint8_t chId)
{
    if(chId == (encode_read() + 60)) {
        if(COMMON_MODE == viin_to_can(&tCwb_Vi_Msg)) {
            cwb_can_write(&tCwb_Can_Tx);
        } else {
            printf("Oversize!\r\n");
            return FALSE;
        } 
    } else if(chId == (encode_read() + 120)) {
        if(COMMON_MODE == viout_to_can(&tCwb_Vi_Msg)) {
            cwb_can_write(&tCwb_Can_Tx);
        } else {
            printf("Oversize!\r\n");
            return FALSE;
        }
    } else {
        printf("Wrong ID!\r\n");
        return FALSE;
    }
    return TRUE;
}

//6.7~13V, but no use
//100% -> 13500
//50% -> 6700
//linear
bool cwb_set_spv_corase(uint16_t hwVol_Mv)
{
    uint32_t wDc = 0;
    if(hwVol_Mv > 13500) {
        return FALSE;
    } else {
        wDc = hwVol_Mv * 100 / 13500;   //add calc;
        pv_set(wDc);
        return TRUE;
    }
}
//
bool cwb_set_eload(uint16_t hwCur_Ma)
{
    uint32_t wDc = 0;
    if(hwCur_Ma > 60000) {
        return FALSE;
    } else {
        ;//add calc
        current_set(wDc);
        return TRUE;
    }
}
//4~13V, but no use
bool cwb_set_spv(uint16_t hwVol_Mv, time_t tDead_Time)
{
    uint16_t hwVol_Adc = 0;
    uint16_t hwCalc = 0;
    uint32_t wDc = 50;
    time_t tTime_Get = time_get(tDead_Time);
    while(time_left(tTime_Get) > 0) {
        cwb_mdelay(5);
        //hwVol_Adc = D2MV(hwAdc_Converted_Value[0][1]);
        hwCalc = 29900 - 10 * hwVol_Adc;
        printf("Adc Vol is %d, Calc Vol is %d\r\n", hwVol_Adc, hwCalc);
        if(hwCalc < (hwVol_Mv - 50)) {
            wDc = (wDc < 100) ? (wDc + 1) : 100;
            pv_set(wDc);
        } else if((hwCalc <= (hwVol_Mv + 50)) && (hwCalc >= (hwVol_Mv - 50))) {  //+-50mv
            printf("!%d %d %d\r\n",hwVol_Adc, hwCalc, hwVol_Mv);
            return TRUE;
        } else if(hwCalc >= (hwVol_Mv + 50)) {
            wDc = (wDc > 0) ? (wDc - 1) : 0;
            pv_set(wDc);
        }
    }
    return FALSE;
}
//10ms one time
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) { 
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        s_chFlag_10ms_Interrup = 1;
    }
}

//
static void cwb_init(void)
{
    cwb_scan_init(100); //10ms scan
    time3_init(72000); //72K
    cwb_drive_init();
    cwb_adc_init();
    cwb_can_init();
    if(can_filter_set(encode_read(), 3)) {
        led_flash(LED_YELLOW);
    } else {
        printf("ENCODE Oversize 60!\r\n");
        led_on(LED_RED);
    }
    printf("cwb sw v1.0, build: %s %s\n\r", __DATE__, __TIME__);
    memset(&tCwb_Vi_Msg, 0, sizeof(tCwb_Vi_Msg));
}

void cwb_updata(void)
{
}
int main(void)
{
    task_Init();
    cwb_init();
    while(1){
        task_Update();
        cwb_updata();
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
#endif
#endif
typedef void (*reset)(void);
static int cmd_cwb_func(int argc, char *argv[])
{
    const char *usage = {
        "cwb usage : \r\n"
        "cwb reset\r\n"
        "cwb current dc value : value is dc\r\n"
        "cwb pv dc value : value is dc\r\n"
        "cwb read encode\r\n"
        "cwb set/reset led x y : product or debug, green or red\r\n"
        "cwb enable/disable spv\r\n"
        "cwb enable/disable elodd\r\n"
        "cwb enable/disable sig\r\n"
        "cwb adc\r\n"
        "cwb spv x : x is mv\r\n"
        "cwb eload y : y is ma\r\n"
        "cwb set ID x : x is 0 ~ 59, usually begin from 1 ~ 60\r\n"
        "cwb send to x : x is other ID\r\n"
        "cwb send fv fc to x : voltage and current in float\r\n"
        "                    x = ID + 60, INPUT\r\n"
        "                    x = ID + 120, OUTPUT\r\n"
        "cwb recv id :\r\n"
    };
    if(argc < 2){
        printf("%s", usage);
        return 0;
    }
    if(2 == argc) {
        if(!strcmp(argv[1], "reset")) {
            GenerateSystemReset();
        }
        if(!strcmp(argv[1], "adc")) {
            printf("%d,%d %d,%d %d,%d %d,%d \r\n", adc_pa0_get(), D2MV(adc_pa0_get()), 
                                                   adc_pa1_get(), D2MV(adc_pa1_get()),
                                                   adc_pa2_get(), D2MV(adc_pa2_get()),
                                                   adc_pa3_get(), D2MV(adc_pa3_get()));
        }
    }
    if(3 == argc) {
        if(!strcmp(argv[1], "read") && !strcmp(argv[2], "encode")) {
            printf("%d\r\n", encode_read());
        }
        if(!strcmp(argv[1], "enable")) {
            if(!strcmp(argv[2], "spv")) {
                cwb_en_spv_set(FALSE);  //negative logic
                printf("ENABLE SPV OK!\r\n");
            } else if(!strcmp(argv[2], "eload")) {
                cwb_en_eload_set(TRUE);
                printf("ENABLE ELOAD OK!\r\n");
            } else if(!strcmp(argv[2], "sig")) {
                cwb_sig_set(TRUE);
                printf("ENABLE SIG OK!\r\n");
            }
        } else if(!strcmp(argv[1], "disable")) {
            if(!strcmp(argv[2], "spv")) {
                cwb_en_spv_set(TRUE); //negative logic
                printf("DISABLE SPV OK!\r\n");
            } else if(!strcmp(argv[2], "eload")) {
                cwb_en_eload_set(FALSE);
                printf("DISABLE ELOAD OK!\r\n");
            } else if(!strcmp(argv[2], "sig")) {
                cwb_sig_set(FALSE);
                printf("DISABLE SIG OK!\r\n");
            }
        }
        if(!strcmp(argv[1], "spv")) {
            uint32_t wVol_Set = 0;
            sscanf(argv[2], "%d", &wVol_Set);
            if(cwb_set_spv_corase((uint16_t)wVol_Set)) {
                printf("Please measure!\r\n");
            } else {
                printf("Set oversize the max!\r\n");
            }
        }
        if(!strcmp(argv[1], "current")) {
            uint32_t wCur_Set = 0;
            sscanf(argv[2], "%d", &wCur_Set);
            if(cwb_set_eload((uint16_t)wCur_Set)) {
                printf("Please measure!\r\n");
            } else {
                printf("Set Oversize the max!\r\n");
            }
        }
        if(!strcmp(argv[1], "recv")) {
            uint32_t wId;
            can_msg_t tMsg = {0,8,{0,0,0,0,0,0,0,0},0};
            sscanf(argv[2], "%d", &wId);
            chCan_Id = (uint8_t)wId;
            if(can_filter_set(chCan_Id, 3)) {
                printf("Set Filter Ok!\r\n");
            } else {
                printf("Set Filter Fail!\r\n");
            }
            cwb_mdelay(200);    //must delay, else no function
            if(cwb_can_read(&tMsg)) {
                //can_msg_print(&tMsg,"\n");
                if(COMMON_MODE == can_to_viset(&tMsg)) {
                    printf("Vol is %dmv\r\n", tCwb_Vi_Msg.hwVol_Set);
                    printf("Cur is %dma\r\n", tCwb_Vi_Msg.hwCur_Set);
                } else {
                    printf("ERROR_MODE\r\n");
                }
            } else {
                printf("No read!\r\n");
            }
        }
    }
    if(4 == argc) {
        if(!strcmp(argv[2], "dc")) {
            uint32_t wDc = 0;
            sscanf(argv[3], "%d", &wDc);
            if(!strcmp(argv[1], "current")) {
                current_set(wDc);
            } else if(!strcmp(argv[1], "pv")) {
                pv_set(wDc);
            }
            printf("OK!\r\n");
        }
        if(!strcmp(argv[1], "set") && !strcmp(argv[2], "id")) {
            uint32_t wCan_Id;
            sscanf(argv[3], "%d", &wCan_Id);  
            chCan_Id = (uint8_t)wCan_Id;
            if(can_filter_set(chCan_Id, 3)) {
                printf("Set Filter Ok!\r\n");
            } else {
                printf("Set Filter Fail!\r\n");
            }
        }
        if(!strcmp(argv[1], "send") && !strcmp(argv[2], "to")) {
            uint32_t wOther_Id = 0;
            can_msg_t tMsg = {0,8,{0,0,0,0,0,0,0,0},0};
            sscanf(argv[3], "%d", &wOther_Id);
            tMsg.id = (uint8_t)wOther_Id;
            tMsg.data[0] = 1;   //voltage test
            tMsg.data[1] = 2;
            tMsg.data[2] = '.';
            tMsg.data[3] = 5;
            tMsg.data[4] = 2;   //current test
            tMsg.data[5] = 0;
            tMsg.data[6] = '.';
            tMsg.data[7] = 5;
            if(cwb_can_write(&tMsg)) {
                printf("send OK!\r\n");
            } else {
                printf("send fail!\r\n");
            }
        }
    }
    if(5 == argc) {
        if(!strcmp(argv[1], "set") && !strcmp(argv[2], "led")) {
            if(!strcmp(argv[3], "debug")) {
                if(!strcmp(argv[4], "green")) {
                    led_on(LED_GREEN);
                } else if(!strcmp(argv[4], "red")) {
                    led_on(LED_RED);
                }
                printf("debug set led OK!\r\n");
            } else if(!strcmp(argv[3], "product")) {
                if(!strcmp(argv[4], "green")) {
                    product_set_led(LED_GREEN, LED_ON);
                } else if(!strcmp(argv[4], "red")) {
                    product_set_led(LED_RED, LED_ON);
                }
                printf("product set led OK!\r\n");
            }
            printf("OK!\r\n");
        }
        if(!strcmp(argv[1], "reset") && !strcmp(argv[2], "led")) {
            if(!strcmp(argv[3], "debug")) {
                if(!strcmp(argv[4], "green")) {
                    led_off(LED_GREEN);
                } else if(!strcmp(argv[4], "red")) {
                    led_off(LED_RED);
                }
                printf("debug reset led OK!\r\n");
            } else if(!strcmp(argv[3], "product")) {
                if(!strcmp(argv[4], "green")) {
                    product_set_led(LED_GREEN, LED_OFF);
                } else if(!strcmp(argv[4], "red")) {
                    product_set_led(LED_RED, LED_OFF);
                }
                printf("product reset led OK!\r\n");
            }
        }
    }
    if(6 == argc) {
        if(!strcmp(argv[1], "send") && !strcmp(argv[4], "to")) {
            float fVol, fCur;
            uint32_t wId;
            sscanf(argv[2], "%f", &fVol);
            sscanf(argv[3], "%f", &fCur);
            sscanf(argv[5], "%d", &wId);
            if((wId <= 120) && (wId > 60)) {    //in
                tCwb_Vi_Msg.hwVin_Get = (uint16_t)(fVol * 1000);
                tCwb_Vi_Msg.hwCin_Get = (uint16_t)(fCur * 1000);
                viin_to_can(&tCwb_Vi_Msg);
                tCwb_Can_Tx.id = (uint8_t)wId;
            } else if((wId > 120) && (wId <= 180)) {    //out
                tCwb_Vi_Msg.hwVout_Get = (uint16_t)(fVol * 1000);
                tCwb_Vi_Msg.hwCout_Get = (uint16_t)(fCur * 1000);
                viout_to_can(&tCwb_Vi_Msg);
                tCwb_Can_Tx.id = (uint8_t)wId;
            } else {
                printf("ID Wrong!\r\n");
            }
            if(cwb_can_write(&tCwb_Can_Tx)) {
                printf("send OK!\r\n");
            } else {
                printf("send fail!\r\n");
            }
        }
    }
    return 0;
}
cmd_t cmd_cwb = {"cwb", cmd_cwb_func, "commands for cwb"};
DECLARE_SHELL_CMD(cmd_cwb)
