/*******************************
 *jiamao.gu@2015.3 initial version
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
#define VREF 2972
#define D2MV(D) ((D) * VREF / 65535)  //2978 is ref voltage
#define MV2D(MV) ((MV) * 4096 / VREF)

//golbal var
uint32_t wCanId = 0;
CWB_MISC_T tCwbViMsg __nvm;
CWB_MISC_T tCwbViMsgFault;
static float s_fVolInCal[10] __nvm;
static float s_fVolOutCal[10] __nvm;
static float s_fCurInCal[10] __nvm;
static float s_fCurOutCal[10] __nvm;
CWB_MODE_E tWorkMode = START;  //START END POLL CFG READ
CWB_STATUS_E tWorkStatus = IDLE; //IDLE BUSY PASS FAIL
ERROR_E tError = ERROR_NO;
bool bflag_three = FALSE;
can_msg_t tCwbCanTx = {0, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 0};
can_msg_t tCwbCanTxFault = {0, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 0};
can_msg_t tCwbCanRx = {0, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 0};
can_msg_t tCwbCanCfg[20]; //for recv multi frame
uint8_t chNumCfg = 0;  //count the num of cfg
uint8_t chNumCfgTotal = 0;
uint16_t BASE_MV_IN = 0;
uint16_t BASE_MV_OUT = 0;
#define HALL_POWER VREF
//#define STEP_MV (60 * HALL_POWER / 5000)
#define STEP_MV 33
//static var
//static uint8_t s_chFlag100msInterrup = 0;
static bool s_bFlagCanInterrup = FALSE; // can receive flag
static bool s_bFlagCanScan = FALSE; // for test can communication
static bool s_bDutTarget = FALSE; //
static bool s_bFlagCurrent = FALSE; //begin test current
//often called fucntions
uint32_t cwb_mdelay(uint32_t wMs);
uint32_t cwb_sdelay(uint8_t chS);
void cwb_update();
//functions
bool cwb_set_spv_corase(uint16_t hwVolMv);
bool cwb_set_eload(uint16_t hwCurMa);
void cwb_mode_choose(void);
bool vi_to_can(CWB_MISC_T *tCwbViMsg, CWB_MODE_E tWorkMode);
bool can_to_vi(CWB_MISC_T *tCwbViMsg, CWB_MODE_E tWorkMode);
bool eight_char_to_four_char(char *chBuf_In, uint8_t *chBufOut);
bool bDebug = FALSE;
uint16_t adc_pa0_get(void);
uint16_t adc_pa1_get(void);
uint16_t adc_pa2_get(void);
uint16_t adc_pa3_get(void);
float vol_out_get(uint16_t hwD);
float vol_in_get(uint16_t hwD);
float cur_out_get(uint16_t hwD);
float cur_in_get(uint16_t hwD);
void cwb_judge_error(void);
void GenerateSystemReset(void);
bool check_dut(void);
bool check_dut_no_power(void);
typedef enum {
    AUTOMATIC,
    MAMUAL = !AUTOMATIC,
} pc_shell_t;
pc_shell_t pc_shell = AUTOMATIC;

//dac1 : PA4  ->  ISET
void dac1_init(uint32_t wOption)
{
    dac_ch1.init(NULL);
}
//dac2 : PA5  -> PV
void dac2_init(uint32_t wOption)
{
    dac_ch2.init(NULL);
}

void __sys_update(void)
{
    cwb_update();
}
//
ERROR_E cwb_test_idle(void)
{
    if(IDLE == tWorkStatus) {
        if(!cwb_set_spv_corase(12500)) {  //init set 12.5v
            tError = ERROR_SPV;
            return tError;
        }
        //led_on(LED_YELLOW); //idle -> on yellow
        cwb_isg_set(FALSE);
    }
    
    return tError;
}

//
ERROR_E cwb_test_busy(void)
{
    static uint8_t s_chCount = 0;
    if(BUSY == tWorkStatus) { //TBD
        if(ERROR_NO != tError) {    //if any error before start, can not start test
            tWorkStatus = FAIL;
            tWorkMode = END;
            printf("tError is %d\n", tError);
            return tError;
        }
        cwb_en_eload_set(FALSE); //enable enload
        bflag_three = FALSE;
        led_off(LED_YELLOW);
        printf("In busy! Current count is %d, all test time is %f\n", s_chCount, tCwbViMsg.fTimesSet);
        if(s_chCount < tCwbViMsg.fTimesSet) {
            s_chCount++;
            if(tError != ERROR_NO) {
                tWorkStatus = FAIL;
                tWorkMode = END;
                return tError;
            } 
        } else {
            s_chCount = 0;  //reach the due to time
            tWorkMode = END;
            if(tError != ERROR_NO) {
                tWorkStatus = FAIL;
            } else {
                tWorkStatus = PASS;
            }
            return tError;
        }
        if(END == tWorkMode) {
            /*if(tError != ERROR_NO) {
                tWorkStatus = FAIL;
            } else {
                tWorkStatus = PASS;
            }*/
            tWorkStatus = FAIL;
            s_chCount = 0;
            printf("break END!\n");
            return tError;
        }
        //set 12.5v
        if(!cwb_set_spv_corase((uint32_t)(tCwbViMsg.fVol1Set * 1000))) {
            printf("Oversize the max value!\r\n");
            tError = ERROR_SPV;
            return tError;
        }
        printf("Set spv 12.5v Ok!\n");
        //set current
        if(!cwb_set_eload((uint16_t)(tCwbViMsg.fCurInSet * 1000))) {
            printf("Current oversize!\n");
        }
        printf("Set Current Ok!\n");
        cwb_mdelay(500);
        s_bFlagCurrent = TRUE;
        //delay 5s if have been broken, return in time
        if(cwb_mdelay((int)tCwbViMsg.fT0Set * 1000 - 500)) {
            s_chCount = 0;
            tWorkMode = END;
            /*if(tError != ERROR_NO) {
                tWorkStatus = FAIL;
            } else {
                tWorkStatus = PASS;
            }*/
            tWorkStatus = FAIL;
            return tError;
        }
        //cwb_mdelay(SPV_FALL);            
        cwb_isg_set(TRUE);
        cwb_mdelay(100);
        //set 9v
        if(!cwb_set_spv_corase((uint32_t)(tCwbViMsg.fVol0Set * 1000))) {
            printf("Oversize the max value!\r\n");
            tError = ERROR_SPV;
            return tError;
        }
        printf("Set spv 9.0v Ok!\n");
        cwb_mdelay(ISG_RISE - 100);
        cwb_isg_set(FALSE);
        //test boost condition
        //once error, break the test
        tCwbViMsgFault.fVoutGet = vol_out_get(adc_pa0_get()); //eload vout -> dut vin
        tCwbViMsgFault.fVinGet = vol_in_get(adc_pa1_get());  //eload vin -> dut vout
        tCwbViMsgFault.fCinGet = cur_in_get(D2MV(adc_pa2_get()));  //eload cur in -> dut cur out
        tCwbViMsgFault.fCoutGet = cur_out_get(D2MV(adc_pa3_get())); //eload cur out -> dut cur in
        if(vol_in_get(adc_pa1_get()) > 12.9) { //TBD
            printf("Boost high, vol is %f\n", vol_in_get(adc_pa1_get()));
            tCwbViMsg.fCountBoostGet++;
            if(tCwbViMsg.fCountBoostGet >= 5) {
                tCwbViMsg.fCountBoostGet = 0;
                tError = ERROR_BOOST_HIGH;
                return tError;
            }
        } else if(vol_in_get(adc_pa1_get()) > 11.5) { //TBD
            tCwbViMsg.fCountBoostGet = 0;
            printf("Boost normal, vol is %f\n", vol_in_get(adc_pa1_get()));
        } else if(vol_in_get(adc_pa1_get()) > 9.5) { //TBD
            printf("Boost lower, vol is %f\n", vol_in_get(adc_pa1_get()));
            tCwbViMsg.fCountBoostGet++;
            if(tCwbViMsg.fCountBoostGet >= 5) {
                tCwbViMsg.fCountBoostGet = 0;
                tError = ERROR_BOOST_LOW;
                return tError;
            }
        } else {
            printf("Can not boost, vol is %f\n", vol_in_get(adc_pa1_get()));
            tCwbViMsg.fCountBoostGet++;
            if(tCwbViMsg.fCountBoostGet >= 5) {
                tCwbViMsg.fCountBoostGet = 0;
                tError = ERROR_CAN_NOT_BOOST;
                return tError;
            }
        }
        for(uint16_t hwIndex = 10; hwIndex < (tCwbViMsg.fT1Set * 1000 + 1); hwIndex += 20) {
            cwb_set_spv_corase((uint16_t)tCwbViMsg.fVol0Set * 1000 + (13400 - (uint16_t)tCwbViMsg.fVol0Set * 1000) * hwIndex / (uint16_t)(tCwbViMsg.fT1Set * 1000));
            if(cwb_mdelay(20)) {
                s_chCount = 0;
                tWorkMode = END;
                /*if(tError != ERROR_NO) {
                    tWorkStatus = FAIL;
                } else {
                    tWorkStatus = PASS;
                }*/
                tWorkStatus = FAIL;
                return tError;
            }
        }
        //set 13.5, delay 50s
        bflag_three = TRUE;
        if(cwb_mdelay((uint32_t)(tCwbViMsg.fT2Set * 1000))) {
            bflag_three = FALSE;
            s_chCount = 0;
            tWorkMode = END;
            /*if(tError != ERROR_NO) {
                tWorkStatus = FAIL;
            } else {
                tWorkStatus = PASS;
            }*/
            tWorkStatus = FAIL;
            return tError;
        }
        s_bFlagCurrent = FALSE;
    }
    
    return tError;
}
//Maybe only exec once, because the upper computer will in poll mode
ERROR_E cwb_test_end(void)
{
    if(END == tWorkMode) {
        bflag_three = FALSE;
        cwb_en_eload_set(TRUE); //disable elaod first
        cwb_set_spv_corase(0);  //then disable spv        
        s_bFlagCurrent = FALSE; //cur messure flag
        tCwbViMsg.fCountBypassGet = 0.0; //clear the count
        tCwbViMsg.fCountCurOutGet = 0.0;
        if(FAIL == tWorkStatus) { 
            led_off(LED_YELLOW); //must add, else red will light continue
            led_on(LED_RED);
            cwb_mdelay(1);
        } else if(PASS == tWorkStatus) {
            led_off(LED_YELLOW);
            led_on(LED_GREEN);
            cwb_mdelay(1);
        } else {
        }
    }
    
    return tError;
}
//
bool cwb_mode_poll(void)
{
    if(POLL == tWorkMode) {
        printf("In poll!\r\n");
        printf("Error code is %d\r\n", tError);
        tCwbCanTx.id = tCwbCanRx.id;
        tCwbCanTx.data[0] = (uint8_t)tError;
        tCwbCanTx.data[2] = (uint8_t)tWorkStatus;
        cwb_can_write(&tCwbCanTx);
        tWorkMode = START;
         if(FAIL == tWorkStatus) {
            led_off(LED_YELLOW);
            led_on(LED_RED);
            cwb_mdelay(1);
        } else if(PASS == tWorkStatus) {
            led_off(LED_YELLOW);
            led_on(LED_GREEN);
            cwb_mdelay(1);
        } else {
            //led_on(LED_YELLOW);
            //tWorkStatus = IDLE;
        }
    }
    
    return TRUE;
}
//
bool cwb_mode_cfg(void)
{
    if(CFG == tWorkMode) {
        printf("In cfg!\r\n");
        if((chNumCfgTotal + 1) == chNumCfg) { //recv over 12 bytes; 1 -> c 
            nvm_save();
            chNumCfg = 0;
            if(can_to_vi(&tCwbViMsg, tWorkMode)) { //maybe cfg error
                if(ERROR_CFG != tError) {
                    tCwbCanTx.data[0] = 0x00;
                    nvm_save();
                } else {
                    tCwbCanTx.data[0] = 0x01; //cfg error
                }
            }
            tCwbCanTx.id = tCwbCanRx.id;
            cwb_can_write(&tCwbCanTx);
        }
        tWorkMode = START;
    }
    
    return TRUE;
}
//
bool cwb_mode_read(void)
{
    if(READ == tWorkMode) {
        printf("In read!\n");
        vi_to_can(&tCwbViMsg, READ);
        cwb_can_write(&tCwbCanTx);    //no checksum
        cwb_mdelay(2);
        tWorkMode = START;
    }
    
    return TRUE;
}
//
bool cwb_mode_fault(void)
{
    if(FAULT == tWorkMode) {
        printf("In fault!\n");
        vi_to_can(&tCwbViMsgFault, FAULT);
        cwb_can_write(&tCwbCanTxFault);    //no checksum
        cwb_mdelay(2);
        tWorkMode = START;
    }
    
    return TRUE;
}
//step1 set the spv
bool cwb_mode_hand1(void)
{
    if(HAND1 == tWorkMode) {
        printf("In hand1, set power 12.5v, wait for check voltage!\n");\
        if(cwb_set_spv_corase(12500)) {
        } else {
            return FALSE;
        }
        tCwbCanTx.id = tCwbCanRx.id;
        tCwbCanTx.data[0] = 2;
        cwb_can_write(&tCwbCanTx);    //no checksum
        cwb_mdelay(2);
        tWorkMode = START;
    }

    return TRUE;
}
//step2 judge target
bool cwb_mode_hand(void)
{
    if(HAND == tWorkMode) {
        printf("In Hand!\n");
        tWorkStatus = IDLE;
        tCwbCanTx.id = tCwbCanRx.id;
        if(check_dut_no_power()) {
            tCwbCanTx.data[0] = 1;
        } else {
            tCwbCanTx.data[0] = 2;
        }
        cwb_can_write(&tCwbCanTx);    //no checksum
        cwb_mdelay(2);
        tWorkMode = START;
        cwb_set_spv_corase(0); //after judge, must set spv 0
    }
    return TRUE;
}
//
void cwb_mode_reset(void)
{
  /*if(REBOOT == tWorkMode) {
      GenerateSystemReset();
  }*/
}
//
void get_vi_100ms(void)
{
      static uint8_t s_chCount = 0;
      static float s_fCur = 0.0;
      //s_chFlag100msInterrup = FALSE;
      tCwbViMsg.fVoutGet = vol_out_get(adc_pa0_get()); //eload vout -> dut vin
      tCwbViMsg.fVinGet = vol_in_get(adc_pa1_get());  //eload vin -> dut vout
      tCwbViMsg.fCinGet = cur_in_get(D2MV(adc_pa2_get()));  //eload cur in -> dut cur out
      tCwbViMsg.fCoutGet = cur_out_get(D2MV(adc_pa3_get())); //eload cur out -> dut cur in
      //tCwbViMsg.fRatioGet = 1.0 * vol_in_get(adc_pa0_get()) * cur_in_get(adc_pa2_get()) / vol_out_get(adc_pa1_get()) * cur_out_get(adc_pa3_get());
      if(s_bFlagCurrent) { //must wait for current steady
          s_fCur += tCwbViMsg.fCinGet; //to get the average eload in current(dut out)
          s_chCount++;
          if(10 <= s_chCount) {
              s_chCount = 0;
              tCwbViMsg.fCinGet = s_fCur / 10;
              s_fCur = 0.0;
              if(bDebug) {  //display the value when in debug mode
                  printf("average current is %f\n", tCwbViMsg.fCinGet);
              }
              cwb_judge_error();
          }
      }
      /*if((10 == s_chCount) && (BUSY == tWorkStatus)) { //1s && busy
          s_chCount = 0; //clear count
          tCwbViMsg.fCinGet = s_fCur / 10;  //calc average
          s_fCur = 0.0;
          if(bDebug) {  //display the value when in debug mode
              printf("average current is %f\n", tCwbViMsg.fCinGet);
          }
          cwb_judge_error();
      }*/
}
//
void test_update(void)
{
    //cwb_test_idle();
    cwb_mode_reset(); //reset my mcu
    cwb_mode_hand1(); //step1 set spv
    cwb_mode_hand();  //step2 judge target
    cwb_test_busy();  //test begin : no led
    cwb_test_end();   //test end : light red or green
}
//
uint32_t cwb_mdelay(uint32_t wMs)
{
    uint32_t wLeft;
    time_t tDeadline = time_get(wMs);
    do {
            wLeft = time_left(tDeadline);
            if(wLeft >= 10) { //system update period is expected less than 10ms
                ulp_update();
            }
            if(END == tWorkMode) { //no need delay complete
                return 1;
            }
    } while(wLeft > 0);
    return 0;
}
//
uint32_t cwb_sdelay(uint8_t chS)
{
    for(uint8_t chI = 0; chI < chS; chI++) {
        cwb_mdelay(1000);
    }
    return 0;
}
//Vout
uint16_t adc_pa0_get(void)
{
    return ADC1-> JDR1 << 1;
}
//translate to voltage out (v)
float vol_out_get(uint16_t hwD)
{
    #if 0
    return (VREF - D2MV(hwD)) * 10.0 / 1000; //10 is the gain of AD628
    #endif
    return D2MV(hwD) * 6.0 / 1000; //6.0 is the gain 
}
//Vin
uint16_t adc_pa1_get(void)
{
    return ADC1-> JDR2 << 1;
}
//translate to voltage in (v)
float vol_in_get(uint16_t hwD)
{
    #if 0
    return (VREF - D2MV(hwD)) * 10.0 / 1000; //10 is the gain of AD628
    #endif
    return D2MV(hwD) * 6.0 / 1000; //6.0 is the gain
}
//Iin()
uint16_t adc_pa2_get(void)
{
    return ADC1-> JDR3 << 1;
}
//translate to current in (A) Power is 3V3
//#define STEP_MV 39
//#define BASE_MV 395
float cur_in_get(uint16_t hwA)
{
    if(0 != s_fCurInCal[0]) {
        if(hwA <= 15000) {
            BASE_MV_IN = (uint16_t)s_fCurInCal[0];
        } else if(hwA <= 20000) {
            BASE_MV_IN = (uint16_t)s_fCurInCal[1];
        } else if(hwA <= 25000) {
            BASE_MV_IN = (uint16_t)s_fCurInCal[2];
        } else if(hwA <= 30000) {
            BASE_MV_IN = (uint16_t)s_fCurInCal[3];
        } else if(hwA <= 35000) {
            BASE_MV_IN = (uint16_t)s_fCurInCal[4];
        } else {
            BASE_MV_IN = (uint16_t)s_fCurInCal[5];
        }
    }
    return (hwA - BASE_MV_IN) * 1.0 / STEP_MV;   //
}
//Iout
uint16_t adc_pa3_get(void)
{
    return ADC1-> JDR4 << 1;
}
//translate to current out (A)
//60mv/A
float cur_out_get(uint16_t hwA)
{
    /*if(0 != s_fCurOutCal[0]) {
        if(hwA <= 15000) {
            BASE_MV_OUT = s_fCurOutCal[0];
        } else if(hwA <= 20000) {
            BASE_MV_OUT = s_fCurOutCal[1];
        } else if(hwA <= 25000) {
            BASE_MV_OUT = s_fCurOutCal[2];
        } else if(hwA <= 30000) {
            BASE_MV_OUT = s_fCurOutCal[3];
        } else if(hwA <= 35000) {
            BASE_MV_OUT = s_fCurOutCal[4];
        } else {
            BASE_MV_OUT = s_fCurOutCal[5];
        }
    }*/
    return (hwA - BASE_MV_OUT) * 1.0 / STEP_MV;   //
}
uint16_t adc_nul_get(void)
{
    return 0;
}

//
bool cwb_can_send(uint8_t chId)
{
    return TRUE;
}
//
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    if(CAN_MessagePending(CAN1, CAN_FIFO0) > 0) {
        s_bFlagCanInterrup = TRUE;
        s_bFlagCanScan = TRUE;
        cwb_can_read(&tCwbCanRx);
        //can_msg_print(&tCwbCanRx, "\n");
    }
}
//
//CANID[11:8] [7:0]
//function  address
void cwb_mode_choose(void)
{
    if(s_bFlagCanInterrup) {
        s_bFlagCanInterrup = FALSE;
        switch(tCwbCanRx.id >> 8 & 0x7) { //
            case START:
                tWorkMode = START;
                tCwbCanTx.id = tCwbCanRx.id;
                tCwbCanTx.data[0] = 1;  //response
                cwb_can_write(&tCwbCanTx);
                printf("Receive Start, begin test!\n");
                printf("Clear all the error!\n");
                tError = ERROR_NO;
                tWorkStatus = BUSY;
                chNumCfg = 0;
                break;
            case END:
                tWorkMode = END;
                tCwbCanTx.id = tCwbCanRx.id;
                tCwbCanTx.data[0] = 1;  //response
                cwb_can_write(&tCwbCanTx);
                printf("Receive end, end test!\n");
                chNumCfg = 0;
                break;
            case POLL:
                tWorkMode = POLL;
                chNumCfg = 0;
                printf("Will in poll!\n");
                break;
            case CFG:
                tWorkMode = CFG; 
                if(chNumCfg > 20) {    //can not over 20, in fact
                    chNumCfg = 0;
                }
                tCwbCanCfg[chNumCfg].id = tCwbCanRx.id;
                tCwbCanCfg[chNumCfg].dlc = tCwbCanRx.dlc;
                memcpy(tCwbCanCfg[chNumCfg].data, tCwbCanRx.data, tCwbCanRx.dlc);
                tCwbCanCfg[chNumCfg].flag = tCwbCanRx.flag;
                can_msg_print(&tCwbCanCfg[chNumCfg], "\n");
                if(0 == chNumCfg) {
                    chNumCfgTotal = tCwbCanCfg[0].data[7] + 1; //1 -> last CRC bit
                    printf("The left can message is %d\n", chNumCfgTotal);
                }
                chNumCfg++;
                break;
            case READ:
                tWorkMode = READ;
                chNumCfg = 0;
                break;
            case HAND:
                tWorkMode = HAND;
                chNumCfg = 0;
                break;
            case FAULT:
                tWorkMode = FAULT;
                break;
            case HAND1: //power on
                tWorkMode = HAND1;
                break;
            case ENDING:
                tWorkMode = ENDING;
                break;
            default:
                chNumCfg = 0;
        }
    }
}
void cwb_judge_error(void)
{
    //only judge 50s && NO ERROR(because if exist error, no need to judge)
    if(bflag_three && (ERROR_NO == tError)) {
        /*if((tCwbViMsg.fVinGet > (13.4 + tCwbViMsg.fVolDeltaLimit)) || (tCwbViMsg.fVinGet < (13.4 - tCwbViMsg.fVolDeltaLimit))) {
            tCwbViMsg.fCountVolInGet++;
            if(tCwbViMsg.fCountVolInGet > tCwbViMsg.fCountVolLimit) {
                printf("Vol not stable!\n");
                tCwbViMsg.fCountVolInGet = 0.0;
                tError = ERROR_VOL_NOT_STABLE;
                bflag_three = FALSE;
            }
        }*/
        /*if((tCwbViMsg.fVoutGet > (tCwbViMsg.fVol1Set + tCwbViMsg.fVolDeltaLimit)) || (tCwbViMsg.fVoutGet < (tCwbViMsg.fVol1Set - tCwbViMsg.fVolDeltaLimit))) {
            tCwbViMsg.fCountVolOutGet++;
            if(tCwbViMsg.fCountVolOutGet > tCwbViMsg.fCountVolLimit) {
                //tError = ERROR_VOL_OUT;
            }
        }*/
        //when no bypass, the vol in(DUT) > vol out(DUT) + 0.6
        if(tCwbViMsg.fVoutGet > (tCwbViMsg.fVinGet + 0.8)) {
            tCwbViMsg.fCountBypassGet++;
            if(tCwbViMsg.fCountBypassGet > tCwbViMsg.fCountVolLimit) {
                printf("Vol in get is %f\n", tCwbViMsg.fVinGet);
                printf("Vol out get is %f\n", tCwbViMsg.fVoutGet);
                memcpy(&tCwbViMsgFault, &tCwbViMsg, sizeof(CWB_MISC_T));
                printf("By pass error!\n");
                tError = ERROR_BYPASS;
                tCwbViMsg.fCountBypassGet = 0;
                bflag_three = FALSE;
            }
        } else {
            tCwbViMsg.fCountBypassGet = 0;
        }
        //printf("%f\n", tCwbViMsg.fCinGet);
        if(tCwbViMsg.fCinGet < (tCwbViMsg.fCurInSet - tCwbViMsg.fCurInDeltaLimit)) {
            tCwbViMsg.fCountCurInGet++;
            if(tCwbViMsg.fCountCurInGet > tCwbViMsg.fCountCurLimit) {
                memcpy(&tCwbViMsgFault, &tCwbViMsg, sizeof(CWB_MISC_T));
                printf("Cur error!\n");
                tError = ERROR_CUR_IN;
                tCwbViMsg.fCountCurInGet = 0;
                bflag_three = FALSE;
            }
        } else {
            tCwbViMsg.fCountCurInGet = 0;
        }
        if(ERROR_NO != tError) {
            tWorkMode = END;
            tWorkStatus = FAIL;
        }
        /*if((tCwbViMsg.fCoutGet > (tCwbViMsg.fCurOutSet + tCwbViMsg.fCurOutDeltaLimit)) || (tCwbViMsg.fCoutGet < (tCwbViMsg.fCurOutSet - tCwbViMsg.fCurOutDeltaLimit))) {
            tCwbViMsg.fCountCurInGet++;
            if(tCwbViMsg.fCountCurOutGet > tCwbViMsg.fCountCurLimit) {
                //tError = ERROR_CUR_OUT;
            }
        }*/
    }
}
//
uint32_t add_checksum(char *pchWrMsg)
{
    uint32_t wVarCheckSum = 0;
    uint32_t wCheckSum;
    for (uint32_t wCpt = 0; wCpt < (chNumCfg - 1); wCpt++) { //besides Check_Sum
        for(uint8_t chIndex = 0; chIndex < 8; chIndex++) {
            wVarCheckSum += tCwbCanCfg[wCpt].data[chIndex];
        }
    }
    wCheckSum = 0x100 - (wVarCheckSum % 0x100);
    return wCheckSum;
}
//
#define RATIO 0
#define VIN 1
#define VOUT 2
#define CIN 3
#define COUT 4
bool vi_to_can(CWB_MISC_T *tCwbViMsg, CWB_MODE_E tWorkMode)
{
    if(NULL == tCwbViMsg) {
        return FALSE;
    }
    if(READ == tWorkMode) {
        tCwbCanTx.id = tCwbCanRx.id;
        switch(tCwbCanRx.data[0]) {
            case RATIO:
                tCwbViMsg -> fRatioGet = 80.1; // for test
                memcpy(&tCwbCanTx.data, &tCwbViMsg -> fRatioGet, sizeof(tCwbViMsg -> fRatioGet));
                printf("Ratio is %f\r\n", *(float *)tCwbCanTx.data);
                break;
            case VIN:
                //tCwbViMsg -> fVinGet = 12.5; // for test
                memcpy(&tCwbCanTx.data, &tCwbViMsg -> fVinGet, sizeof(tCwbViMsg -> fVinGet));
                printf("Input voltage is %f\r\n", *(float *)tCwbCanTx.data);
                break;
            case VOUT:
                //tCwbViMsg -> fVoutGet = 11.5; // for test
                memcpy(&tCwbCanTx.data, &tCwbViMsg -> fVoutGet, sizeof(tCwbViMsg -> fVoutGet));
                printf("Output voltage is %f\r\n", *(float *)tCwbCanTx.data);
                break;
            case CIN:
                //tCwbViMsg -> fCinGet = 20.5; // for test
                memcpy(&tCwbCanTx.data, &tCwbViMsg -> fCinGet, sizeof(tCwbViMsg -> fCinGet));
                printf("Input current is %f\r\n", *(float *)tCwbCanTx.data);
                break;
            case COUT:
                //tCwbViMsg -> fCoutGet = 30.5; // for test
                memcpy(&tCwbCanTx.data, &tCwbViMsg -> fCoutGet, sizeof(tCwbViMsg -> fCoutGet));
                printf("Output current is %f\r\n", *(float *)tCwbCanTx.data);
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
                    printf("Fault Input voltage is %f\r\n", *(float *)tCwbCanTxFault.data);
                    break;
                case VOUT:
                    //tCwbViMsgFault -> fVoutGet = 11.5; // for test
                    memcpy(&tCwbCanTxFault.data, &tCwbViMsg -> fVoutGet, sizeof(tCwbViMsg -> fVoutGet));
                    printf("Fault Output voltage is %f\r\n", *(float *)tCwbCanTxFault.data);
                    break;
                case CIN:
                    //tCwbViMsgFault -> fCinGet = 20.5; // for test
                    memcpy(&tCwbCanTxFault.data, &tCwbViMsg -> fCinGet, sizeof(tCwbViMsg -> fCinGet));
                    printf("Fault Input current is %f\r\n", *(float *)tCwbCanTxFault.data);
                    break;
                case COUT:
                    //tCwbViMsgFault -> fCoutGet = 30.5; // for test
                    memcpy(&tCwbCanTxFault.data, &tCwbViMsg -> fCoutGet, sizeof(tCwbViMsg -> fCoutGet));
                    printf("Fault Output current is %f\r\n", *(float *)tCwbCanTxFault.data);
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
        if((chBufIn[chIndex] <= 9) && (chBufIn[chIndex] >= 0)) {
            chBufIn[chIndex] += '0';
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
        tCwbViMsg -> fVol0Set = *(float *)chBuf;//VOL0
        printf("VOL0 SET is %f\n", tCwbViMsg -> fVol0Set);
        
        eight_char_to_four_char(tCwbCanCfg[2].data, chBuf);
        tCwbViMsg -> fVol1Set = *(float *)chBuf;//VOL1
        printf("VOL1 SET is %f\n", tCwbViMsg -> fVol1Set);
        
        eight_char_to_four_char(tCwbCanCfg[3].data, chBuf);
        tCwbViMsg -> fVolDeltaLimit = *(float *)chBuf;//VOL_DELTA
        printf("VOL_DELTA is %f\n", tCwbViMsg -> fVolDeltaLimit);
        
        eight_char_to_four_char(tCwbCanCfg[4].data, chBuf);
        tCwbViMsg -> fT0Set = *(float *)chBuf;//T0
        printf("T0 SET is %f\n", tCwbViMsg -> fT0Set);
        
        eight_char_to_four_char(tCwbCanCfg[5].data, chBuf);
        tCwbViMsg -> fT1Set = *(float *)chBuf;//T1
        printf("T1 SET is %f\n", tCwbViMsg -> fT1Set);
        
        eight_char_to_four_char(tCwbCanCfg[6].data, chBuf);
        tCwbViMsg -> fT2Set = *(float *)chBuf;//T2
        printf("T2 SET is %f\n", tCwbViMsg -> fT2Set);
        
        eight_char_to_four_char(tCwbCanCfg[7].data, chBuf);
        tCwbViMsg -> fPower = *(float *)chBuf;//Power
        tCwbViMsg -> fCurInSet = tCwbViMsg -> fPower / tCwbViMsg -> fVol1Set;
        tCwbViMsg -> fCurOutSet = tCwbViMsg -> fPower / tCwbViMsg -> fVol1Set;
        printf("Power is %f %f %f\n", tCwbViMsg -> fPower, tCwbViMsg -> fCurInSet, tCwbViMsg -> fCurOutSet);
        
        eight_char_to_four_char(tCwbCanCfg[8].data, chBuf);
        tCwbViMsg -> fCountCurLimit = *(float *)chBuf;//COUNT_C
        printf("CUR IN COUNT LIMIT is %f\n", tCwbViMsg -> fCountCurLimit);
        
        eight_char_to_four_char(tCwbCanCfg[9].data, chBuf);
        tCwbViMsg -> fCountVolLimit = *(float *)chBuf;//COUNT_V
        printf("VOL IN COUNT LIMIT is %f\n", tCwbViMsg -> fCountVolLimit);
        
        eight_char_to_four_char(tCwbCanCfg[10].data, chBuf);
        tCwbViMsg -> fCurInDeltaLimit = *(float *)chBuf;//CUR_IN_DELTA
        printf("CUR_IN_DELTA_LIMIT is %f\n", tCwbViMsg -> fCurInDeltaLimit);
        
        eight_char_to_four_char(tCwbCanCfg[11].data, chBuf);
        tCwbViMsg -> fCurOutDeltaLimit = *(float *)chBuf;//CUR_OUT_DELTA
        printf("CUR_OUT_DELTA_LIMIT is %f\n", tCwbViMsg -> fCurOutDeltaLimit);
        
        eight_char_to_four_char(tCwbCanCfg[12].data, chBuf);
        tCwbViMsg -> fTimesSet = *(float *)chBuf;//TIME SET
        printf("Test times is %f\n", tCwbViMsg -> fTimesSet);
        
        tCwbViMsg -> hwCheckSum = tCwbCanCfg[13].data[6] * 16 + tCwbCanCfg[13].data[7];//CHECK_SUM
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
            tError = ERROR_CFG;
        }
    }
    return TRUE;
}
//
bool error_get(void)
{
    /*if(tCwbViMsg.hwCheckSum != tCwbViMsg.hwCheckSum_Calc) {
        tError = ERROR_CFG;
    }
    if(tCwbViMsg.fCountVolInGet >= tCwbViMsg.fCount_Vol_In_Limit) {
        tError = ERROR_VOL_OUT;
    }
    if(tCwbViMsg.fCountCurOutGet >= tCwbViMsg.fCount_Cur_Out_Limit) {
        tError = ERROR_CUR_OUT;
    }
    if(tCwbViMsg.fRatioGet >= tCwbViMsg.fRatioLimit) {
        tError = ERROR_RATIO;
    }*/
    
    return TRUE;
}
//
bool cwb_cfg_recv(void)
{
    if(!cwb_can_read(&tCwbCanRx)) {
        return FALSE;
    }
    //can_msg_print(&tCwbCanRx, "\n");
    return TRUE;
}
//6.7~13V, but no use
//100% -> 13500
//50% -> 6700
//linear
bool cwb_set_spv_corase(uint16_t hwVolMv)
{
    #if 0
    uint32_t wDc = 0;
    if(hwVolMv > 13500) {
        return FALSE;
    } else {
        wDc = hwVolMv * 100 / 13500;   //add calc;
        pv_set(wDc);
        return TRUE;
    }
    #endif
    uint32_t wDigital = 0;
    float fVol = 0.0;
    if(hwVolMv > 13500) {
        return FALSE;
    } else {
        fVol = 1.0 * hwVolMv / 5.16;
        wDigital = MV2D((uint16_t)(fVol));
        dac_ch2.write(wDigital);
        return TRUE;
    }
}
//
bool cwb_set_eload(uint16_t hwCurMa)
{
    //********************************0  1  2  3  4    5    6    7    8    9    10   11   12   13   14   15   16  17   18
    const uint16_t c_hwCurRef[19] = {0, 0, 0, 0, 108, 127, 148, 168, 190, 212, 233, 255, 278, 301, 325, 347, 372, 390, 410};
    uint16_t hwMv = 0;
    uint16_t hwCurTmp = 0;
    if((hwCurMa > 45000) || (hwCurMa < 11400)) {
        return FALSE;
    } else {
        hwCurTmp = hwCurMa / 100;
        //printf("%d\n", hwCurTmp);
        for(uint8_t chIndex = 0; chIndex < 18; chIndex++) { //add calc
            if((hwCurTmp > c_hwCurRef[chIndex]) && (hwCurTmp < c_hwCurRef[chIndex + 1])) {
                hwMv = (uint16_t)(100 * (chIndex + 1.0 * (hwCurTmp - c_hwCurRef[chIndex]) / (c_hwCurRef[chIndex + 1] - c_hwCurRef[chIndex]))); 
            }
        }
        //printf("%d\n", hwMv);
        dac_ch1.write(MV2D(hwMv));
        return TRUE;
    }
}
//100ms one time
void TIM2_IRQHandler(void)
{
    static uint32_t s_wCount = 0;
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) { 
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        //printf("1!\n");
        //s_chFlag100msInterrup = 1;
        get_vi_100ms();
        s_wCount++;
        if(500 == s_wCount) { //judge once every 5s
            s_wCount = 0;
            //printf("%d\n", s_bFlagCanScan);
            if(BUSY == tWorkStatus) { //only in busy mode
                if(s_bFlagCanScan) {  //if exist can frame
                    s_bFlagCanScan = FALSE; //clear can flag
                } else {  //if no can frame, means communication error
                    printf("Can communication error!Stop Test!\n");
                    tWorkMode = END;
                    tError = ERROR_CAN;
                }
            }
        }
    }
}
//calc the hall in base voltage
void cwb_get_hall_in()
{
    uint32_t hwSum = 0;
    for(uint8_t chIndex = 0; chIndex < 128; chIndex++) {
        //printf("cur in is %d\r\n", D2MV(adc_pa2_get()));
        hwSum += D2MV(adc_pa2_get());
    }
    //printf("sum in is %d\n", hwSum);
    BASE_MV_IN = hwSum >> 7;
    printf("cur in base is %d\n", BASE_MV_IN);
}
//calc the hall out base voltage
void cwb_get_hall_out()
{
    uint32_t hwSum = 0;
    for(uint8_t chIndex = 0; chIndex < 128; chIndex++) {
        //printf("cur in is %d\r\n", D2MV(adc_pa3_get()));
        hwSum += D2MV(adc_pa3_get());
    }
    BASE_MV_OUT = hwSum >> 7;
    printf("cur out base is %d\n", BASE_MV_OUT);
}
//check dut target or not
//only use in init becase of the delay is too long
bool check_dut(void)
{
    tError = ERROR_NO;
    cwb_set_spv_corase(12500);
    cwb_mdelay(500);
    if(vol_in_get(adc_pa1_get()) > 11.5) {
        printf("Vol is %f, SPV is %f\n", vol_in_get(adc_pa1_get()), vol_out_get(adc_pa0_get()));
        return TRUE;
    } else if(vol_in_get(adc_pa1_get()) > 5.0) {
        //econsider as vol low
        printf("Error : DUT VOL OUT LOW, Vol is %f, SPV is %f\n", vol_in_get(adc_pa1_get()), vol_out_get(adc_pa0_get()));
        tError = ERROR_VOL_IN_LOW;
        return TRUE;
    } else {
        //SPV out is lower, then consider as short
        if(vol_out_get(adc_pa0_get()) < 2.0) {
            printf("Error : DUT short! Vol is %f, SPV is %f\n", vol_in_get(adc_pa1_get()), vol_out_get(adc_pa0_get()));
            tError = ERROR_SHORT;
            return TRUE;
        } else {
            printf("Vol is %f, SPV is %f\n", vol_in_get(adc_pa1_get()), vol_out_get(adc_pa0_get()));
            return FALSE;
        }
    }
}

//check dut target or not
//read the out of dut to judge the target
bool check_dut_no_power(void)
{
    tError = ERROR_NO;
    if(vol_in_get(adc_pa1_get()) > 11.5) {
        printf("Vol is %f, SPV is %f\n", vol_in_get(adc_pa1_get()), vol_out_get(adc_pa0_get()));
        return TRUE;
    } else if(vol_in_get(adc_pa1_get()) > 5.0) {
        //econsider as vol low
        printf("Error : DUT VOL OUT LOW, Vol is %f, SPV is %f\n", vol_in_get(adc_pa1_get()), vol_out_get(adc_pa0_get()));
        tError = ERROR_VOL_IN_LOW;
        tWorkStatus = FAIL;
        return TRUE;
    } else {
        //SPV out is lower, then consider as short
        if(vol_out_get(adc_pa0_get()) < 2.0) {
            printf("Error : DUT short! Vol is %f, SPV is %f\n", vol_in_get(adc_pa1_get()), vol_out_get(adc_pa0_get()));
            tError = ERROR_SHORT;
            tWorkStatus = FAIL;
            return TRUE;
        } else {
            printf("Vol is %f, SPV is %f\n", vol_in_get(adc_pa1_get()), vol_out_get(adc_pa0_get()));
            return FALSE;
        }
    }
}
//
static void cwb_init(void)
{
    cwb_scan_init(10); //100ms scan
    //time3_init(72000); //72K
    dac1_init(0);
    dac2_init(0);
    cwb_drive_init();
    cwb_adc_init();
    cwb_can_init();
    //shell_mute((const struct console_s *) &uart1);
    printf("%d\r\n", encode_read()); //read the encode
    can_filter_set(wCanId);
    cwb_isg_set(FALSE);
    cwb_en_eload_set(TRUE); //disable eload
    if(can_filter_set(encode_read())) {
        //led_flash(LED_YELLOW);
    } else {
        printf("ENCODE Oversize 60!\r\n");
        //led_on(LED_RED);
    }
    printf("cwb sw v1.0, build: %s %s\n\r", __DATE__, __TIME__);
    //cwb_mdelay(2000); //wait for hall steady
    cwb_light_after_power(); //instead 2s delay for hall steady
    cwb_get_hall_in();
    cwb_get_hall_out();
    cwb_en_spv_set(TRUE);
    s_bDutTarget = check_dut();
    if(s_bDutTarget) {
        printf("Dut is on!\n");
    } else {
        printf("Dut is not on!\n");
    }
    cwb_set_spv_corase(0); //set spv 0
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
    printf("%d\n", *p);
}
#endif
#endif
//typedef void (*reset)(void);
typedef void (*fault)(void);
static int cmd_cwb_func(int argc, char *argv[])
{
    const char *usage = {
        "cwb usage : \r\n"
        "cwb reset\r\n"
        "cwb mode x\r\n"
        "cwb nvm : test nvm function\r\n"
        "cwb print : print vi data\r\n"
        "cwb filter x\r\n"
        "cwb send cfg : ID -> 3 << 8 | 50\r\n"
        "cwb recv cfg\r\n"
        "cwb floattohex\r\n"
        "cwb read sn\r\n"
        "cwb cur in\r\n"
        "cwb disable/enable spv\n"
        "cwb disable/enable sig\n"
        "cwb set spv x\n"
        "cwb test fall\n"
        "cwb set led x\n"
        "cwb test rise time: test spv rise time\n"
        "cwb test fall time: test spv fall time\n"
    };
    if(argc < 2){
        printf("%s", usage);
        return 0;
    }
    if(2 == argc) {
        if(!strcmp(argv[1], "reset")) {
            GenerateSystemReset();
        }
        if(!strcmp(argv[1], "fault")) {
            //fault fault1 = (fault)0xfffffff1;
            //(*fault1)();
            hardfault();
        }
        if(!strcmp(argv[1], "adc")) {
            printf("%d,%d %d,%d %d,%d %d,%d \r\n", adc_pa0_get(), D2MV(adc_pa0_get()), 
                                                   adc_pa1_get(), D2MV(adc_pa1_get()),
                                                   adc_pa2_get(), D2MV(adc_pa2_get()),
                                                   adc_pa3_get(), D2MV(adc_pa3_get()));
        } else if(!strcmp(argv[1], "encode")) {
            printf("%d\r\n", encode_read());
        } else if(!strcmp(argv[1], "nvm")) {
            tCwbViMsg.fVol0Set = 9.0;//VOL0
            tCwbViMsg.fVol1Set = 12.5;//VOL1
            tCwbViMsg.fVolDeltaLimit = 0.5;
            tCwbViMsg.fT0Set = 5.0;//T0
            tCwbViMsg.fT1Set = 5.0;//T1
            tCwbViMsg.fT2Set = 50.0;//T2
            tCwbViMsg.fPower = 400.0;
            tCwbViMsg.fCurInSet = tCwbViMsg.fCurOutSet = tCwbViMsg.fPower / 12.5;
            tCwbViMsg.fCountVolLimit = 10.0;//COUNT_V
            tCwbViMsg.fCountCurLimit = 10.0;//COUNT_C
            tCwbViMsg.fCurInDeltaLimit = 2;
            tCwbViMsg.fCurOutDeltaLimit = 2;
            tCwbViMsg.fTimesSet = 2.0;
            nvm_save();
        } else if(!strcmp(argv[1], "print")) {
            printf("Work Mode is %d : 0 -> start 1 -> end 2 -> poll 3 -> cfg 4 -> read 5 -> hand\n", tWorkMode);
            printf("Work Status is %d : 0 -> idle 1 -> busy 2 -> pass 3 -> fail\n", tWorkStatus);
            printf("Work error is %d : 0 -> NO 1 -> CFG 2 -> SPV 3 -> vol_in  4 -> cur_in 5 -> vol_out 6 -> cut_out\n", tError); 
            printf("VOL0 is %f\n", tCwbViMsg.fVol0Set);
            printf("VOL1 is %f\n", tCwbViMsg.fVol1Set);
            printf("Power is %f\n", tCwbViMsg.fPower);
            printf("Cur is %f\n", tCwbViMsg.fCurInSet);
            printf("T0 is %f\n", tCwbViMsg.fT0Set);
            printf("T1 is %f\n", tCwbViMsg.fT1Set);
            printf("T2 is %f\n", tCwbViMsg.fT2Set);
            printf("COUNT_CUR is %f\n", tCwbViMsg.fCountCurLimit);
            printf("COUNT_VOL is %f\n", tCwbViMsg.fCountVolLimit);
            printf("CUR_IN_DELTA is %f\n", tCwbViMsg.fCurInDeltaLimit);
            printf("CUR_OUT_DELTA is %f\n", tCwbViMsg.fCurOutDeltaLimit);
            printf("Test times is %f\n", tCwbViMsg.fTimesSet);
        } else if(!strcmp(argv[1], "floattohex")) {
            float fData = 12.5;
            uint8_t chData[8] = {0,0,0,0,1,0,4,1};
            char chData1[8] = {0,0,0,0,0,0,0,0};
            char chData_Test[8] = {0x30,0x30,0x30,0x30,0x31,0x30,0x34,0x31};
            char chInt[8] = {'0', '0', '0', '0', 'a', '0', '4', '0'};
            uint8_t chTmp1[4];
            eight_char_to_four_char(chInt, chTmp1);
            printf("%x %x %x %x\r\n", chTmp1[0], chTmp1[1], chTmp1[2], chTmp1[3]);
            //uint32_t wA;
            //memcpy(&wA, chTmp1, sizeof(chTmp1));
            float fA = *(float *)chTmp1;
            printf("%f\r\n", fA);
            //uint8_t chData_Test[8] = {0x0,0x0,0x10,0x41,0x0,0x0,0x0,0x0};
            //char chData_Test1[] = "00001041";
            float fData1;
            float fData2;
            memcpy(chData, &fData, sizeof(fData));
            sprintf(chData1, "%c", chData);
            printf("0x%x %x %x %x %x %x %x %x\r\n", chData[0], chData[1], chData[2], chData[3], chData[4], chData[5], chData[6], chData[7]);
            printf("0x%x %x %x %x %x %x %x %x\r\n", chData1[0], chData1[1], chData1[2], chData1[3], chData1[4], chData1[5], chData1[6], chData1[7]);
            fData1 = *(float *)chData1;
            printf("%f\r\n", fData1);
            memcpy(&fData2, chData, sizeof(chData));
            printf("%f\r\n", fData2);
            fData2 = *(float *)chData_Test;
            //uint8_t chTmp[4];
            //sscanf(chData_Test, "%2x%2x%2x%2x", &chTmp[0], &chTmp[1], &chTmp[2], &chTmp[3]);
            //eight_char_to_four_char(chData_Test, chTmp);
            //printf("%x %x %X %x\r\n", chTmp[0], chTmp[1], chTmp[2], chTmp[3]);
            //printf("%f %f\r\n", fData2, *(float *)chTmp);
        } else if(!strcmp(argv[1], "-m")) {
            shell_unmute((const struct console_s *) &uart1);
            pc_shell = MAMUAL;
        } else if(!strcmp(argv[1], "-a")) {
            shell_mute((const struct console_s *) &uart1);
            pc_shell = AUTOMATIC;
        } else if(!strcmp(argv[1], "target")) {
            s_bDutTarget = check_dut();
            if(s_bDutTarget) {
                printf("Dut is on!\n");
            } else {
                printf("Dut is not on!\n");
            }
        }
    }
    if(3 == argc) {
        if(!strcmp(argv[1], "mode")) {
            if(!strcmp(argv[2], "start")) {
                tWorkMode = START;
                tWorkStatus = BUSY;
                printf("In start mode!\r\n");
            } else if(!strcmp(argv[2], "end")) {
                tWorkMode = END;
                printf("In end mode!\r\n");
            } else if(!strcmp(argv[2], "poll")) {
                tWorkMode = POLL;
                printf("In poll mode!\r\n");
            } else if(!strcmp(argv[2], "cfg")) {
                tWorkMode = CFG;
                printf("In cfg mode!\r\n");
            } else if(!strcmp(argv[2], "read")) {
                tWorkMode = READ;
                printf("In read mode!\r\n");
            }
        } else if(!strcmp(argv[1], "filter")) {
            sscanf(argv[2], "%d", &wCanId);
            if(can_filter_set(wCanId)) {
            } else {
                printf("ID oversize!\r\n");
            }
        } else if(!strcmp(argv[1], "send") && !strcmp(argv[2], "cfg")) {
            can_msg_t tMsg0 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xb}, 0}; //0xb -> 11Bytes 
            can_msg_t tMsg1 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x4, 0x1}, 0}; //V0 : 3.5  V1 : 12.5
            can_msg_t tMsg2 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0x4, 0x0, 0x4, 0x1}, 0}; //T0 : 10.4 T1 : 49.6
            can_msg_t tMsg3 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0x8, 0x0, 0x3, 0xf}, 0}; //Vol: 11.1 Voh: 13.1
            can_msg_t tMsg4 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0xa, 0x0, 0x4, 0x0}, 0}; //Eff: 80.1 Is ：25.5
            can_msg_t tMsg5 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0xa, 0x0, 0x4, 0x0}, 0}; //T0 : 10.4 T1 : 49.6
            can_msg_t tMsg6 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0x5, 0xc, 0x4, 0x2}, 0}; //Vol: 11.1 Voh: 13.1
            can_msg_t tMsg7 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0x9, 0x6, 0x4, 0x3}, 0}; //Eff: 80.1 Is ：25.5
            can_msg_t tMsg8 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0xa, 0x0, 0x4, 0x0}, 0}; //Vol: 11.1 Voh: 13.1
            can_msg_t tMsg9 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0xa, 0x0, 0x4, 0x0}, 0}; //V0 : 3.5  V1 : 12.5
            can_msg_t tMsg10 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0x8, 0x0, 0x3, 0xf}, 0}; //T0 : 10.4 T1 : 49.6
            can_msg_t tMsg11 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0x8, 0x0, 0x3, 0xf}, 0}; //Vol: 11.1 Voh: 13.1
            can_msg_t tMsg12 = {3 << 8 | wCanId, 8, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc, 0xd}, 0}; //Vol: 11.1 Voh: 13.1
            if(cwb_can_write(&tMsg0)) {
                printf("OK!\r\n");
            }
            cwb_mdelay(2);
            if(cwb_can_write(&tMsg1)) {
                printf("OK!\r\n");
            }
            cwb_mdelay(2);
            if(cwb_can_write(&tMsg2)) {
                printf("OK!\r\n");
            }
            cwb_mdelay(10);
            if(cwb_can_write(&tMsg3)) {
                printf("OK!\r\n");
            }
            cwb_mdelay(2);
            if(cwb_can_write(&tMsg4)) {
                printf("OK!\r\n");
            }
            cwb_mdelay(2);
            if(cwb_can_write(&tMsg5)) {
                printf("OK!\r\n");
            }
            cwb_mdelay(10);
            if(cwb_can_write(&tMsg6)) {
                printf("OK!\r\n");
            }
            cwb_mdelay(2);
            if(cwb_can_write(&tMsg7)) {
                printf("OK!\r\n");
            }
            cwb_mdelay(2);
            if(cwb_can_write(&tMsg8)) {
                printf("OK!\r\n");
            }
            cwb_mdelay(10);
            if(cwb_can_write(&tMsg9)) {
                printf("OK!\r\n");
            }
            cwb_mdelay(2);
            if(cwb_can_write(&tMsg10)) {
                printf("OK!\r\n");
            }
            cwb_mdelay(2);
            if(cwb_can_write(&tMsg11)) {
                printf("OK!\r\n");
            }
            cwb_mdelay(10);
            if(cwb_can_write(&tMsg12)) {
                printf("OK!\r\n");
            }
        } else if(!strcmp(argv[1], "recv") && !strcmp(argv[2], "cfg")) {
            //cwb_cfg_recv();
           // printf("%d\r\n", s_chFlag_Can_Interrup);
        } else if(!strcmp(argv[1], "read") && !strcmp(argv[2], "sn")) {
            printf("The flash size is %dKB\r\n", *(uint16_t *)0x1ffff7e0);
            printf("UID0 : is %d\r\n", *(uint32_t *)(0x1ffff7e8));
            printf("UID1 : is %d\r\n", *(uint32_t *)(0x1ffff7ec));
            printf("UID2 : is %d\r\n", *(uint32_t *)(0x1ffff7f0));
        } else if(!strcmp(argv[1], "vol") && !strcmp(argv[2], "out")) {
            printf("%d %f\n", D2MV(adc_pa0_get()), vol_out_get(adc_pa0_get()));
        } else if(!strcmp(argv[1], "vol") && !strcmp(argv[2], "in")) {
            printf("%d %f\n", D2MV(adc_pa1_get()), vol_in_get(adc_pa1_get()));
        } else if(!strcmp(argv[1], "disable") && !strcmp(argv[2], "spv")) {
            cwb_en_spv_set(FALSE);
            printf("disable spv!\n");
        } else if(!strcmp(argv[1], "enable") && !strcmp(argv[2], "spv")) {
            cwb_en_spv_set(TRUE);
            printf("enable spv!\n");
        } else if(!strcmp(argv[1], "disable") && !strcmp(argv[2], "isg")) {
            cwb_isg_set(FALSE);
            printf("disable isg!\n");
        } else if(!strcmp(argv[1], "enable") && !strcmp(argv[2], "isg")) {
            cwb_isg_set(TRUE);
            printf("enable isg!\n");
        } else if(!strcmp(argv[1], "disable") && !strcmp(argv[2], "eload")) {
            cwb_en_eload_set(TRUE);
            printf("disable eload!\n");
        } else if(!strcmp(argv[1], "enable") && !strcmp(argv[2], "eload")) {
            cwb_en_eload_set(FALSE);
            printf("enable eload!\n");
        } else if(!strcmp(argv[1], "test") && !strcmp(argv[2], "fall")) {
            cwb_set_spv_corase(12400);
            cwb_mdelay(5000);
            cwb_isg_set(TRUE);
            cwb_mdelay(100);
            cwb_set_spv_corase(9000);
            cwb_mdelay(250);
            cwb_isg_set(FALSE);
            //detect DUT out  
            if(vol_in_get(adc_pa1_get()) > 12.9) {
                printf("Boost high, vol is %f\n", vol_in_get(adc_pa1_get()));
                //tError = ERROR_BOOST_HIGH;
                //return tError;
            } else if(vol_in_get(adc_pa1_get()) > 12.0) {
                printf("Boost normal, vol is %f\n", vol_in_get(adc_pa1_get()));
            } else if(vol_in_get(adc_pa1_get()) > 9.5) {
                printf("Boost lower, vol is %f\n", vol_in_get(adc_pa1_get()));
                //tError = ERROR_BOOST_LOW;
                //return tError;
            } else {
                printf("Can not boost, vol is %f\n", vol_in_get(adc_pa1_get()));
                //tError = ERROR_CAN_NOT_BOOST;
                //return tError;
            }
            for(uint16_t hwIndex = 10; hwIndex < 5001; hwIndex += 10) {
                cwb_set_spv_corase(9000 + 4500 * hwIndex / 5000);
                cwb_mdelay(10);
            }
        } else if(!strcmp(argv[1], "cur") && !strcmp(argv[2], "in")) {
            uint32_t hwSum = 0;
            for(uint8_t chIndex = 0; chIndex < 128; chIndex++) {
                printf("cur in is %d\r\n", D2MV(adc_pa2_get()));
                hwSum += D2MV(adc_pa2_get());
            }
            printf("cur in average is %d\n", hwSum >> 7);
            printf("cur in average is %f\n", cur_in_get(hwSum >> 7));
            printf("cur in in buf is %f\n", tCwbViMsg.fCinGet);
        } else if(!strcmp(argv[1], "cur") && !strcmp(argv[2], "out")) {
            uint32_t hwSum = 0;
            for(uint8_t chIndex = 0; chIndex < 128; chIndex++) {
                printf("cur out is %d\n", D2MV(adc_pa3_get()));
                hwSum += D2MV(adc_pa3_get());
            }
            printf("cur out average is %d\n", hwSum >> 7);
            printf("cur out average is %f\n", cur_out_get(hwSum >> 7));
        } else if(!strcmp(argv[1], "set") && !strcmp(argv[2], "three")) {
            bflag_three = TRUE;
        } else if(!strcmp(argv[1], "reset") && !strcmp(argv[2], "three")) {
            bflag_three = FALSE;
        } else if(!strcmp(argv[1], "set") && !strcmp(argv[2], "debug")) {
            printf("set ok!\n");
            bDebug = TRUE;
        } else if(!strcmp(argv[1], " reset") && !strcmp(argv[2], "debug")) {
            bDebug = FALSE;
        }
    }
    if(4 == argc) {
        if(!strcmp(argv[1], "set") && !strcmp(argv[2], "spv")) {
            uint16_t hwVol = 0;
            hwVol = atoi(argv[3]);
            cwb_set_spv_corase(hwVol);
            //dac_ch2.write(MV2D(hwVol));
            printf("OK!\r\n");
        }
        if(!strcmp(argv[1], "set") && !strcmp(argv[2], "current")) {
            uint16_t hwCur = 0;
            hwCur = atoi(argv[3]);
            if(cwb_set_eload(hwCur)) {
                printf("%dma set ok!\r\n", hwCur);
            } else {
                printf("Data is oversize!\r\n");
            }
        }
        if(!strcmp(argv[1], "set") && !strcmp(argv[2], "led")) {
            if(!strcmp(argv[3], "red")) {
                product_set_led(LED_RED, LED_ON);
                product_set_led(LED_GREEN, LED_OFF);
                printf("OK!\n");
            } else if(!strcmp(argv[3], "green")) {
                product_set_led(LED_GREEN, LED_ON);
                product_set_led(LED_RED, LED_OFF);
                printf("OK!\n");
            } else if(!strcmp(argv[3], "yellow")) {
                product_set_led(LED_YELLOW, LED_ON);
                printf("OK!\n");
            }
        }
        if(!strcmp(argv[1], "reset") && !strcmp(argv[2], "led")) {
            if(!strcmp(argv[3], "red")) {
                product_set_led(LED_RED, LED_OFF);
                printf("OK!\n");
            } else if(!strcmp(argv[3], "green")) {
                product_set_led(LED_GREEN, LED_OFF);
                printf("OK!\n");
            }
        } else if(!strcmp(argv[1], "test") && !strcmp(argv[2], "rise")) {
            uint16_t hwTime = 0;
            hwTime = atoi(argv[3]);
            printf("low vol is 9v, high vol is 13.4v, time is %ds\n", hwTime);
            for(uint16_t hwIndex = 0; hwIndex < (hwTime * 1000 + 1); hwIndex += 10) {
                cwb_set_spv_corase(9000 + hwIndex * 4500 / (hwTime * 1000));
                cwb_mdelay(10);
            }
            /*cwb_set_spv_corase(9000);
            cwb_mdelay(hwTime * 1000);
            cwb_set_spv_corase(13400);*/
        } else if(!strcmp(argv[1], "test") && !strcmp(argv[2], "fall")) {
            for(int i = 0; i < 10; i++) {
                cwb_set_spv_corase(12500);
                cwb_mdelay(500);
                cwb_set_spv_corase(9000);
                cwb_mdelay(500);
            }
        }
    }
    return 0;
}
cmd_t cmd_cwb = {"cwb", cmd_cwb_func, "commands for cwb"};
DECLARE_SHELL_CMD(cmd_cwb)

static int cmd_cal_func(int argc, char *argv[])
{
    const char *usage = {
        "cal usage : \r\n"
        "cal vol in v0 v1  v9\n"
        "cal vol out v0 v1  v9\n"
        "cal cur in i0 i1  i9\n"
        "cal cur out i0 i1  i9\n"
        "cal dac1 v0\n"
        "cal dac2 v0\n"
        "cal print\n"
    };
    uint8_t chEcode = -1;
    if(2 == argc) {
        if(!strcmp(argv[1], "print")) {
            for(uint8_t chIndex = 0; chIndex < 10; chIndex++) {
                printf("%f  ", s_fVolInCal[chIndex]);
            }
            printf("\n");
        }
        chEcode = 0;
    }
    if((argc > 2) && !strcmp(argv[1], "vol")) {
        if(!strcmp(argv[2], "in")) {
            uint8_t chNum = argc - 3;
            for(uint8_t chIndex = 0; chIndex < chNum; chIndex++) {
                s_fVolInCal[chIndex] = atof(argv[chIndex + 3]);
            }
            nvm_save();
            for(uint8_t chIndex = 0; chIndex < 10; chIndex++) {
                printf("%f ", s_fVolInCal[chIndex]);
            }
            printf("\n");
        } else if(!strcmp(argv[2], "out")) {
            uint8_t chNum = argc - 3;
            for(uint8_t chIndex = 0; chIndex < chNum; chIndex++) {
                s_fVolOutCal[chIndex] = atof(argv[chIndex + 3]);
            }
            nvm_save();
            for(uint8_t chIndex = 0; chIndex < 10; chIndex++) {
                printf("%f ", s_fVolOutCal[chIndex]);
            }
            printf("\n");
        }
        chEcode = 0;
    }
    
    if((argc > 2) && !strcmp(argv[1], "cur")) {
        if(!strcmp(argv[2], "in")) {
            uint8_t chNum = argc - 3;
            float fCurIn[6]; //Current Value
            uint16_t hwCurIn[6]; //Voltage Value
            for(uint8_t chIndex = 0; chIndex < chNum; chIndex += 2) {
                fCurIn[chIndex / 2] = atof(argv[chIndex + 3]);
                hwCurIn[(chIndex + 1) / 2] = atoi(argv[chIndex + 4]);
            }
            for(uint8_t chIndex = 0; chIndex < 6; chIndex++) {
                s_fCurInCal[chIndex] = hwCurIn[chIndex] - fCurIn[chIndex] * STEP_MV;
            }
            nvm_save();
            for(uint8_t chIndex = 0; chIndex < 10; chIndex++) {
                printf("%f ", s_fCurInCal[chIndex]);
            }
            printf("\n");
        } else if(!strcmp(argv[2], "out")) {
            uint8_t chNum = argc - 3;
            float fCurOut[6];
            uint16_t hwCurOut[6];
            for(uint8_t chIndex = 0; chIndex < chNum; chIndex += 2) {
                fCurOut[chIndex / 2] = atof(argv[chIndex + 3]);
                hwCurOut[(chIndex + 1) / 2] = atoi(argv[chIndex + 4]);
            }
            for(uint8_t chIndex = 0; chIndex < 6; chIndex++) {
                s_fCurOutCal[chIndex] = hwCurOut[chIndex] - fCurOut[chIndex] * 33;
            }
            nvm_save();
            for(uint8_t chIndex = 0; chIndex < 10; chIndex++) {
                printf("%f ", s_fCurOutCal[chIndex]);
            }
            printf("\n");
        }
        chEcode = 0;
    }
    if(argc == 3) {
        if(!strcmp(argv[1], "dac1")) {
            uint16_t hwMv = 0;
            hwMv = atoi(argv[2]);
            dac_ch1.write(MV2D(hwMv));
            printf("dac1 %dmv set Ok!\n", hwMv);
        }
        if(!strcmp(argv[1], "dac2")) {
            uint16_t hwMv = 0;
            hwMv = atoi(argv[2]);
            dac_ch2.write(MV2D(hwMv));
            printf("dac2 %dmv set ok!\n", hwMv);
        }
        chEcode = 0;
    }
    if((argc < 2) || (chEcode)){
        printf("%s", usage);
        return 0;
    }
    return 0;
}

cmd_t cmd_cal = {"cal", cmd_cal_func, "cwb calibration"};
DECLARE_SHELL_CMD(cmd_cal)
