/*
	jiamao.gu 2014 first version in Shanghai
 */

#include <string.h>
#include "config.h"
#include "sys/task.h"
#include "ulp/sys.h"
#include "ulp_time.h"
#include "can.h"
#include "priv/usdt.h"
#include "ulp/debug.h"
#include "shell/cmd.h"
#include "uart.h"
#include "stdio.h"
#include "rsu.h"
#include "led.h"


#define PDI_DEBUG		0        // Test dubug by putty

FUNCTION_T Function[10];  				// max 10 functions address
POSITION_E position = LEFT; 			// left position
FIXTURE_E SlotA = IDLE, SlotB = COMPLETE;   // for Ping Pang Operation
static char counterA = 0;		//counter the num of barcodeA from Up Computer, in order to compare with the num of RSU placed
static char counterB = 0;		//counter the num of barcodeB from Up Compuetr, aimed to compare with the num of RSU placed
static ERROR_E error_result = ERROR_NO;
static char flag_start = 0;
static char flag_position = 0;
static char flag_position_a = 0;
static char flag_position_b = 0;
can_msg_t msg_can = {0,8,{0,0,0,0,0,0,0,0},0};
int pdi_mdelay(int ms)
{
	int left;
	time_t deadline = time_get(ms);
	do {
		left = time_left(deadline);
		if(left >= 10) { //system update period is expected less than 10ms
			sys_update();
		}
	} while(left > 0);
	return 0;
}
//delay ms
int RSU_mdelay(int ms)
{
	int left;
	time_t deadline = time_get(ms);
	do {
		left = time_left(deadline);
		if(left >= 10) { //system update period is expected less than 10ms
			sys_update();
		}
		if((position == LEFT) && (SlotA == IDLE) && (flag_position_a == 1)) {
			left = 0;
			error_result = ERROR_BACHU;
		}
		if((position == RIGHT) && (SlotB == IDLE) && (flag_position_b == 1)) {
			left = 0;
			error_result = ERROR_BACHU;
		}
	} while(left > 0);
	return 0;
}
// get left or right position for other c file like pdi.c and drive.c
POSITION_E *Position_Get(void)
{
	return &position;
}
ERROR_E *Error_Get(void)
{
	return &error_result;
}
void DealAfterStop(PRODUCT_T product)
{
	if(error_result != ERROR_NUM) {
		char buffer_temp[256];
		memset(buffer_temp,0,sizeof(buffer_temp));
		if(!RSU_Uart_Get(buffer_temp)) { // All the receive data put in autoliv_buf
			if (buffer_temp[0] == 'O') {	
				printf("ITAC OK \r\n");
			} else if(buffer_temp[0] == 'N') {
				printf("ITAC may not OK \r\n");
				error_result = ERROR_ITAC;
			} else if((buffer_temp[0] == 0x0C) && (buffer_temp[1] == 0x0A) || (buffer_temp[3] == 0x0C) && (buffer_temp[4] == 0x0A) ) {
				printf("2%x %x\r\n",buffer_temp[0],buffer_temp[1]);
				flag_start = 1;
				if(flag_position == 0) { //B is being tested or the first time
					counterA++;			//if equal, counterA is clear
					//accordint to different counter flash different leds
					if(counterA == 1) {
						for(int i = 0; i < 4; i++) {
							led_off((led_t)(LED_RED1 + 3 * i));
							led_off((led_t)(LED_GREEN1 + 3 * i));
						}
						led_flash(LED_RED1);
					} else if(counterA == 2) {
						led_off(LED_RED1);
						led_Update_Immediate();
						led_flash(LED_RED1);
						led_flash(LED_RED2);
					} else if(counterA == 3) {
						led_off(LED_RED1);
						led_off(LED_RED2);
						led_Update_Immediate();
						led_flash(LED_RED1);
						led_flash(LED_RED2);
						led_flash(LED_RED3);
					} else if(counterA == 4) {
						led_off(LED_RED1);
						led_off(LED_RED2);
						led_off(LED_RED3);
						led_Update_Immediate();
						led_flash(LED_RED1);
						led_flash(LED_RED2);
						led_flash(LED_RED3);
						led_flash(LED_RED4);
					}
				} else if(flag_position == 1) {// A is being tested
					counterB++;				//if equal, counterB is clear
					if(counterB == 1) {
						for(int i = 0; i < 4; i++) {
							led_off((led_t)(LED_RED5 + 3 * i));
							led_off((led_t)(LED_GREEN5 + 3 * i));
						}
						led_flash(LED_RED5);
					} else if(counterB == 2) {
						led_off(LED_RED5);
						led_Update_Immediate();
						led_flash(LED_RED5);
						led_flash(LED_RED6);
					} else if(counterB == 3) {
						led_off(LED_RED5);
						led_off(LED_RED6);
						led_Update_Immediate();
						led_flash(LED_RED5);
						led_flash(LED_RED6);
						led_flash(LED_RED7);
					} else if(counterB == 4) {
						led_off(LED_RED5);
						led_off(LED_RED6);
						led_off(LED_RED7);
						led_off(LED_RED8);
						led_Update_Immediate();
						led_flash(LED_RED5);
						led_flash(LED_RED6);
						led_flash(LED_RED7);
						led_flash(LED_RED8);
					}
				}
			}
		}
	}
	RSU_Result(product);
}
void DealAfterReceive(void)
{
	char buffer[256];
	memset(buffer,0,sizeof(buffer));	// Global variable attention!!!
	if(!RSU_Uart_Get(buffer)) {	// All the receive data put in autoliv_buf
		if((buffer[0] == 0x33)&&(buffer[1] == 0x55)) {
			//B562 Side R018
			//D2XX Front R023
                        //D2xx Z R024
			RSU_Uart_Send("R025");	//TBD
			printf("ID:R025\r\n");
		} else if((buffer[0] == 0x41)&&(buffer[1] == 0x42)) {
			ADC_DATE_T adc_temp;
			RSU_Read_Power(&adc_temp);
			printf("%f\r\n",adc_temp.average);
			char adc_t[5];
			sprintf(adc_t,"%3.1f",adc_temp.average);
			RSU_Uart_Send(adc_t);
			printf("%s",adc_t);
		} else if((buffer[0] == 0x01)&&(buffer[1] == 0x01)) {
			RSU_Uart_Send("DL-OK");
			printf("DL-OK\r\n");
		} else if(((buffer[0] == 0x0C)&&(buffer[1] == 0x0A)) || ((buffer[3] == 0x0C)&&(buffer[4] == 0x0A))) {
			printf("1%x %x\r\n",buffer[0],buffer[1]);
			flag_start = 1;
			if(flag_position == 0) { //B is being tested or the first time
				counterA++;			//if equal, counterA is clear
				//accordint to different counter flash different leds
				if(counterA == 1) {
					for(int i = 0; i < 4; i++) {
						led_off((led_t)(LED_RED1 + 3 * i));
						led_off((led_t)(LED_GREEN1 + 3 * i));
					}
					led_flash(LED_RED1);
				} else if(counterA == 2) {
					led_off(LED_RED1);
					led_Update_Immediate();
					led_flash(LED_RED1);
					led_flash(LED_RED2);
				} else if(counterA == 3) {
					led_off(LED_RED1);
					led_off(LED_RED2);
					led_Update_Immediate();
					led_flash(LED_RED1);
					led_flash(LED_RED2);
					led_flash(LED_RED3);
				} else if(counterA == 4) {
					led_off(LED_RED1);
					led_off(LED_RED2);
					led_off(LED_RED3);
					led_Update_Immediate();
					led_flash(LED_RED1);
					led_flash(LED_RED2);
					led_flash(LED_RED3);
					led_flash(LED_RED4);
				}
			} else if(flag_position == 1) {// A is being tested
				counterB++;				//if equal, counterB is clear
				if(counterB == 1) {
					for(int i = 0; i < 4; i++) {
						led_off((led_t)(LED_RED5 + 3 * i));
						led_off((led_t)(LED_GREEN5 + 3 * i));
					}
					led_flash(LED_RED5);
				} else if(counterB == 2) {
					led_off(LED_RED5);
					led_Update_Immediate();
					led_flash(LED_RED5);
					led_flash(LED_RED6);
				} else if(counterB == 3) {
					led_off(LED_RED5);
					led_off(LED_RED6);
					led_Update_Immediate();
					led_flash(LED_RED5);
					led_flash(LED_RED6);
					led_flash(LED_RED7);
				} else if(counterB == 4) {
					led_off(LED_RED5);
					led_off(LED_RED6);
					led_off(LED_RED7);
					led_off(LED_RED8);
					led_Update_Immediate();
					led_flash(LED_RED5);
					led_flash(LED_RED6);
					led_flash(LED_RED7);
					led_flash(LED_RED8);
				}
			}
		}  else if(buffer[0] == 'S' && buffer[1] == 'A') {
				flag_position = 0;
				position = LEFT;
		}  else if(buffer[0] == 'S' && buffer[1] == 'B') {
				flag_position = 1;
				position = RIGHT;
		}
	}
}
void Status_transform(void)
{
	//for counterA & B
	//static char his_a = 0;
	//static char his_b = 0;
	/*if(!RSU_TargetA_On() && (his_a == 0)) {
		for(int i = 100000; i > 0; i--);
		if(!RSU_TargetA_On()) {
			his_a = 1;
			RSU_CounterA_Add();
		}
	}*/
	if(RSU_TargetA_On()) {
		for(int i = 100000; i > 0; i--);
		if(RSU_TargetA_On()) {
			//his_a = 0;
			SlotA = IDLE;
			if(flag_position_a == 1) {
				error_result = ERROR_BACHU;
			}
		}
	}
	/*if(!RSU_TargetB_On() && (his_b == 0)) {
		for(int i = 100000; i > 0; i--);
		if(!RSU_TargetB_On()) {
			his_b = 1;
			RSU_CounterB_Add();
		}
	}*/
	if(RSU_TargetB_On()) {
		for(int i = 100000; i > 0; i--);
		if(RSU_TargetB_On()) {
			//his_b = 0;
			SlotB = IDLE;
			if(flag_position_b == 1) {
				error_result = ERROR_BACHU;
			}
		}
	}
	//status transform
	if((!RSU_TargetA_On()) && (SlotA == IDLE)) {  // on + IDLE --> READY
		SlotA = READY;
	}
	if((!RSU_TargetB_On()) && (SlotB == IDLE)) {
		SlotB = READY;
	}
	if(RSU_TargetA_On()) {
		SlotA = IDLE;
	}
	if(RSU_TargetB_On()) {
		SlotB = IDLE;
	}
}
/*all the init include drive and so on init*/
void pdi_init()
{
	RSU_Init();
	RSU_check_action();
}
void pdi_update(void)
{
	DealAfterReceive();
	Status_transform();
}
/************************************
** some behavior can not be predicted. 
** so need this update **************
** it will be called as the cpu idle* 
*************************************/
void __sys_update(void)
{
	pdi_update();                      // function here can be often called 
}
//counter the position of RSU and flash the related reds
//send UUTX to Up Computer
//flash the leds
ERROR_E Test_Start(FIXTURE_STATUS_T *fixture)
{
	if(error_result != ERROR_NO) {
		return error_result;
	}
	RSU_mdelay(500);
	if(position == LEFT) {
		RSU_RelayA_Set();
		RSU_RelayB_Reset();
	} else if(position == RIGHT) {
		RSU_RelayB_Set();
		RSU_RelayA_Reset();
	}
	RSU_mdelay(10); //!!!!
	EXIST_T exist_temp = {{EXIST_NO,EXIST_NO,EXIST_NO,EXIST_NO},0};
	exist_temp = RSU_Read_Status(fixture);
	if(position == LEFT) {
		if(counterA != exist_temp.counter) {		
			RSU_Uart_Send("N-UUTA");
			printf("N-UUTA\r\n");
			error_result = ERROR_NUM;
			flag_position = 0;	//next product still in left
			printf("!!! left\r\n");
                        counterA = 0;
                        return error_result;
		} else {
			RSU_Uart_Send("UUTA");
			printf("UUTA\r\n");
			flag_position = 1;	//next product is right
			printf("CounterA will be clear!\r\n");
			printf("flag_position set 1\r\n");
		}
		if(counterA == 1) {
			if((exist_temp.p[0] == EXIST_YES) && (exist_temp.p[1] == EXIST_NO) && (exist_temp.p[2] == EXIST_NO) && (exist_temp.p[3] == EXIST_NO))
				error_result = ERROR_NO;
			else {
				error_result = ERROR_POSITION;
				flag_position = 0;	//next product still in left
			}
		}
		if(counterA == 2) {
			if((exist_temp.p[0] == EXIST_YES) && (exist_temp.p[1] == EXIST_YES) && (exist_temp.p[2] == EXIST_NO) && (exist_temp.p[3] == EXIST_NO))
				error_result = ERROR_NO;
			else {
				error_result = ERROR_POSITION;
				flag_position = 0;	//next product still in left
			}
		}
		if(counterA == 3) {
			if((exist_temp.p[0] == EXIST_YES) && (exist_temp.p[1] == EXIST_YES) && (exist_temp.p[2] == EXIST_YES) && (exist_temp.p[3] == EXIST_NO))
				error_result = ERROR_NO;
			else {
				error_result = ERROR_POSITION;
				flag_position = 0;	//next product still in left
			}
		}
		if(counterA == 4) {
			if((exist_temp.p[0] == EXIST_YES) && (exist_temp.p[1] == EXIST_YES) && (exist_temp.p[2] == EXIST_YES) && (exist_temp.p[3] == EXIST_YES))
				error_result = ERROR_NO;
			else {
				error_result = ERROR_POSITION;
				flag_position = 0;	//next product still in left
			}
		}
		counterA = 0;
	} else if(position == RIGHT) {
		if(counterB != exist_temp.counter) {
			RSU_Uart_Send("N-UUTB");
			printf("N-UUTB\r\n");
			error_result = ERROR_NUM;
			flag_position = 1;	//next product still in right
			printf("!!! right\r\n");
                        counterB = 0;
                        return error_result;
		} else {
			RSU_Uart_Send("UUTB");
			printf("UUTB\r\n");
			flag_position = 0;	//next product is left
			printf("CounterB will be clear!\r\n");
			printf("flag_position set 0\r\n");
		}
		if(counterB == 1) {
			if((exist_temp.p[0] == EXIST_YES) && (exist_temp.p[1] == EXIST_NO) && (exist_temp.p[2] == EXIST_NO) && (exist_temp.p[3] == EXIST_NO))
				error_result = ERROR_NO;
			else {
				error_result = ERROR_POSITION;
				flag_position = 1;	//next product still in right
			}	
		}
		if(counterB == 2) {
			if((exist_temp.p[0] == EXIST_YES) && (exist_temp.p[1] == EXIST_YES) && (exist_temp.p[2] == EXIST_NO) && (exist_temp.p[3] == EXIST_NO))
				error_result = ERROR_NO;
			else {
				error_result = ERROR_POSITION;
				flag_position = 1;	//next product still in right
			}
		}
		if(counterB == 3) {
			if((exist_temp.p[0] == EXIST_YES) && (exist_temp.p[1] == EXIST_YES) && (exist_temp.p[2] == EXIST_YES) && (exist_temp.p[3] == EXIST_NO))
				error_result = ERROR_NO;
			else {
				error_result = ERROR_POSITION;
				flag_position = 1;	//next product still in right
			}
		}
		if(counterB == 4) {
			if((exist_temp.p[0] == EXIST_YES) && (exist_temp.p[1] == EXIST_YES) && (exist_temp.p[2] == EXIST_YES) && (exist_temp.p[3] == EXIST_YES))
				error_result = ERROR_NO;
			else {
				error_result = ERROR_POSITION;
				flag_position = 1;	//next product still in right
			}
		}
		counterB = 0;
	}
	flag_start = 0;
	RSU_Start_Action();
	return error_result;
}
//read over three times can flame
//1 2 2 2 2 2 2 2 2 
ERROR_E Test_Message(MSG_T *msg)
{
	if(error_result != ERROR_NO) {
		return error_result;
	}
	RSU_Power_On();
	if(position == LEFT) {
		RSU_RelayA_Reset();	//connect the RSU with ECU
	} else if(position == RIGHT) {
		RSU_RelayB_Reset();
	}
	RSU_mdelay(7000);	
	if(error_result == ERROR_BACHU) {
		return error_result;
	}
	RSU_Can_Read(&msg_can);
	RSU_Can_Read(&msg_can);
	RSU_Can_Read(&msg_can);
	RSU_Can_Read(&msg_can);
	can_msg_print(&msg_can,"\n");
	*msg = RSU_Can_Explain(&msg_can);
	RSU_Power_Off();
	return error_result;
}
//judge the result according to position and can flame
//upload to itac
//set the related leds
void Test_Stop(FIXTURE_STATUS_T fixture, MSG_T msg, PRODUCT_T *product, can_msg_t *msg_can)
{
	if(error_result == ERROR_NO) {
		if(msg.BYTE_FLAG.FLAG0 != 0) {
			error_result = ERROR_ECUNG;
		}
		if(position == LEFT) {
			if((fixture.BYTEBIT.BIT1 == 0) && (msg.BYTE_FLAG.FLAG1 == 0)) {
				product -> product[0] = OK;
				printf("RSU1 is %d, PASS!\r\n",msg.BYTE_FLAG.FLAG1);
			} else if((fixture.BYTEBIT.BIT1 == 0) && (msg.BYTE_FLAG.FLAG1 == 1)) {
				product -> product[0] = NOK;
				error_result = ERROR_FUNC;
				printf("RSU1 is %d, FAIL!\r\n",msg.BYTE_FLAG.FLAG1);
			} else if((fixture.BYTEBIT.BIT1 == 1) && (msg.BYTE_FLAG.FLAG1 == 1)) {
				//TBD
				product -> product[0] = NC;
				printf("RSU1 is NOT on!\r\n");
			} else if((fixture.BYTEBIT.BIT1 == 1) && (msg.BYTE_FLAG.FLAG1 == 0)) {
				printf("IMPOSSBLE!\r\n");
			}			
			if((fixture.BYTEBIT.BIT2 == 0) && (msg.BYTE_FLAG.FLAG2 == 0)) {
				product -> product[1] = OK;
				printf("RSU2 is %d, PASS!\r\n",msg.BYTE_FLAG.FLAG2);
			} else if((fixture.BYTEBIT.BIT2 == 0) && (msg.BYTE_FLAG.FLAG2 == 1)) {
				product -> product[1] = NOK;
				error_result = ERROR_FUNC;
				printf("RSU2 is %d, FAIL!\r\n",msg.BYTE_FLAG.FLAG2);
			} else if((fixture.BYTEBIT.BIT2 == 1) && (msg.BYTE_FLAG.FLAG2 == 1)) {
				//TBD
				product -> product[1] = NC;
				printf("RSU2 is NOT on!\r\n");
			} else if((fixture.BYTEBIT.BIT2 == 1) && (msg.BYTE_FLAG.FLAG2 == 0)) {
				printf("IMPOSSBLE!\r\n");
			}
			if((fixture.BYTEBIT.BIT3 == 0) && (msg.BYTE_FLAG.FLAG3 == 0)) {
				product -> product[2] = OK;
				printf("RSU3 is %d, PASS!\r\n",msg.BYTE_FLAG.FLAG3);
			} else if((fixture.BYTEBIT.BIT3 == 0) && (msg.BYTE_FLAG.FLAG3 == 1)) {
				product -> product[2] = NOK;
				error_result = ERROR_FUNC;
				printf("RSU3 is %d, FAIL!\r\n",msg.BYTE_FLAG.FLAG3);
			} else if((fixture.BYTEBIT.BIT3 == 1) && (msg.BYTE_FLAG.FLAG3 == 1)) {
				//TBD
				product -> product[2] = NC;
				printf("RSU3 is NOT on!\r\n");
			} else if((fixture.BYTEBIT.BIT3 == 1) && (msg.BYTE_FLAG.FLAG3 == 0)) {
				printf("IMPOSSBLE!\r\n");
			}
			if((fixture.BYTEBIT.BIT4 == 0) && (msg.BYTE_FLAG.FLAG4 == 0)) {
				product -> product[3] = OK;
				printf("RSU4 is %d, PASS!\r\n",msg.BYTE_FLAG.FLAG4);
			} else if((fixture.BYTEBIT.BIT4 == 0) && (msg.BYTE_FLAG.FLAG4 == 1)) {
				product -> product[3] = NOK;
				error_result = ERROR_FUNC;
				printf("RSU4 is %d, FAIL!\r\n",msg.BYTE_FLAG.FLAG4);
			} else if((fixture.BYTEBIT.BIT4 == 1) && (msg.BYTE_FLAG.FLAG4 == 1)) {
				//TBD
				product -> product[3] = NC;
				printf("RSU4 is NOT on!\r\n");
			} else if((fixture.BYTEBIT.BIT4 == 1) && (msg.BYTE_FLAG.FLAG4 == 0)) {
				printf("IMPOSSBLE!\r\n");
			}
		} else if(position == RIGHT) {
			if((fixture.BYTEBIT.BIT5 == 0) && (msg.BYTE_FLAG.FLAG5 == 0)) {
				product -> product[4] = OK;
				printf("RSU5 is %d, PASS!\r\n",msg.BYTE_FLAG.FLAG5);
			} else if((fixture.BYTEBIT.BIT5 == 0) && (msg.BYTE_FLAG.FLAG5 == 1)) {
				product -> product[4] = NOK;
				error_result = ERROR_FUNC;
				printf("RSU5 is %d, FAIL!\r\n",msg.BYTE_FLAG.FLAG5);
			} else if((fixture.BYTEBIT.BIT5 == 1) && (msg.BYTE_FLAG.FLAG5 == 1)) {
				//TBD
				product -> product[4] = NC;
				printf("RSU6 is NOT on!\r\n");
			} else if((fixture.BYTEBIT.BIT5 == 1) && (msg.BYTE_FLAG.FLAG5 == 0)) {
				printf("IMPOSSBLE!\r\n");
			}			
			if((fixture.BYTEBIT.BIT6 == 0) && (msg.BYTE_FLAG.FLAG6 == 0)) {
				product -> product[5] = OK;
				printf("RSU6 is %d, PASS!\r\n",msg.BYTE_FLAG.FLAG6);
			} else if((fixture.BYTEBIT.BIT6 == 0) && (msg.BYTE_FLAG.FLAG6 == 1)) {
				product -> product[5] = NOK;
				error_result = ERROR_FUNC;
				printf("RSU6 is %d, FAIL!\r\n",msg.BYTE_FLAG.FLAG6);
			} else if((fixture.BYTEBIT.BIT6 == 1) && (msg.BYTE_FLAG.FLAG6 == 1)) {
				//TBD
				product -> product[5] = NC;
				printf("RSU6 is NOT on!\r\n");
			} else if((fixture.BYTEBIT.BIT6 == 1) && (msg.BYTE_FLAG.FLAG6 == 0)) {
				printf("IMPOSSBLE!\r\n");
			}
			if((fixture.BYTEBIT.BIT7 == 0) && (msg.BYTE_FLAG.FLAG7 == 0)) {
				product -> product[6] = OK;
				printf("RSU7 is %d, PASS!\r\n",msg.BYTE_FLAG.FLAG7);
			} else if((fixture.BYTEBIT.BIT7 == 0) && (msg.BYTE_FLAG.FLAG7 == 1)) {
				product -> product[6] = NOK;
				error_result = ERROR_FUNC;
				printf("RSU7 is %d, FAIL!\r\n",msg.BYTE_FLAG.FLAG7);
			} else if((fixture.BYTEBIT.BIT7 == 1) && (msg.BYTE_FLAG.FLAG7 == 1)) {
				//TBD
				product -> product[6] = NC;
				printf("RSU7 is NOT on!\r\n");
			} else if((fixture.BYTEBIT.BIT7 == 1) && (msg.BYTE_FLAG.FLAG7 == 0)) {
				printf("IMPOSSBLE!\r\n");
			}
			if((fixture.BYTEBIT.BIT8 == 0) && (msg.BYTE_FLAG.FLAG8 == 0)) {
				product -> product[7] = OK;
				printf("RSU8 is %d, PASS!\r\n",msg.BYTE_FLAG.FLAG8);
			} else if((fixture.BYTEBIT.BIT8 == 0) && (msg.BYTE_FLAG.FLAG8 == 1)) {
				product -> product[7] = NOK;
				error_result = ERROR_FUNC;
				printf("RSU8 is %d, FAIL!\r\n",msg.BYTE_FLAG.FLAG8);
			} else if((fixture.BYTEBIT.BIT8 == 1) && (msg.BYTE_FLAG.FLAG8 == 1)) {
				//TBD
				product -> product[7] = NC;
				printf("RSU8 is NOT on!\r\n");
			} else if((fixture.BYTEBIT.BIT8 == 1) && (msg.BYTE_FLAG.FLAG8 == 0)) {
				printf("IMPOSSBLE!\r\n");
			}
		}
	}
	if(position == LEFT) {
		switch(error_result) {
			case ERROR_NO:
				RSU_Uart_Send("UUTAOK");
				RSU_mdelay(500);
				RSU_Uart_SendChar(msg_can -> data[0]);
				RSU_Uart_SendChar(msg_can -> data[1]);
				RSU_Uart_SendChar(msg_can -> data[2]);
				RSU_Uart_SendChar(msg_can -> data[3] + 1);
				RSU_Uart_SendChar(msg_can -> data[4] + 1);
				RSU_Uart_SendChar(msg_can -> data[5] + 1);
				RSU_Uart_SendChar(msg_can -> data[6] + 1);
				RSU_Uart_SendChar(msg_can -> data[7] + 1);
				printf("A PASS \r\n");
				break;
			case ERROR_BACHU:
				RSU_Uart_Send("UUTANG-BRK");
				printf("UUTANG-BRK\r\n");
				printf("A is being ba chu!\r\n");
				break;
			case ERROR_POSITION:
				RSU_Uart_Send("P-UUTA");
				printf("P-UUTA\r\n");
				break;
			case ERROR_NUM:
				printf("A num of RSU not correct \r\n");
				printf("A will be tested again!\r\n");
				break;
			case ERROR_ECUNG:
				RSU_Uart_Send("ECU-NG");
				printf("ECU-NG\t\n");
				break;
			case ERROR_FUNC:
				RSU_Uart_Send("UUTANG");
				RSU_mdelay(500);
				RSU_Uart_SendChar(msg_can -> data[0]);
				RSU_Uart_SendChar(msg_can -> data[1]);
				RSU_Uart_SendChar(msg_can -> data[2]);
				RSU_Uart_SendChar(msg_can -> data[3] + 1);
				RSU_Uart_SendChar(msg_can -> data[4] + 1);
				RSU_Uart_SendChar(msg_can -> data[5] + 1);
				RSU_Uart_SendChar(msg_can -> data[6] + 1);
				RSU_Uart_SendChar(msg_can -> data[7] + 1);
				printf("A function not OK \r\n");
				break;
			default:
				break;
		}
	} else if(position == RIGHT) {
		switch(error_result) {
			case ERROR_NO:
				RSU_Uart_Send("UUTBOK");
				RSU_mdelay(500);
				RSU_Uart_SendChar(msg_can -> data[0]);
				RSU_Uart_SendChar(msg_can -> data[1]);
				RSU_Uart_SendChar(msg_can -> data[2]);
				RSU_Uart_SendChar(msg_can -> data[3] + 1);
				RSU_Uart_SendChar(msg_can -> data[4] + 1);
				RSU_Uart_SendChar(msg_can -> data[5] + 1);
				RSU_Uart_SendChar(msg_can -> data[6] + 1);
				RSU_Uart_SendChar(msg_can -> data[7] + 1);
				printf("B PASS \r\n");
				break;
			case ERROR_BACHU:
				RSU_Uart_Send("UUTBNG-BRK");
				printf("UUTBNG-BRK\r\n");
				printf("B is being ba chu!\r\n");
				break;
			case ERROR_POSITION:
				RSU_Uart_Send("P-UUTB");
				printf("P-UUTB\r\n");
				break;
			case ERROR_NUM:
				printf("B num of RSU not correct \r\n");
				printf("B wil be tested again!\r\n");
				break;
			case ERROR_ECUNG:
				RSU_Uart_Send("ECU-NG");
				printf("ECU-NG\t\n");
				break;
			case ERROR_FUNC:
				RSU_Uart_Send("UUTBNG");
				RSU_mdelay(500);
				RSU_Uart_SendChar(msg_can -> data[0]);
				RSU_Uart_SendChar(msg_can -> data[1]);
				RSU_Uart_SendChar(msg_can -> data[2]);
				RSU_Uart_SendChar(msg_can -> data[3] + 1);
				RSU_Uart_SendChar(msg_can -> data[4] + 1);
				RSU_Uart_SendChar(msg_can -> data[5] + 1);
				RSU_Uart_SendChar(msg_can -> data[6] + 1);
				RSU_Uart_SendChar(msg_can -> data[7] + 1);
				printf("B function not OK \r\n");
				break;
			default:
				break;
		}
	}
}
void RSU_Change_position()
{
	char buffer[256];
	memset(buffer,0,sizeof(buffer));	// Global variable attention!!!
	if(position == LEFT) {
		switch(error_result) {
			case ERROR_NO:
				position = RIGHT;
				break;
			case ERROR_BACHU:
				position = RIGHT;
				break;
			case ERROR_NUM:
				position = LEFT;
				break;
			case ERROR_POSITION:
				break;
			case ERROR_ECUNG:
				position = LEFT;
				break;
			case ERROR_FUNC:
				position = RIGHT;
				break;
			default:
				break;
		}
	} else if(position == RIGHT) {
		switch(error_result) {
			case ERROR_NO:
				position = LEFT;
				break;
			case ERROR_BACHU:
				position = LEFT;
				break;
			case ERROR_NUM:
				position = RIGHT;
				break;
			case ERROR_POSITION:
				position = RIGHT;
				break;
			case ERROR_ECUNG:
				position = RIGHT;
				break;
			case ERROR_FUNC:
				position = LEFT;
				break;
			default:
				break;
		}
	}
}
int main(void)
{
	sys_init();
	pdi_init();
	FIXTURE_STATUS_T fixture_temp;
	memset(&fixture_temp,0,sizeof(fixture_temp));
	MSG_T msg_temp;
	memset(&msg_temp,0,sizeof(msg_temp));
	PRODUCT_T product_temp;
	memset(&product_temp,0,sizeof(product_temp));
	while(1) {		
		sys_update();
		if((SlotA == READY) && (flag_position == 0) && (SlotB == IDLE) && (flag_start == 1)) {
			Test_Start(&fixture_temp);
			flag_position_a = 1;
			Test_Message(&msg_temp);
			Test_Stop(fixture_temp,msg_temp,&product_temp,&msg_can);
			flag_position_a = 0;
			DealAfterStop(product_temp);
			RSU_Change_position();
			SlotA = COMPLETE;
			error_result = ERROR_NO;
		}
		if((SlotB == READY) && (flag_position == 1) && (SlotA == IDLE) && (flag_start == 1)) {
			Test_Start(&fixture_temp);
			flag_position_b = 1;
			Test_Message(&msg_temp);
			Test_Stop(fixture_temp,msg_temp,&product_temp,&msg_can);
			flag_position_b = 0;
			DealAfterStop(product_temp);
			RSU_Change_position();
			SlotB = COMPLETE;
			error_result = ERROR_NO;
		}
	}
}

#if 1
static int cmd_b515rsu_func(int argc, char *argv[])
{
	const char *usage = {
		"b515rsu , usage:\n"
		"b515rsu get position  \r\n"
		"b515rsu set position x  left or right\r\n"
	};

	if (argc < 2) {
		printf("%s", usage);
		return 0;
	}
	if(argc == 3) {
		if(!strcmp(argv[1],"get")&&!strcmp(argv[2],"position")) {
			if(position == LEFT) {
				printf("Current position is left!\r\n");
			} else if(position == RIGHT) {
				printf("Current position is right!\r\n");
			}
		}
	}
	if(argc == 4) {
		if(!strcmp(argv[1],"set")&&!strcmp(argv[2],"position")) {
			if(!strcmp(argv[3],"left")) {
				position = LEFT;
				printf("Set the positon left OK!\r\n");
			} else if(!strcmp(argv[3],"right")) {
				position = RIGHT;
				printf("Set the positionb right OK!\r\n");
			} else {
				printf("Not correct position set!\r\n");
			}
		}
	}
	return 0;
}

const cmd_t cmd_b515rsu = {"b515rsu", cmd_b515rsu_func, "b515rsu cmd i/f"};
DECLARE_SHELL_CMD(cmd_b515rsu)
#endif