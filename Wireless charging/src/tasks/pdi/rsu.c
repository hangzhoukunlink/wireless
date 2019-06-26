#include <string.h>
#include <stdarg.h>
#include "config.h"
#include "sys/task.h"
#include "ulp/sys.h"
#include "ulp_time.h"
#include "priv/usdt.h"
#include "ulp/debug.h"
#include "shell/cmd.h"
#include "stm32f10x.h"
#include "uart.h"
#include "can.h"
#include "stdio.h"
#include "rsu.h"
#include "led.h"

#ifdef CONFIG_PDI_B515RSU	
//old version
#if 0		
//PE
#define TARGETA				(1 << 13)
#define TARGETB				(1 << 12)
#define TESTA				(1 << 15) //relay
#define TESTB				(1 << 14) //relay
#define POWER				(1 << 11) //power on 
//PB
#define BEEP_ON				(1 << 12)
#define COUNTERA			(1 << 14)
#define COUNTERB			(1 << 13)
//PA
#define RSU0				0
#define RSU1				1
#define RSU2				4
#define RSU3				5
#define RSU4				6
#define RSU5				7
//PB
#define RSU6				0
#define RSU7				1
#define ADC 0
#endif
//new version
#if 1		
//PE
#define TARGETA				(1 << 13)
#define TARGETB				(1 << 12)
#define TESTB				(1 << 1) //relay
#define POWER				(1 << 11) //power on
//PC
#define TESTA 				(1 << 9) 
//PB
#define BEEP_ON				(1 << 14)
//#define COUNTERA			(1 << 14)
//#define COUNTERB			(1 << 13)
//PA
#define RSU0				0
#define RSU1				1
#define RSU2				4
#define RSU3				5
#define RSU4				6
#define RSU5				7
//PB
#define RSU6				0
#define RSU7				1
#define ADC 0
#endif
#endif

#define RSU_SHELL uart3
static const can_bus_t* rsu_can_bus = &can1;
static const autoliv_t pdi_autoliv = {
		.bus = &uart3,
		.data_len = 200,              // Xuce Up computer config file max length < 200, I think
		.dead_time = 20
	};
POSITION_E *position_rsu;
ERROR_E *error_result_rsu;
FIXTURE_STATUS_T fixture_status;
FIXTURE_STATUS_T *Fixture_Status_Return(void)
{
	return &fixture_status;
}
//init the rsu_uart
void RSU_Uart_Init(void)
{
	uart_cfg_t cfg = {
		.baud = 9600,
	};
	pdi_autoliv.bus -> init(&cfg);
	pdi_autoliv.bus -> putchar('!');//for test the usart
}
//for the communication with Up_computer
int RSU_Uart_Get(char *rx_data)
{
	time_t overtime;
	if (pdi_autoliv.bus -> poll()) {
		for (int i = 0; i < pdi_autoliv.data_len; i++) {
			//according to overtime
			overtime = time_get(pdi_autoliv.dead_time);
			while (pdi_autoliv.bus -> poll() == 0) {
				if (time_left(overtime) < 0)
					return 0;
			}
			rx_data[i] = pdi_autoliv.bus -> getchar();
		}
		return 0;
	}
	return 1;
}
//
void RSU_Uart_Send(char *tx_data)
{
	for(int i = 0; i < strlen(tx_data); i++) {
		pdi_autoliv.bus -> putchar(tx_data[i]);
	}
}
//
void RSU_Uart_SendChar(char tx_data)
{
	pdi_autoliv.bus -> putchar(tx_data);
}
//for counter&&beeper&&ad&&relay&&target init
void RSU_Interface_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	//BEEPER
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//TESTB PDCT_C
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//TESTA
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//TARGETA TARGETB
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//ADC
	#if ADC
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); /*72Mhz/8 = 9Mhz*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//RSU1_CH0 RSU2_CH1 RSU3_CH4 RSU4_CH5 RSU5_CH6 RSU6_CH7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//RSU7_CH8 RSU8_CH9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	#else
	//RSU1_CH0 RSU2_CH1 RSU3_CH4 RSU4_CH5 RSU5_CH6 RSU6_CH7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//RSU7_CH8 RSU8_CH9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	#endif
	//power adc ch10
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); /*72Mhz/8 = 9Mhz*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	ADC_InitTypeDef ADC_InitStructure;
	ADC_DeInit(ADC1);
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5); //9Mhz/(239.5 + 12.5) = 35.7Khz
	//ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
}
//init rsu_can
void RSU_CAN_Init(void)
{
	can_cfg_t cfg_pdi_can = {
		.baud = 500000,
	};
	rsu_can_bus -> init(&cfg_pdi_can);	// init can
	can_filter_t filter[] = {
		{
			.id = 0x5a0,
			.mask = 0xffff,
			.flag = 0,
		},
	};
	rsu_can_bus -> filt(filter, 1);
}
//rsu init
void RSU_Init(void)
{
	RSU_Uart_Init();
	RSU_Interface_Init();
	RSU_CAN_Init();
} 
//rsu read can message
int RSU_Can_Read(can_msg_t *msg)
{
	time_t deadtime = time_get(100);
	while(time_left(deadtime) > 0) {	
		if(!rsu_can_bus -> recv(msg)) {
			printf("Read date from ECU :\r\n");
			return 0;
		}
	}
	printf("Nothing Read from ECU\r\n");
	return 1;
}
//explain the can message
MSG_T RSU_Can_Explain(can_msg_t *can_msg)
{
	MSG_T msg;
	memset(&msg,0,sizeof(msg));
	int temp = 0;
	temp = (can_msg -> data[0] << 16) | (can_msg -> data[1] << 8) | (can_msg -> data[2]);
	printf("0x%x\r\n",temp);
	position_rsu = Position_Get();
	msg.BYTE_FLAG.FLAG0 = (temp >> 22) & 0x01;
	if(*position_rsu == LEFT) {
		msg.BYTE_FLAG.FLAG1 = (temp >> 21) & 0x03;
		msg.BYTE_FLAG.FLAG2 = (temp >> 19) & 0x03;
		msg.BYTE_FLAG.FLAG3 = (temp >> 17) & 0x03;
		msg.BYTE_FLAG.FLAG4 = (temp >> 15) & 0x03;	
	} else if(*position_rsu == RIGHT) {
		msg.BYTE_FLAG.FLAG5 = (temp >> 13) & 0x03;
		msg.BYTE_FLAG.FLAG6 = (temp >> 11) & 0x03;
		msg.BYTE_FLAG.FLAG7 = (temp >> 9) & 0x03;
		msg.BYTE_FLAG.FLAG8 = (temp >> 7) & 0x03;
	}
	return msg;
}
//set beep on
void RSU_Beep_On()
{
	GPIOB -> ODR |= BEEP_ON;
	return ;
}
//reset beep on
void RSU_Beep_Off()
{
	GPIOB -> ODR &= ~BEEP_ON;
	return ;
}
/*//<--
int CounterA_Rise()
{
	GPIOB -> ODR |= COUNTERA;
	return 0;
}
//-->
int CounterA_Down()
{
	GPIOB -> ODR &= ~COUNTERA;
	return 0;
}
//<--
int CounterB_Rise()
{
	GPIOB -> ODR |= COUNTERB;
	return 0;
}
//-->
int CounterB_Down()
{
	GPIOB -> ODR &= ~COUNTERB;
	return 0;
}
//counter A ++
void RSU_CounterA_Add()
{
	CounterA_Rise();
	//RSU_mdelay(40);
	for(int i = 500000; i > 0; i--);
	CounterA_Down();
	for(int i = 500000; i > 0; i--);
}
//counter B ++
void RSU_CounterB_Add()
{
	CounterB_Rise();
	//RSU_mdelay(40);
	for(int i = 500000; i > 0; i--);
	CounterB_Down();
	for(int i = 500000; i > 0; i--);
}*/
//set A relay 
void RSU_RelayA_Set()
{
	GPIOC -> ODR |= TESTA;
}
//reset A relay
void RSU_RelayA_Reset()
{
	GPIOC -> ODR &= ~TESTA;
}
//set B relay
void RSU_RelayB_Set()
{
	GPIOE -> ODR |= TESTB;
}
//reset B relay
void RSU_RelayB_Reset()
{
	GPIOE -> ODR &= ~TESTB;
}
//
void RSU_Power_On()
{
	GPIOE -> ODR |=	POWER; 
	printf("Power on successful!\r\n");
}
//
void RSU_Power_Off()
{
	GPIOE -> ODR &= ~POWER;
	printf("Power off successful!\r\n");
}
//judge A target
int RSU_TargetA_On()
{
	if((GPIOE -> IDR & TARGETA) == 0)
		return 0;  //in the left position
	else
		return 1;
}
//judge B target
int RSU_TargetB_On()
{
	if((GPIOE -> IDR & TARGETB) == 0)
		return 0;  //in the right position
	else
		return 1;
}
//flash the led and beep 
void RSU_check_action()
{
	for(int i = 1; i < 9; i++) {
		led_on((led_t)(LED_GREEN + 3 * i));
		led_on((led_t)(LED_RED + 3 * i));
	}
	RSU_Beep_On();
	RSU_mdelay(200);
	for(int i = 1; i < 9; i++) {
		led_off((led_t)(LED_GREEN + 3 * i));
		led_off((led_t)(LED_RED + 3 * i));
	}
	RSU_Beep_Off();
	RSU_mdelay(100);
	for(int i = 0; i < 5; i++){
		for(int j = 1; j < 9; j++) {
			led_on((led_t)(LED_GREEN + 3 * j));
			led_on((led_t)(LED_RED + 3 * j));
		}
		RSU_mdelay(200);
		for(int j = 1; j < 9; j++) {
			led_off((led_t)(LED_GREEN + 3 * j));
			led_off((led_t)(LED_RED + 3 * j));
		}
		RSU_mdelay(100);
	}
}
void RSU_Start_Action()
{	
	position_rsu = Position_Get();
	if(*position_rsu == LEFT){
		for(int i = 0; i < 4; i++) {
			led_off((led_t)(LED_GREEN1 + 3 * i));
			led_off((led_t)(LED_RED1 + 3 * i));
		}
		led_Update_Immediate();
		for(int i = 0; i < 4; i++) {
			led_flash((led_t)(LED_GREEN1 + 3 * i));
			led_flash((led_t)(LED_RED1 + 3 * i));
		}
	} else{
		for(int i = 0; i < 4; i++) {
			led_off((led_t)(LED_GREEN5 + 3 * i));
			led_off((led_t)(LED_RED5 + 3 * i));
		}
		led_Update_Immediate();
		for(int i = 0; i < 4; i++) {
			led_flash((led_t)(LED_GREEN5 + 3 * i));
			led_flash((led_t)(LED_RED5 + 3 * i));
		}
	}
}
//read rsu status
//use bit operation to save memory 
//return the related position information
EXIST_T RSU_Read_Status(FIXTURE_STATUS_T *fixture_status)
{
	fixture_status -> BYTEBIT.BIT1 = (GPIOA -> IDR & (1 << RSU0)) >> RSU0;
	fixture_status -> BYTEBIT.BIT2 = (GPIOA -> IDR & (1 << RSU1)) >> RSU1;
	fixture_status -> BYTEBIT.BIT3 = (GPIOA -> IDR & (1 << RSU2)) >> RSU2;
	fixture_status -> BYTEBIT.BIT4 = (GPIOA -> IDR & (1 << RSU3)) >> RSU3;
	fixture_status -> BYTEBIT.BIT5 = (GPIOA -> IDR & (1 << RSU4)) >> RSU4;
	fixture_status -> BYTEBIT.BIT6 = (GPIOA -> IDR & (1 << RSU5)) >> RSU5;
	fixture_status -> BYTEBIT.BIT7 = (GPIOB -> IDR & (1 << RSU6)) >> RSU6;
	fixture_status -> BYTEBIT.BIT8 = (GPIOB -> IDR & (1 << RSU7)) >> RSU7;
	printf("0x%x\r\n", fixture_status -> BYTE);
	printf("%d %d %d %d %d %d %d %d\r\n",fixture_status -> BYTEBIT.BIT1,fixture_status -> BYTEBIT.BIT2, \
										fixture_status -> BYTEBIT.BIT3,fixture_status -> BYTEBIT.BIT4,	\
										fixture_status -> BYTEBIT.BIT5,fixture_status -> BYTEBIT.BIT6,	\
										fixture_status -> BYTEBIT.BIT7,fixture_status -> BYTEBIT.BIT8);
	position_rsu = Position_Get();
	char temp = 0;
	if(*position_rsu == LEFT) {
		temp = fixture_status -> BYTE_HBYTE.BYTEL;
	} else if(*position_rsu == RIGHT) {
		temp = fixture_status -> BYTE_HBYTE.BYTEH;
	}
	//calculate the num of rsu
	EXIST_T counter;
	for(int i = 0; i < 4; i++) {
		counter.p[i] = EXIST_NO;
	}
	counter.counter = 0;
	//value 1 -> EXIST_NO : No RSU Placed
	//value 0 -> EXIST_YES : RSU Placed
	for(int i = 0; i < 4; i++) {
		if((temp & (1 << i)) != 0) {
			counter.p[i] = EXIST_NO;
		} else {
			counter.p[i] = EXIST_YES;
			counter.counter ++;
		}
	}
	return counter;
}
//read the power voltage about 12v
void RSU_Read_Power(ADC_DATE_T *adc_array)
{
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	RSU_mdelay(5);
	adc_array -> sum = 0;
	adc_array -> average = 0.0;
	for(int i =0; i < 16; i++) {
		adc_array -> a[i] = ADC1 -> DR;
		adc_array -> b[i] = adc_array -> a[i] * 3.3 / 4095;
		adc_array -> sum += adc_array -> a[i];
		adc_array -> average += adc_array -> b[i];
	}
	adc_array -> average = adc_array -> average * 11.0 / 16; // 10K & 1K
	ADC_SoftwareStartConvCmd(ADC1,DISABLE);
}
//
void RSU_Result(PRODUCT_T product)
{
	position_rsu = Position_Get();
	if(*position_rsu == LEFT) {
		for(int i = 0; i < 4; i++) {
			switch(product.product[i]) {
				case OK:
					led_on((led_t)(LED_GREEN1 + 3 * i));
					led_off((led_t)(LED_RED1 + 3 * i));
					break;
				//case NOK:
					//led_off((led_t)(LED_GREEN1 + 3 * i));
					//led_on((led_t)(LED_RED1 + 3 * i));
					//break;
				case NC:
					led_off((led_t)(LED_GREEN1 + 3 * i));
					led_off((led_t)(LED_RED1 + 3 * i));
					break;
				default:
					break;
			}
		}
		error_result_rsu = Error_Get();
		//fail
		//if((product.product[0] == NOK) || (product.product[1] == NOK) || (product.product[2] == NOK) || (product.product[3] == NOK) || (*error_result_rsu != ERROR_NO)) {
		if((*error_result_rsu == ERROR_FUNC) || (*error_result_rsu == ERROR_NUM) || (*error_result_rsu == ERROR_ITAC) || (*error_result_rsu == ERROR_BACHU) || (*error_result_rsu == ERROR_ECUNG) || (*error_result_rsu == ERROR_POSITION)) {
			for(int i = 0; i < 4; i++) {
				led_on((led_t)(LED_RED1 + i * 3));
				led_off((led_t)(LED_GREEN1 + i * 3));
			}
			RSU_Beep_On();
			RSU_mdelay(1000);
			RSU_Beep_Off();
		} else {
			RSU_Beep_On();
			RSU_mdelay(200);
			RSU_Beep_Off();
			RSU_mdelay(200);
			RSU_Beep_On();
			RSU_mdelay(200);
			RSU_Beep_Off();
		}	
	} else if(*position_rsu == RIGHT) {
		for(int i = 4; i < 8; i++) {
			switch(product.product[i]) {
				case OK:
					led_on((led_t)(LED_GREEN1 + 3 * i));
					led_off((led_t)(LED_RED1 + 3 * i));
					break;
				//case NOK:
				//	led_off((led_t)(LED_GREEN1 + 3 * i));
				//	led_on((led_t)(LED_RED1 + 3 * i));
				//	break;
				case NC:
					led_off((led_t)(LED_GREEN1 + 3 * i));
					led_off((led_t)(LED_RED1 + 3 * i));
					break;
				default:
					break;
			}
		}
		//fail
		error_result_rsu = Error_Get();
		//if((product.product[4] == NOK) || (product.product[5] == NOK) || (product.product[6] == NOK) || (product.product[7] == NOK) || *error_result_rsu != ERROR_NO) {
		if((*error_result_rsu == ERROR_FUNC) || (*error_result_rsu == ERROR_NUM) || (*error_result_rsu == ERROR_ITAC) || (*error_result_rsu == ERROR_BACHU) || (*error_result_rsu == ERROR_ECUNG) || (*error_result_rsu == ERROR_POSITION)) {
			for(int i = 0; i < 4; i++) {
				led_on((led_t)(LED_RED5 + i * 3));
				led_off((led_t)(LED_GREEN5 + i * 3));
			}
			RSU_Beep_On();
			RSU_mdelay(1000);
			RSU_Beep_Off();
		} else {
			RSU_Beep_On();
			RSU_mdelay(200);
			RSU_Beep_Off();
			RSU_mdelay(200);
			RSU_Beep_On();
			RSU_mdelay(200);
			RSU_Beep_Off();
		}
	}
}
#if 1
#if defined ( __CC_ARM   ) /*------------------RealView Compiler -----------------*/ 
__asm void GenerateSystemReset(void)
{
	MOV R0, #1           //;
	MSR FAULTMASK, R0    //; FAULTMASK
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
static int cmd_rsu_func(int argc, char *argv[])
{
	const char *usage = {
		" rsu , usage:\r\n"
		" rsu init \r\n"
		" rsu reset \r\n"
		" rsu read \r\n"
		" rsu precheck \r\n"
		" rsu set relay A or B\r\n"
		" rsu set beep on \r\n"
		" rsu set led x y    x:1~8 y:red,green\r\n"
		" rsu set counter x\r\n"
		" rsu read rsu \r\n"
		" rsu read power \r\n"
		" rsu target \r\n"
		" rsu set power on\r\n"
		" rsu set power off\r\n"
	};

	if (argc < 2) {
		printf("%s", usage);
		return 0;
	}
	
	if(argc == 2) {
		if(!strcmp(argv[1],"init")) {
			printf("ok!\r\n");
			RSU_Init();
		}
		if(!strcmp(argv[1],"read")) {
			RSU_Power_On();
			RSU_mdelay(7000);
			can_msg_t msg_temp = {0,8,{0,0,0,0,0,0,0,0},0};
			MSG_T msg;
			memset(&msg,0,sizeof(msg));
			RSU_Can_Read(&msg_temp);
			RSU_Can_Read(&msg_temp);
			RSU_Can_Read(&msg_temp);
			RSU_Can_Read(&msg_temp);
			can_msg_print(&msg_temp,"\n");
			msg = RSU_Can_Explain(&msg_temp);
			position_rsu = Position_Get();
			if(*position_rsu == LEFT) {
				printf("%d\r\n",msg.BYTE_FLAG.FLAG1);
				printf("%d\r\n",msg.BYTE_FLAG.FLAG2);
				printf("%d\r\n",msg.BYTE_FLAG.FLAG3);
				printf("%d\r\n",msg.BYTE_FLAG.FLAG4);
			} else if(*position_rsu == RIGHT) {
				printf("%d\r\n",msg.BYTE_FLAG.FLAG5);
				printf("%d\r\n",msg.BYTE_FLAG.FLAG6);
				printf("%d\r\n",msg.BYTE_FLAG.FLAG7);
				printf("%d\r\n",msg.BYTE_FLAG.FLAG8);
			}
			RSU_Power_Off();
		}
		if(!strcmp(argv[1],"reset")) {
			GenerateSystemReset();
		}
		if(!strcmp(argv[1],"precheck")) {
			printf("ok!\r\n");
			RSU_check_action();
		}
		if(!strcmp(argv[1],"target")) {
			position_rsu = Position_Get();
			if(*position_rsu == LEFT) {
				if(!RSU_TargetA_On()) {
					printf("A target\r\n");
				} else {
					printf("A not target\r\n");
				}
			} else if(*position_rsu == RIGHT) {
				if(!RSU_TargetB_On()) {
					printf("B target\r\n");
				} else {
					printf("B not target\r\n");
				}				
			}
		}
	}
	if(argc == 3) {
		if((!strcmp(argv[1],"read"))&&(!strcmp(argv[2],"rsu"))) {
			printf("ok!\r\n");
			RSU_RelayA_Set();
			RSU_RelayB_Set();
			RSU_mdelay(10);
			EXIST_T counter;
			FIXTURE_STATUS_T fixture_status_temp;
			//memset(&fixture_status_temp, 0 ,sizeof(fixture_status_temp));
			fixture_status_temp.BYTE = 0;
			counter = RSU_Read_Status(&fixture_status_temp);
			position_rsu = Position_Get();
			if(*position_rsu == LEFT) {
				printf("%d\r\n",fixture_status_temp.BYTE_HBYTE.BYTEL);
			} else if(*position_rsu == RIGHT) {
				printf("%d\r\n",fixture_status_temp.BYTE_HBYTE.BYTEH);
			}
			for(int i = 0; i < 4; i++) {
				printf("%d\r\n",counter.p[i]);
			}
			printf("All the rsu is %d\r\n",counter.counter);
		}
		if((!strcmp(argv[1],"read"))&&(!strcmp(argv[2],"power"))) {
			ADC_DATE_T adc_temp;
			RSU_Read_Power(&adc_temp);
			printf("%f\r\n",adc_temp.average);
		}
	}
	if(argc == 4) {
		if((!strcmp(argv[2],"relay"))&&(!strcmp(argv[1],"set"))) {
			if(!strcmp(argv[3],"A")) {
				RSU_RelayA_Set();
				printf("Relay A set successful !\r\n");
			} else if(!strcmp(argv[3],"B")) {
				RSU_RelayB_Set();
				printf("Relay B set successful !\r\n");
			} else {
				printf("Please input again !\r\n");
			}
		}
		if((!strcmp(argv[2],"relay"))&&(!(strcmp(argv[1],"reset")))) {
			if(!strcmp(argv[3],"A")) {
				RSU_RelayA_Reset();
				printf("Relay A reset successful !\r\n");
			} else if(!strcmp(argv[3],"B")) {
				RSU_RelayB_Reset();
				printf("Relay B reset successful !\r\n");
			} else {
				printf("Please input again !\r\n");
			}
		}
		if((!strcmp(argv[2],"power"))&&(!strcmp(argv[3],"on"))) {
			RSU_Power_On();
			printf("Set power on!\r\n");
		}
		if((!strcmp(argv[2],"power"))&&(!strcmp(argv[3],"off"))) {
			RSU_Power_Off();
			printf("Set power off!\r\n");
		}
		if(!strcmp(argv[2],"beep")) {
			if(!strcmp(argv[3],"on")) {
				RSU_Beep_On();
				RSU_mdelay(200);
				RSU_Beep_Off();
				RSU_mdelay(200);
				RSU_Beep_On();
				RSU_mdelay(200);
				RSU_Beep_Off();
			}
		}
		if(!strcmp(argv[2],"counter")) {
			if(!strcmp(argv[3],"A")) {
				//RSU_CounterA_Add();
				//CounterA_Rise();
				printf("set counter A successful !\r\n");
			} else if(!strcmp(argv[3],"B")) {
				//RSU_CounterB_Add();
				printf("set counter B successful !\r\n");
			} else {
				printf("Please Input again !\r\n");
			}
		}
	}
	int index = 1;
	led_t result = NR_OF_LED;
	if(argc == 5) {
		if((!strcmp(argv[1],"set"))&&(!strcmp(argv[2],"led"))) {
			sscanf(argv[3],"%d",&index);
			result = (strcmp(argv[4],"green") == 0)?LED_GREEN: \
				((strcmp(argv[4],"red") == 0)?LED_RED:LED_YELLOW);
			led_on((led_t)(result + 3 * index));
		}
		if((!strcmp(argv[1],"reset"))&&(!strcmp(argv[2],"led"))) {
			sscanf(argv[3],"%d",&index);
			result = (strcmp(argv[4],"green") == 0)?LED_GREEN: \
				((strcmp(argv[4],"red") == 0)?LED_RED:LED_YELLOW);
			led_off((led_t)(result + 3 * index));
		}
	}
	return 0;
}

const cmd_t cmd_rsu = {"rsu", cmd_rsu_func, "rsu cmd i/f"};
DECLARE_SHELL_CMD(cmd_rsu)
#endif