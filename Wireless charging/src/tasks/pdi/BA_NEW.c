/*
 *David peng.guo@2011 initial version
 *jiamao.gu@2013 5.29 change
 *the length of barcode must larger one byte than the SN: this one time, Max3232 5V is forget, 12V is forget too
 *SN is the barcode!!!!
 *add filter in usdt.c 
 *add gpio config in drv.c and usdt.c like CONFIG_PDI_ba_gujiamao 
 *Read error code: buffer must clear before reading and if read 0 0 likewise, break the function
 *in limit file , error expected must captical, even if display in little mode
 *jiamao.gu@2013 9.16 change
 *rewrite the function of check error
 *rewrite the function of set relay 
 *add the communication woth Xuce Company's Up Computer(Step0,Step1,Step2,Step3)
 *add the Ping Pang Operation for the autoliv
 *optimize the barcode length, need change once ahead not as usual
 *no need to read the barcode by MCU, directly get by Up computer, so the uart2 is used for kline
 *
 */

#include <string.h>
#include "config.h"
#include "sys/task.h"
#include "ulp/sys.h"
#include "ulp_time.h"
#include "can.h"
#include "drv.h"
#include "ls1203.h"
#include "cfg.h"
#include "priv/usdt.h"
#include "ulp/debug.h"
#include "shell/cmd.h"
#include "common/debounce.h"
#include "pdi.h"
#include "stdio.h"


#define PDI_DEBUG		0        // Test dubug by putty
#define PDI_AUTOLIV     0        // Test debug by AccessPort
#define Length_Barcode 14        // TBD for different Product
//pdi ba can msg
static const can_msg_t ba_clrdtc_msg =		{0x607, 8, {0x04, 0x2e, 0xfe, 0x90, 0xaa, 0, 0, 0}, 0};
//static const can_msg_t ba_resis =			{0x607, 8, {0x03, 0x22, 0xfe, 0x80, 0, 0, 0, 0}, 0};
#if PDI_DEBUG
static const can_msg_t ba_errcode_msg =		{0x607, 8, {0x03, 0x22, 0xfe, 0x80, 0, 0, 0, 0}, 0};
static const can_msg_t ba_getsn_msg =		{0x607, 8, {0x03, 0x22, 0xfe, 0x8d, 0, 0, 0, 0}, 0};
static const can_msg_t ba_part_msg =		{0x7D2, 8, {0x03, 0x22, 0xF1, 0x00, 0x55, 0x55, 0x55, 0x55}, 0};
#endif
static const can_msg_t ba_reqseed_msg =		{0x607, 8, {0x02, 0x27, 0x7d, 0, 0, 0, 0, 0}, 0};
static const can_msg_t ba_start_msg1 =		{0x607, 8, {0x02, 0x10, 0x03, 0, 0, 0, 0, 0}, 0};
//static const can_msg_t ba_start_msg2 =		{0x7D2, 8, {0x02, 0x10, 0x03, 0, 0, 0, 0, 0}, 0};
//static const can_msg_t ba_7D2_msg =			{0x7D2, 8, {0x30, 0, 0, 0, 0, 0, 0, 0},0};
static const autoliv_t pdi_autoliv = {
		.bus = &uart3,
		.data_len = 200,              // Xuce Up computer config file max length < 200
		.dead_time = 20
};
static const can_bus_t* pdi_can_bus = &can1;
static can_msg_t ba_msg_buf[32];		// for multi frame buffer
static char ba_data_buf[256];			// data buffer
static char ba_fault_buf[64];			// fault buffer
static char bcodea[Length_Barcode + 1];	// for A LEFT
static char bcodeb[Length_Barcode + 1];	// for B RIGHT

static char autoliv_buf[256];          	// for Xuce Up computer buffer
config_t autoliv ;						// for Xuce config file
FUNCTION_T Function[10];  				// max 10 functions address
POSITION_E position = LEFT; 			// left position

static struct debounce_s target1;
static struct debounce_s target2;
ERROR_E error_result = ERROR_NO;
INFORM_E inform_result = INFORM_NO;
FIXTURE_E SlotA = IDLE, SlotB = IDLE;   // for Ping Pang Operation

//get data from ECU
static int ba_GetCID(short cid, char *data);
static int ba_GetFault(char *data, int *pnum_fault);
//communicate with up computer, drive function 
INFORM_E Step0(void);
INFORM_E Step1(void);
INFORM_E Step2(void);
INFORM_E Step3(void);   // for receiving ITAC-OK or ITAC-NG
//communicate with up computer, app function
void DealAfterReceive(FUNCTION_T *FUNCITON); // Step0 Step1 Step2 
void DealAfterStop(FUNCTION_T *FUNCITON, time_t deadline);    //Step3 
//delay n ms
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
// get left or right position for other c file like pdi.c and drive.c
POSITION_E *Position_Get(void)
{
	return &position;
}

/*start session*/
int pdi_startsession(void)
{
#if PDI_DEBUG
	int i, num_fault;
#endif
	int msg_len;
	unsigned char seed[2], result;
	can_msg_t msg;
	can_msg_t sendkey_msg = {
		0x607,
		8,
		{0x04, 0x27, 0x7e, 0xff, 0xff, 0, 0, 0},
		0
	}; //TBD

	if (usdt_GetDiagFirstFrame(&ba_start_msg1, 1, NULL, &msg, &msg_len))		//start session
		return 1;
#if PDI_DEBUG
	can_msg_print(&msg, "\n");
#endif
        
    if (usdt_GetDiagFirstFrame(&ba_start_msg1, 1, NULL, &msg, &msg_len))		//start session
		return 1;
#if PDI_DEBUG
	can_msg_print(&msg, "\n");
#endif
        
	if (usdt_GetDiagFirstFrame(&ba_reqseed_msg, 1, NULL, &msg, &msg_len))		//req seed
		return 1;
#if PDI_DEBUG
	can_msg_print(&msg, "\n");
#endif

	/*calculate the key from seed*/
	seed[0] = (unsigned char)msg.data[3];
	seed[1] = (unsigned char)msg.data[4];
	result = seed[0] ^ seed[1];
	result ^= 0x23;  // 0x34
	result ^= 0x9a;  // 0xab
	sendkey_msg.data[3] = (char)((0x239a + result) >> 8);
	sendkey_msg.data[4] = (char)((0x239a + result) & 0x00ff);
	/*send key*/
	if (usdt_GetDiagFirstFrame(&sendkey_msg, 1, NULL, &msg, &msg_len))
		return 1;
	/*judge the send key response*/
	if ((msg.data[1] != 0x67) || (msg.data[2] != 0x7e))//sendkey_msg[1]+0x40 and sendkey_msg[2]
		return 1;
#if PDI_DEBUG
	can_msg_print(&msg, "\n");
#endif

#if PDI_DEBUG
	/*get serial number*/
	printf("\nSN Code:\n");
	usdt_GetDiagFirstFrame(&ba_getsn_msg, 1, NULL, ba_msg_buf, &msg_len);
	if (msg_len > 1)
		usdt_GetDiagLeftFrame(ba_msg_buf, msg_len);
	for (i = 0; i < msg_len; i ++)
		can_msg_print(ba_msg_buf + i, "\n");

	/*get error code*/
	printf("\nError Code:\n");
	usdt_GetDiagFirstFrame(&ba_errcode_msg, 1, NULL, ba_msg_buf, &msg_len);
	if (msg_len > 1)
		usdt_GetDiagLeftFrame(ba_msg_buf, msg_len);
	for (i = 0; i < msg_len; i++)
		can_msg_print(ba_msg_buf + i, "\n");
	/*tester point*/
	printf("\nPart:\n");
	usdt_GetDiagFirstFrame(&ba_getpart_msg, 1, NULL, ba_msg_buf, &msg_len);
	if (msg_len > 1)
		usdt_GetDiagLeftFrame(ba_msg_buf, msg_len);
	for (i = 0; i < msg_len; i++)
		can_msg_print(ba_msg_buf + i, "\n");

	if (ba_GetFault(ba_fault_buf, &num_fault))
		printf("##ERROR##\n");
	else {
		printf("##OK##\n");
		printf("num of fault is: %d\n", num_fault);
		for (i = 0; i < num_fault*2; i += 2)
			printf("0x%02x, 0x%02x\n", ba_fault_buf[i]&0xff, ba_fault_buf[i+1]&0xff);
	}
	/*clear all*/
	printf("\nClear all:\n");
	if (usdt_GetDiagFirstFrame(&ba_clrdtc_msg, 1, NULL, &msg, &msg_len))	//req seed
		return 1;
	can_msg_print(&msg, "\n");
#endif

	return 0;
}
#if 0
static int ba_error_check(char *fault, int *num, const struct pdi_cfg_s *sr)
{
	int i, j, flag;
	char ba_fault_temp[2];
	const struct pdi_rule_s* pdi_cfg_rule;
	for(i = 0; i < *num; i ++) {
		flag = 0;
		ba_fault_temp[0] = fault[i*2];
		ba_fault_temp[1] = fault[(i*2) + 1];
		for(j = 0; j < sr -> nr_of_rules; j ++) {
			pdi_cfg_rule = pdi_rule_get(sr, j);
			if(&pdi_cfg_rule == NULL) return 1;
			switch(pdi_cfg_rule -> type) {
				case PDI_RULE_ERROR:
					if(pdi_verify(pdi_cfg_rule, ba_fault_temp) == 0) {
						flag = 1;
						break;
					}
				case PDI_RULE_UNDEF:
					break;
			}
			if(flag == 1) break;
		}
		if(flag == 0) return 1;
	}
	return 0;
}
#endif
#if 1
//whether the error code is allowed
//To be improved 
static int ba_error_check(char *fault, int *num, const config_t sr)
{
	int i, j;
	short ba_fault_temp[32];
	memset(ba_fault_temp,'\0',32);
	short faultable_temp[10];
	memset(faultable_temp,'\0',10);
	int length1 = *num;
	int length2 = strlen(sr.errorable) / 2;
	for(i = 0; i < *num; i ++) {  
		ba_fault_temp[i] = fault[i * 2] << 8 | fault[i * 2 + 1];  // two char merged in a short 
	}
	for(j = 0; j < strlen(sr.errorable); j += 2) {
		faultable_temp[(j + 1) / 2] = sr.errorable[j] << 8 | sr.errorable[j + 1]; // two char merged in a short
	}
	for(i = 0; i < length1; i++) {
		for(j = 0; j < length2; j++) {
			if(ba_fault_temp[i] == faultable_temp[j]) { // compare one error with all the allowable codes
				break;                                  // if equal, out the circle 
			}
		}
		if(j == length2) {								//if one error code is not equal with all the allowable codes
			return 1;                                   //return 
		}
	}
	return 0;
}
#endif
/*get contents of cid,saved to data*/
static int ba_GetCID(short cid, char *data)
{
	can_msg_t msg_res, pdi_send_msg = {0x607, 8, {0x03, 0x22, 0xff, 0xff, 0, 0, 0, 0}, 0};
	int i = 0, msg_len;

	pdi_send_msg.data[2] = (char)(cid >> 8);
	pdi_send_msg.data[3] = (char)(cid & 0x00ff);
	if (usdt_GetDiagFirstFrame(&pdi_send_msg, 1, NULL, &msg_res, &msg_len))
		return 1;
        can_msg_print(&msg_res, "\n");
	if (msg_len > 1) {
		ba_msg_buf[0] = msg_res;
		if(usdt_GetDiagLeftFrame(ba_msg_buf, msg_len))
			return 1;
	}

	if (msg_len == 1) {
		if (msg_res.data[1] == 0x62)
			memcpy(data, (msg_res.data + 4), msg_res.data[0] - 3);
		else
			return 1;
	} else if (msg_len > 1) {
		memcpy(data, (msg_res.data + 5), 3);
		data += 3;
		for (i = 1; i < msg_len; i++) {
			memcpy(data, (ba_msg_buf + i)->data + 1, 7);
			data += 7;
		}
	}
	return 0;
}

/*checking barcode*/
static int ba_check_barcode()
{
	char read_bcode[Length_Barcode + 1];
	int try_times = 5;
	printf("##START##EC-Checking barcode...##END##\n");
	while (ba_GetCID(0xfe8d, ba_data_buf)) {
		try_times --;
		if (try_times < 0)
			return 1;
	}
	memcpy(read_bcode, ba_data_buf, Length_Barcode);
	printf("##START##RB-");
	printf(read_bcode,'\0');
	printf("##END##\n");
	if(position == LEFT) {
		if (memcmp(read_bcode, bcodea, Length_Barcode))  //maybe Length_Barcode - 1 ??
			return 1;
	}
	else {
		if (memcmp(read_bcode, bcodeb, Length_Barcode))
			return 1;
	}
	return 0;
}

/*checking Resis*/
/*static int ba_check_resis()
{
	int msg_len;
	can_msg_t msg;
	printf("##START##EC-Checking resis...##END##\n");
	if (usdt_GetDiagFirstFrame(&ba_resis, 1, NULL, &msg, &msg_len))
		return 1;
	if (msg_len > 1)
	usdt_GetDiagLeftFrame(ba_msg_buf, msg_len);
	for (int i = 0; i < msg_len; i ++)
		can_msg_print(ba_msg_buf + i, "\n");


	return 0;
}*/

/*clear error code*/
int pdi_clear_dtc(void)
{
	int msg_len;
	can_msg_t msg;
	pdi_startsession();
	if (usdt_GetDiagFirstFrame(&ba_clrdtc_msg, 1, NULL, &msg, &msg_len))
		return 1;
	if (msg.data[1] != 0x6E)	/*positive response is 0x7b*/
		return 1;
	return 0;
}

/*get fault from ECU,saved to data,the nr of fault is pnum_fault*/
static int ba_GetFault(char *data, int *pnum_fault)
{
	int i, result = 0;
	if (ba_GetCID(0xfe80, data))
		return 1;
	//memset(data + 117, 0x00, 10);
	for (i = 0; i < 117; i += 2) {
		if((data[i] | data[i+1]) == 0)
			break;
		if(data[i] == (char)(0xaa))    // maybe in null address, the data is 0xaa, to be confirmed
			break;
		result ++;
	}
	*pnum_fault = result;
	return 0;
}
/**/
void Function_Init(FUNCTION_T *FUNCTION)
{
	FUNCTION[1] = Step1;    // as 0101 head
	FUNCTION[2] = Step2;    // as 0202 head
	FUNCTION[3] = Step0;    // as 0303 head
	FUNCTION[0] = Step3;    // as ITAC- head
}
/*all the init include drive and so on init*/
void pdi_init(FUNCTION_T *FUNCTION)
{
	can_cfg_t cfg_pdi_can = {
		.baud = 500000,
	};
	pdi_relay_init();                   // mbi5024 init
	autoliv_init(&pdi_autoliv);         // init the uart for Xuce Up computer
	pdi_can_bus->init(&cfg_pdi_can);	// init can
	usdt_Init(pdi_can_bus);
	pdi_drv_Init();						// led,beep
	pre_check_action();					// flash the green1 red1 green2 red2 and beep
	config_init(autoliv);               // init the Config file like relay and limit file 
	
	debounce_init(&target1,10,0);       // for filter
	debounce_init(&target2,10,0);
	
	autoliv_send(&pdi_autoliv,"0k!!\r\n");// whether the uart ok?
	memset(autoliv_buf,0,sizeof(autoliv_buf));// clear the autoliv_buf, update in pdi_update and deal in DealAfterReceive and DealAfterStop
	memset(bcodea,0,sizeof(bcodea));
	memset(bcodeb,0,sizeof(bcodeb));
	Function_Init(FUNCTION);              // init the function point array
}
/*********************************************
** add a debounce function to filter the error
** get zero 10 times means target is ok ******
** after test, the status is COMPLETE ********
**********************************************/
void Status_transform(void)
{
	if((!target1_on()) && (SlotA == IDLE)) {   // on + IDLE --> READY
		SlotA = READY;
	}
	if((!target2_on()) && (SlotB == IDLE)) {   // on + IDLE --> READY
		SlotB = READY;
	}
	if(target1_on() && target2_on()) {         // default condition : IDLE
		SlotA = IDLE;
		SlotB = IDLE;
    }
	if(target1_on() && (SlotA == COMPLETE)) {  // off + COMPLETE --> IDLE
		SlotA = IDLE;
	}
	if(target2_on() && (SlotB == COMPLETE)) {	// off + COMPLETE --> IDLE
		SlotB = IDLE;
	}
}
/***********************************************
**Lock A : A target && A not be checked complete
**Lock B : B target && B not be checked complete
***********************************************/
void Lock_Fixture(void)
{												// maybe use in the future
}
/*
**A Product not target, the scanner is A**************
**B Product not target, the scanner is B**************
**Both A and B are not target, the scanner is A*******
**Both A and B are target, the scanner must be clear**
*/
void pdi_update(void)
{
	DealAfterReceive(Function);
	Status_transform();
	Lock_Fixture();
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
/***************************************
** according to the barcode from scanner
** pick up the fifth usually as the name
** of limit file or relay file *********
** copy all for barcode check **********
****************************************/
char *Pick_Barcode(char *barcode)
{
	barcode[14] = '\0';
	printf("##START##SB-");
	printf(barcode,'\0');             	// use happily long long ago
	printf("##END##\n");
	barcode[5] = '\0';
	printf("##START##STATUS-5##END##\n");
	return barcode;
}
/****************************************
** set the can/kline ********************
** pick up the barcode ****************** 
** set the relay according to relay file*
** return error information *************
*****************************************/
ERROR_E TestStart(void)
{
	pdi_can();							  // set the can relay
	pdi_IGN_on();                         // set the power relay or switch
	pdi_set_relays(autoliv);              // set the load relays, may exist bugs
	return ERROR_NO;
}
/******************************************
** Blink the yellow led *******************
** compare barcode from ECU and scaner ****
** must send the startsession before read**
** process bar running*********************
*/
ERROR_E TestBarcode(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	char processbar[2];
	int rate;
	start_action();
	/*delay 1s*/
	for (rate = 5; rate <= 17; rate ++) {
		pdi_mdelay(89);                      
		printf("##START##STATUS-");		// in such format, so the data can be displayed in processbar
		sprintf(processbar, "%d", rate);
		printf("%s", processbar);
		printf("##END##\n");
	}
	pdi_startsession();                 // startsession before getting message
	if (ba_check_barcode()) {
		error_result = ERROR_BW;
		return error_result;
	}
	printf("####EC-      Checking barcode Done...##END##\n");
	return ERROR_NO;
}
/**********************************************
** process bar running ************************
** read the fault from ECU ********************
** must send the startsession *****************
** check the error code to judge the result ***
***********************************************/
ERROR_E TestECU(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	char processbar[2];
	int rate;
	printf("##START##EC-Waiting for ECU ready...##END##\n");
	/*delay 7s*/
	for (rate = 17; rate <= 96; rate ++) {
		pdi_mdelay(89);
		printf("##START##STATUS-");
		sprintf(processbar, "%d", rate);
		printf("%s", processbar);
		printf("##END##\n");
	}
	memset(ba_fault_buf,0,sizeof(ba_fault_buf)); // clear this buffer, if not, you will see the bug
	int try_times = 0;
	int result = 0;
	int num_fault = 0;
	/**** when result is 1, must get fault again, oversize 3 times *****
	**** num_fault is not 0, check the error_code ********************** 
	*******************************************************************/
	pdi_startsession();
	do{
		result = ba_GetFault(ba_fault_buf,&num_fault);
		if(++try_times == 4) {
			error_result = ERROR_CW;
			return error_result;
		}
	}while(result == 1);
	if(!num_fault) {
		error_result = ERROR_NO;
		return error_result;
	}
	else {
		printf("##START##EC-");
		printf("num of fault is: %d\n", num_fault);
		for (int i = 0; i < num_fault * 2; i += 2)  // according to size of error code , sometimes 3 bytes 
			printf("0x%02x, 0x%02x\n", ba_fault_buf[i]&0xff,ba_fault_buf[i+1]&0xff);
		printf("##END##\n");
		if(ba_error_check(ba_fault_buf, &num_fault, autoliv)) { // check whether the error code is allowable
			error_result = ERROR_CODE;
		}
		else {
			error_result = ERROR_ENCODE;
		}
		return error_result;
	}       
}
/********************************************************
********After Power off, check the can communication*****
*********************************************************/
void Poweroff()
{
	int msg_len;
	can_msg_t msg;
	pdi_IGN_off();
	/*do{                                // if no IMU CAN
		pdi_mdelay(1000);
	}while(!usdt_GetDiagFirstFrame(&ba_start_msg1, 1, NULL, &msg, &msg_len));
	*/
	pdi_mdelay(1000);                    // 
}
/******************************
** change the status **********
** unlock the fixture *********
** display the led ************
******************************/
void TestStop(void)
{
	//pdi_unlock();
	Poweroff();
	if(position == LEFT) {
		switch(error_result) {
			case ERROR_NO:
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-OK\r\n");
				printf("UUTA-FUNC-OK\r\n");
				break;
			case ERROR_NRP:                                  // not happened
				printf("##START##EC-Target A is not on the right position##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG\r\n");
				printf("UUTA-FUNC-NG\r\n");
				break;
			case ERROR_NCF:                                  // not happened
				printf("##START##EC-No This Config File A ##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG\r\n");
				printf("UUTA-FUNC-NG\r\n");
				break;
			case ERROR_BW:
				printf("##START##EC-barcode A Wrong##END##\n");
				printf("UUTA-FUNC-NG\r\n");
				break;
			case ERROR_CW:
				printf("##START##STATUS-Get A DTC error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG\r\n");
				printf("UUTA-FUNC-NG\r\n");
				break;
			case ERROR_CODE:
				printf("##START##STATUS-Get A DTC error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG\r\n");
				printf("UUTA-FUNC-NG\r\n");
				break;
			case ERROR_ENCODE:
				printf("##START##EC-These faults are acceptale...##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-OK\r\n");
				printf("UUTA-FUNC-OK\r\n");
				break;
			default:
				break ;
		}
	}
	else {
		switch(error_result) {
			case ERROR_NO:
				printf("UUTB-FUNC-OK\r\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-OK\r\n");
				break;
			case ERROR_NRP:
				printf("##START##EC-Target B is not on the right position##END##\n");
				printf("UUTB-FUNC-NG\r\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG\r\n");			
				break;
			case ERROR_NCF:
				printf("##START##EC-No This Config File B##END##\n");
				printf("UUTB-FUNC-NG\r\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG\r\n");
				break;
			case ERROR_BW:
				printf("##START##EC-barcode B Wrong##END##\n");
				break;
			case ERROR_CW:
				printf("##START##STATUS-Get B DTC error!##END##\n");
				printf("UUTB-FUNC-NG\r\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG\r\n");
				break;
			case ERROR_CODE:
				printf("##START##STATUS-Get B DTC error!##END##\n");
				printf("UUTB-FUNC-NG\r\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG\r\n");
				break;
			case ERROR_ENCODE:
				printf("##START##EC-These faults are acceptale...##END##\n");
				printf("UUTB-FUNC-OK\r\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-OK\r\n");			
				break;
			default:
				break ;
		}
	}
}
/**/
void DealAfterReceive(FUNCTION_T *FUNCTION)
{
	memset(autoliv_buf,0,sizeof(autoliv_buf));   // Global variable attention!!!
	if(!autoliv_get(&pdi_autoliv,autoliv_buf)) { // All the receive data put in autoliv_buf
		#if PDI_AUTOLIV 
		autoliv_send(&pdi_autoliv,autoliv_buf);  // Retrun the data, just for debug
		#endif
		if(autoliv_buf[0] > 0x00 && autoliv_buf[0] < 0x09)
			FUNCTION[autoliv_buf[0]]();                  // Deal with the data
		else 
			autoliv_send(&pdi_autoliv,"Frame Head Error\r\n");
	}
}
/**/
void DealAfterStop(FUNCTION_T *FUNCTION, time_t dead_time)
{
	memset(autoliv_buf,0,sizeof(autoliv_buf));   // Global variable attention!!!
	time_t over_time;
	over_time = time_get(dead_time);
	while(time_left(over_time) > 0) {
		if(!autoliv_get(&pdi_autoliv,autoliv_buf)) { // All the receive data put in autoliv_buf
			#if PDI_AUTOLIV 
			autoliv_send(&pdi_autoliv,autoliv_buf);  // Retrun the data, just for debug
			#endif
			if (autoliv_buf[2] == 'I') {
				FUNCTION[0]();                       // Receiving the "ITAC-OK or NG", retrun
				return ;
			}
		}
	}
	printf("ITAC Overtime \r\n");
	if(error_result == ERROR_NO || error_result == ERROR_ENCODE) {
		pdi_pass_action();
	}
	else {
		pdi_fail_action();
	}
}
/*****************************************
*** Up:Send 0303ECUPDIID\r\n *************
*** Down:Send ECUPDIIDE003\r\n ***********
******************************************/
INFORM_E Step0(void)
{
	char autoliv_buffer[256];                   // Temp variable
	memset(autoliv_buffer,0,sizeof(autoliv_buffer));
	for(int i = 0; i < strlen("ECUPDIID"); i++) {
		autoliv_buffer[i] = autoliv_buf[i + 2]; // Jump out 0303
	}
	autoliv_send(&pdi_autoliv,"ECUPDIIDE001");  // Send to Up
	printf("%s\r\n","ECUPDIIDE001");
	return INFORM_NO;
}
/****************************************************************
*** Up: Send 01010f+part_code+relay+softversion+IMU+error+\r\n***
*** Down: Send "DL-CONF-OK" *************************************
*****************************************************************/
INFORM_E Step1(void)
{
	//Clear the struct
	config_init(autoliv);
	for(int i = 0; i < length_bcode_part; i++) {
		autoliv.bcode_part[i] = autoliv_buf[i + 3];   // Jump out 03030f
	}
	#if PDI_AUTOLIV
	autoliv_send(&pdi_autoliv,autoliv.bcode_part);        // Just for debug
	autoliv_send(&pdi_autoliv,"\r\n");            // Just for debug
	#endif
	printf("Rx:part barcode : \r\n");
	printf("%s\r\n",autoliv.bcode_part);
	printf("Rx:relay :\r\n");
	for(int i = 0; i < length_relay; i++) {
		autoliv.relay[i] = autoliv_buf[i + 3 + length_bcode_part]; // Jump out 01010f + bcode_part
		printf("%x ",autoliv.relay[i]);
		if((i + 1) % 6 == 0) {
			printf("\r\n");
		}
	}
	#if PDI_AUTOLIV
	autoliv_send(&pdi_autoliv,autoliv.relay);     // Just for debug
	autoliv_send(&pdi_autoliv,"\r\n");			  // Just for debug
	#endif
	for(int i = 0; i < length_softversion; i++) {
		autoliv.softversion[i] = autoliv_buf[i + 3 + length_bcode_part + length_relay]; // Jump out 01010f + bcode_part + relay
	}
	#if PDI_AUTOLIV
	autoliv_send(&pdi_autoliv,autoliv.softversion);
	autoliv_send(&pdi_autoliv,"\r\n");
	#endif
	printf("Rx:softversion :\r\n");
	printf("%s\r\n",autoliv.softversion);
	autoliv.IMU = autoliv_buf[3 + length_bcode_part + length_relay + length_softversion];
	int i = 0; 
	printf("Rx:errorable :\r\n");
	while((autoliv_buf[i + 3 + length_bcode_part + length_relay + length_softversion + 1 ] != 0x0D) && (autoliv_buf[i + 3 + length_bcode_part + length_relay + length_softversion + 2] != 0x0A)) {
		autoliv.errorable[i] = autoliv_buf[i + 3 + length_bcode_part + length_relay + length_softversion + 1];
		printf("%x",autoliv.errorable[i]);
		if((i + 1) % 6 == 0) {
			printf("\r\n");
		}
		i++;
	}
	#if PDI_AUTOLIV
	autoliv_send(&pdi_autoliv,autoliv.errorable);
	autoliv_send(&pdi_autoliv,"\r\n");
	#endif
	autoliv_send(&pdi_autoliv,"DL-CONF-OK");
	printf("DL-CONF-OK");
	#if PDI_AUTOLIV
	autoliv_send(&pdi_autoliv,"\r\n");
	#endif
	return INFORM_NO;
}
/*************************************************************
**** Up: Send 0202+barcode+\r\n ******************************
**** Down: Send "BARC-OK\r\n" or "BARC-NG\r\n" ***************
*************************************************************/
INFORM_E Step2(void)
{
	char autoliv_buffer[256];                   // Temp variable
	memset(autoliv_buffer,0,sizeof(autoliv_buffer));
	int i = 0;
	while((autoliv_buf[i + 2] != 0x0D) && (autoliv_buf[i + 3] != 0x0A)) {
		autoliv_buffer[i] = autoliv_buf[i + 2]; 
		i++;
	}
	/*if((SlotA == IDLE) && (SlotB != IDLE)) {
		memcpy(bcodea,autoliv_buffer,Length_Barcode);
	}
	if((SlotB == IDLE) && (SlotA != IDLE)) {
		memcpy(bcodeb,autoliv_buffer,Length_Barcode);
	}
	if((SlotA == IDLE) && (SlotB == IDLE)) {
		memcpy(bcodea,autoliv_buffer,Length_Barcode); // in default condition, A has the high right
	}*/
	if(position == LEFT) {
		memcpy(bcodea,autoliv_buffer,Length_Barcode);
		printf("Rx:barcodea :\r\n");
		printf("%s\r\n",bcodea);
	}
	else {
		memcpy(bcodeb,autoliv_buffer,Length_Barcode);
		printf("Rx:barcodeb :\r\n");
		printf("%s\r\n",bcodeb);
	}
	//if(!strncmp(autoliv_buffer,autoliv.bcode_part,length_bcode_part)) {
		autoliv_send(&pdi_autoliv,"BARC-OK");
		printf("BARC-OK\r\n");
		inform_result = INFORM_NO;
	//}
	//else {
	//	autoliv_send(&pdi_autoliv,"BARC-NG\r\n");
	//	printf("BARC-NG\r\n");
        //        inform_result = INFORM_BARC;		
	//}
        return inform_result;
}
/**************************************************************
**** Up: Send "0303ITAC-OK\r\n" or "ITAC-NG"***************************
**** Down: Receive this string ,take the corresponding action**
**************************************************************/
INFORM_E Step3(void)
{	
	if((autoliv_buf[7] == 'O') && (autoliv_buf[8] == 'K')) {
		printf("%s\r\n",autoliv_buf);
		printf("PASS\r\n");
		pdi_pass_action();
	}
	if((autoliv_buf[7] == 'N') && (autoliv_buf[8] == 'G')) {
		printf("%s\r\n",autoliv_buf);
		printf("FAIL\r\n");
		pdi_fail_action();
	}
	return INFORM_NO;
}
int main(void)
{
	sys_init();
	pdi_init(Function);
	/*check process*/
	while(1) {
		sys_update();	
		if((SlotA == READY) && bcodea[0] != '*' && bcodea[0] != 0 && (SlotB != COMPLETE) && (inform_result == INFORM_NO)) {
			printf("AAAA\n");
			autoliv_send(&pdi_autoliv,"UUTA-ST\r\n");
			position = LEFT;
			TestStart();
			TestBarcode();
			TestECU();
			TestStop();
			DealAfterStop(Function,3000);
			error_result = ERROR_NO;               // set default condition
			SlotA = COMPLETE;
			position = RIGHT;
			printf("##START##STATUS-100##END##\n");
			memset(bcodea,'*',sizeof(bcodea));
			printf("%s\n",bcodea);
		}
		if((SlotB == READY) && bcodeb[0] != '*' && bcodeb[0] != 0 && (SlotA != COMPLETE) && (inform_result == INFORM_NO)) {
			printf("BBBB\n");
			autoliv_send(&pdi_autoliv,"UUTB-ST\r\n");
			position = RIGHT;
			TestStart();
			TestBarcode();
			TestECU();
			TestStop();
			DealAfterStop(Function,3000);
			error_result = ERROR_NO;               // set default condition
			SlotB = COMPLETE;
			position = LEFT;
			printf("##START##STATUS-100##END##\n");
			memset(bcodeb,'*',sizeof(bcodeb));
		}				
	}
}

#if 1
static int cmd_ba_func(int argc, char *argv[])
{
	int num_fault, i;

	const char *usage = {
		"ba , usage:\n"
		"ba fault		read the error code\n"
		"ba clear       clear the history error\n"
		"ba lock x      lock the fixture x\n"
		"ba unlock x    unlock the fixture x\n"
		"ba get position  \r\n"
		"ba set position x  left or right\r\n"
	};

	if (argc < 2) {
		printf("%s", usage);
		return 0;
	}

	if (argc == 2) {
		if(argv[1][0] == 'f') {
			pdi_IGN_on();
			pdi_can();
			pdi_set_relays(autoliv);
			pdi_mdelay(1000);
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			}
			pdi_startsession();
			if (ba_GetFault(ba_fault_buf, &num_fault))
				printf("##ERROR##\n");
			else {
				printf("##OK##\n");
				printf("num of fault is: %d\n", num_fault);
				for (i = 0; i < num_fault*2; i += 2)
					printf("0x%02x,0x%02x\n", ba_fault_buf[i]&0xff,
					ba_fault_buf[i+1]&0xff);
			}
			pdi_IGN_off();
		}
		if(!strcmp(argv[1],"clear")) {
			if(position == LEFT) {
				printf("Current position is Left\r\n");
			}
			else {
				printf("Current position is Right\r\n");
			}
			pdi_can();
			pdi_IGN_on();
			pdi_mdelay(1000);
			if(pdi_clear_dtc()) {
				printf("##ERROR##\n");
			}
			else {
				printf("##OK##\n");
			}
		}
	}
	if(argc == 3) {
		if(!strcmp(argv[1],"unlock")) {
			if(!strcmp(argv[2],"1")){
				position = LEFT;
			}
			if(!strcmp(argv[2],"2")){
				position = RIGHT;
			}	
			pdi_unlock();
		}
		if(!strcmp(argv[1],"lock")) {
			if(!strcmp(argv[2],"1")){
				position = LEFT;
			}
			if(!strcmp(argv[2],"2")){
				position = RIGHT;
			}
			pdi_lock();
		}
		if(!strcmp(argv[2],"position")) {
			if(position == LEFT) {
				printf("Current position is Left\r\n");
			}
			else {
				printf("Current position is Right\r\n");
			}
		}
	}
	if(argc == 4) {
		if(!strcmp(argv[2],"position")) {
			if(!strcmp(argv[3],"left")) {
				position = LEFT;
			}
			if(!strcmp(argv[3],"right")) {
				position = RIGHT;
			}
		}
	}
	return 0;
}

const cmd_t cmd_ba = {"ba", cmd_ba_func, "ba cmd i/f"};
DECLARE_SHELL_CMD(cmd_ba)
#endif