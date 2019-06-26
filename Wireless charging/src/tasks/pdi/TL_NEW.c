/*
 *David peng.guo@2011 initial version
 *jiamao.gu@2013 5.29 change
 *the length of tlrcode must larger one byte than the SN: this one time, Max3232 5V is forget, 12V is forget too
 *SN is the tlrcode!!!!
 *add filter in usdt.c 
 *add gpio config in drv.c and usdt.c like CONFIG_PDI_tl_gujiamao 
 *Read error code: buffer must clear before reading and if read 0 0 likewise, break the function
 *in limit file , error expected must captical, even if display in little mode
 *jiamao.gu@2013 9.16 change
 *rewrite the function of check error
 *rewrite the function of set relay 
 *add the communication with Xuce Company's Up Computer(Step0,Step1,Step2,Step3)
 *add the Ping Pang Operation for the Autoliv
 *optimize the barcode length, need change once ahead not as usual
 *no need to read the barcode by MCU, directly get by Up computer, so the uart2 is used for kline
 *jiamao.gu@2013 9.25
 *add counter++ and some notes
 */

#include <string.h>
#include <stdio.h>
#include "config.h"
#include "sys/task.h"
#include "ulp/sys.h"
#include "ulp_time.h"
#include "can.h"
#include "drv.h"
#include "priv/usdt.h"
#include "ulp/debug.h"
#include "shell/cmd.h"
#include "common/debounce.h"
#include "pdi.h"
#include "mbi5025.h"

config_t autoliv;	
#define PDI_DEBUG		1        // Test dubug by putty
#define Length_barcode 13        // TBD for different Product
#define Length_SN      18		 // TBD for different Product
#define ID 0x752

static const autoliv_t pdi_autoliv = {
		.bus = &uart3,
		.data_len = 200,              // Xuce Up computer config file max length < 200, I think
		.dead_time = 20
};
static const can_bus_t* pdi_can_bus = &can1;
//pdi tl can msg
static const can_msg_t tl_clrdtc_msg =		{ID, 8, {0x04, 0x14, 0xFF, 0xFF, 0xFF, 0, 0, 0}, 0};
//static const can_msg_t tl_resis =			{0x607, 8, {0x03, 0x22, 0xfe, 0x80, 0, 0, 0, 0}, 0};
#if PDI_DEBUG
static const can_msg_t tl_errcode_msg =		{ID, 8, {0x03, 0x19, 0xfd, 0x39, 0, 0, 0, 0}, 0};
static const can_msg_t tl_getsn_msg =			{ID, 8, {0x03, 0x22, 0xfe, 0x8d, 0, 0, 0, 0}, 0};
static const can_msg_t tl_part_msg =			{0x7D2, 8, {0x03, 0x22, 0xF1, 0x00, 0x55, 0x55, 0x55, 0x55}, 0};
#endif
static const can_msg_t tl_reqseed_msg =		{ID, 8, {0x02, 0x27, 0x61, 0, 0, 0, 0, 0}, 0};
static const can_msg_t tl_start_msg =			{ID, 8, {0x02, 0x10, 0x60, 0, 0, 0, 0, 0}, 0};
static can_msg_t tl_msg_buf[32];		// for multi frame buffer
static char tl_data_buf[256];			// data buffer
static char tl_fault_buf[64];			// fault buffer
static char bcodea[Length_barcode + 1];	// for A LEFT, + 1 for '\0'
static char bcodeb[Length_barcode + 1];	// for B RIGHT, + 1 for '\0'
static char SN[Length_SN + 1];
static char his_a = 0;
static char his_b = 0;

static char autoliv_buf[512];          	// for Xuce Up computer buffer					// for Xuce config file
FUNCTION_T Function[10];  				// max 10 functions address
ERROR_E error_result = ERROR_NO;		// error happened during test product
POSITION_E position = LEFT; 			// left position in default condition, only change after test
FIXTURE_E SlotA = IDLE, SlotB = IDLE;   // for Ping Pang Operation

// get data from ECU
static int tl_GetCID(char sid,short cid, char *data);
static int tl_GetFault(char *data, int *pnum_fault);

static DEBUG_OR_PRODUCT_E debug_or_product = PRODUCT; //for new protocol add by hyf 2014/5/22
// communicate with up computer, drive function 
void Step0(void);
void Step1(void);
void Step2(void);
void Step3(void);   // for receiving ITAC-OK or ITAC-NG
void Step4(void);   // for SN check
void Step6(void);   //for mode change
// communicate with up computer, app function
void DealAfterReceive(FUNCTION_T *FUNCITON); // Step0 Step1 Step2 step4
void DealAfterStop(FUNCTION_T *FUNCITON, time_t deadline);// Step3 
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
	can_msg_t msg;
	can_msg_t sendkey_msg = {
		ID,
		8,
		{0x04, 0x27, 0x62, 0xff, 0xff, 0, 0, 0},
		0
	}; // TBD By Me
	int try_time_start = 5;
	do {
		usdt_GetDiagFirstFrame(&tl_start_msg, 1, NULL, &msg, &msg_len);		//start session, why twice?	
		try_time_start --;
	}while((msg.data[1]!= 0x50) && (try_time_start != 0)); 
	if(try_time_start == 0) {
		return 1;
	}
#if PDI_DEBUG
	printf("111111\r\n");
	can_msg_print(&tl_start_msg,"\n");
	can_msg_print(&msg, "\n");
#endif
	int try_time_reqseed = 5;
	do {
		usdt_GetDiagFirstFrame(&tl_reqseed_msg, 1, NULL, &msg, &msg_len);		//req seed
		try_time_reqseed --;
	}while((msg.data[1]!= 0x67) && (try_time_reqseed != 0));
	if(try_time_reqseed == 0) {
		return 1;
	}	
#if PDI_DEBUG
	printf("222222\r\n");
	can_msg_print(&tl_reqseed_msg,"\n");
	can_msg_print(&msg, "\n");
#endif
#if 0
	unsigned char seed[3], result[3];
	unsigned char accessLevel = (unsigned char)msg.data[2];
	/*calculate the key from seed*/
	seed[0] = (unsigned char)msg.data[3];
	seed[1] = (unsigned char)msg.data[4];
	seed[2] = (unsigned char)msg.data[5];
	CalculateKey(accessLevel,seed,result);
	sendkey_msg.data[3] = result[0];
	sendkey_msg.data[4] = result[1];
	sendkey_msg.data[5] = result[2];
#endif
	unsigned char seed[2];
	unsigned char result;
	seed[0] = (unsigned char)msg.data[3];
	seed[1] = (unsigned char)msg.data[4];
	result = seed[0] ^ seed[1];
	result ^= 0x34;
	result ^= 0xab;
	sendkey_msg.data[3] = (char)((0x34ab + result) >> 8);
	sendkey_msg.data[4] = (char)((0x34ab + result) & 0x00ff);
	/*send key*/
	int try_time_key = 5;
	do {
		usdt_GetDiagFirstFrame(&sendkey_msg, 1, NULL, &msg, &msg_len);
		try_time_key --;
	}while(msg.data[1] != 0x67 && (try_time_key != 0));
	if(try_time_key == 0) {
		return 1;
	}
#if PDI_DEBUG
	printf("333333\r\n");
	can_msg_print(&sendkey_msg, "\n");
	can_msg_print(&msg, "\n");
#endif
	/*judge the send key response*/
	if ((msg.data[1] != (sendkey_msg.data[1] + 0x40)) || (msg.data[2] != sendkey_msg.data[2]))//sendkey_msg[1] + 0x40 and sendkey_msg[2]
		return 1;
#if 0
#if PDI_DEBUG
	/*get serial number*/
	printf("\nSN Code:\n");
	usdt_GetDiagFirstFrame(&tl_getsn_msg, 1, NULL, tl_msg_buf, &msg_len);
	if (msg_len > 1)
		usdt_GetDiagLeftFrame(tl_msg_buf, msg_len);
	for (i = 0; i < msg_len; i ++)
		can_msg_print(tl_msg_buf + i, "\n");

	/*get error code*/
	printf("\nError Code:\n");
	usdt_GetDiagFirstFrame(&tl_errcode_msg, 1, NULL, tl_msg_buf, &msg_len);
	if (msg_len > 1)
		usdt_GetDiagLeftFrame(tl_msg_buf, msg_len);
	for (i = 0; i < msg_len; i++)
		can_msg_print(tl_msg_buf + i, "\n");
	/*tester point*/
	printf("\nPart:\n");
	usdt_GetDiagFirstFrame(&tl_getpart_msg, 1, NULL, tl_msg_buf, &msg_len);
	if (msg_len > 1)
		usdt_GetDiagLeftFrame(tl_msg_buf, msg_len);
	for (i = 0; i < msg_len; i++)
		can_msg_print(tl_msg_buf + i, "\n");

	if (tl_GetFault(tl_fault_buf, &num_fault))
		printf("##ERROR##\n");
	else {
		printf("##OK##\n");
		printf("num of fault is: %d\n", num_fault);
		for (i = 0; i < num_fault*2; i += 2)
			printf("0x%02x, 0x%02x\n", tl_fault_buf[i]&0xff, tl_fault_buf[i+1]&0xff);
	}
	/*clear all*/
	printf("\nClear all:\n");
	if (usdt_GetDiagFirstFrame(&tl_clrdtc_msg, 1, NULL, &msg, &msg_len))	//req seed
		return 1;
	can_msg_print(&msg, "\n");
#endif
#endif

	return 0;
}
//
//whether some bytes in range 
//
static int tl_esc_check()
{
	int a = 0, b = 1;
	unsigned short v1 = 0, v2 = 0, v3 = 0;
	can_msg_t esc_msg;
	time_t deadtime = time_get(100);
	printf("##START##EC-Checking ESC...##END##\n");
	while(time_left(deadtime) > 0) {            
		pdi_can_bus -> recv(&esc_msg);
		switch(esc_msg.id) {
		case 0x245:
			// Update by jiamao.gu 9.29 in Shanghai
			if(a == 1) break;
			v1 = esc_msg.data[0] << 8 | esc_msg.data[1];
			//printf("LongitudeAcc is %x\n", v1);
			if((v1 > 0x582d) || (v1 < 0x53f4)) {
				return 1;
			}
			v2 = esc_msg.data[2] << 8 | esc_msg.data[3];
			//printf("LateralAcce is %x\n", v2);
			if((v2 > 0x582d) || (v2 < 0x53f4)) {
				return 1;
			}
			v3 = esc_msg.data[4] << 8 | esc_msg.data[5];
			//printf("VehDynYawRate is %x\n", v3);
			if((v3 > 0x2253) || (v3 < 0x21d0)) {
				return 1;
			}
			/*esc_msg.data[2] &= 0xf0; //compare the low 4 bits, must clear high 4 bits
			if(esc_msg.data[2] != 0x00) {
				printf("Sensor status error\r\n");
				return 1;
			}
			else {
				printf("Sensor status ok\r\n");
			}
			
			v1 = esc_msg.data[1];  //data[1] << 8 | data[0]
			v1 <<= 8;
			v1 |= esc_msg.data[0];
			if(v1 > 0x3334) {
				printf("Yaw rate error\r\n");
				return 1;
			}
			else {
				printf("Yaw rate ok\r\n");
			}
			
			v2 = esc_msg.data[5];  //data[5] << 8 | data[4]
			v2 <<= 8;
			v2 |= esc_msg.data[4];
			if(v2 > 0x3662) {
				printf("Lateral acceleation fail\r\n");
				return 1;
			}
			else {
				printf("Lateral acceleation ok\r\n");
			}	*/		
			a = 1;
			break;
		case 0x95:
			//update by jiamao.gu 9.29 in shanghai
			if(b == 1) break;
			v3 = esc_msg.data[3];   // data[3] << 8 | data[2]
			v3 <<= 8;
			v3 |= esc_msg.data[2];
			if(v3 > 0x3662) {
				printf("Longitudinal acceleation fail\r\n");
				return 1;
			}
			else {
				printf("Longitudinal acceleation ok\r\n");
			}
			b = 1;
			break;
        default :
            break;
		}
		if((a == 1)&&(b == 1)) return 0;
		else continue;
	}
	return 0;
}
/*get contents of cid,saved to data*/
static int tl_GetCID(char sid,short cid, char *data)
{
	can_msg_t msg_res, pdi_send_msg = {ID, 8, {0x03, 0xff, 0xff, 0xff, 0, 0, 0, 0}, 0};
	int i = 0, msg_len;
	pdi_send_msg.data[1] = sid;
	pdi_send_msg.data[2] = (char)(cid >> 8);
	pdi_send_msg.data[3] = (char)(cid & 0x00ff);	
	if (usdt_GetDiagFirstFrame(&pdi_send_msg, 1, NULL, &msg_res, &msg_len))
		return 1;
	printf("444444\r\n");
	can_msg_print(&pdi_send_msg,"\n");
	can_msg_print(&msg_res, "\n");
	if (msg_len > 1) {
		tl_msg_buf[0] = msg_res;
		if(usdt_GetDiagLeftFrame(tl_msg_buf, msg_len))
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
			memcpy(data, (tl_msg_buf + i)->data + 1, 7);
			data += 7;
		}
	}
	return 0;
}
/* read barcode*/
// return 1 : no date
// retrun 0 : ok
static int tl_read_barcode() 
{
	char read_bcode[Length_barcode + 1];
	int try_times = 5;
	while (tl_GetCID(0x22,0xf18c, tl_data_buf)) {
		try_times --;
		if (try_times < 0)
			return 1;
	}
	memcpy(read_bcode, tl_data_buf, Length_barcode);
	printf(read_bcode,'\0'); // add '\0' in the end
	return 0;
}
/* checking tlrcode */
// return 1 : no data; not equal
// return 0 : ok
static int tl_check_barcode()
{
	char read_bcode[Length_barcode + 1];
	int try_times = 5;
	printf("##START##EC-Checking barcode...##END##\n");
	while (tl_GetCID(0x22,0xf18c, tl_data_buf)) {
		try_times --;
		if (try_times < 0)
			return 1;
	}
	memcpy(read_bcode, tl_data_buf, Length_barcode);
	printf("##START##RB-");
	printf(read_bcode,'\0');
	printf("##END##\n");
	if(position == LEFT) {
		if (memcmp(read_bcode, bcodea, Length_barcode))  //maybe Length_tlrcode - 1 ??
			return 1;
	}
	else {
		if (memcmp(read_bcode, bcodeb, Length_barcode))
			return 1;
	}
	return 0;
}
/* read Serial Number */
//return 1 : no date
//return 0 : ok
static int tl_read_sn()
{
	char read_sn[Length_SN + 1];
	int try_times = 5;
	memset(tl_data_buf,0,sizeof(tl_data_buf));
	while(tl_GetCID(0x22,0xff8c,tl_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	tl_data_buf[Length_SN] = '\0';
	memcpy(read_sn,tl_data_buf,sizeof(read_sn));
	printf("SN:%s\r\n",read_sn);
	return 0;
}
/* equal Serial Number */
//return 1 : not equal
//return 0 : ok
static int tl_check_sn()
{
	char read_sn[Length_SN + 1];
	int try_times = 5;
	memset(tl_data_buf,0,sizeof(tl_data_buf));
	while(tl_GetCID(0x22,0xff8c,tl_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	tl_data_buf[Length_SN] = '\0';
	memcpy(read_sn,tl_data_buf,sizeof(read_sn));
	printf("SN Read from ECU : \r\n");
	printf("%s\r\n",read_sn);
	printf("Compare the SN :");
	if(memcmp(SN, read_sn, Length_SN))
		return 1;
	return 0;
}
/* read CPN */
//return 1 : no date
//return 0 : ok
static int tl_read_cpn()
{
	char read_cpn[length_cpn + 1];
	int try_times = 5;
	memset(tl_data_buf,0,sizeof(tl_data_buf));
	while(tl_GetCID(0x22,0xf187,tl_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	tl_data_buf[length_cpn] = '\0';
	memcpy(read_cpn, tl_data_buf, sizeof(read_cpn));
	printf("CPN:%s\r\n", read_cpn);
	return 0;
}
/* equal Serial Number */
//return 1 : not equal
//return 0 : ok
static int tl_check_cpn()
{
	char read_cpn[length_cpn + 1];
	int try_times = 5;
	memset(tl_data_buf, 0, sizeof(tl_data_buf));
	while(tl_GetCID(0x22,0xf187,tl_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	tl_data_buf[length_cpn] = '\0';
	memcpy(read_cpn, tl_data_buf, sizeof(read_cpn));
	printf("CPN Read from ECU : \r\n");
	printf("%s\r\n",read_cpn);
	printf("Compare the CPN :");
	if(memcmp(autoliv.cpn, read_cpn, length_cpn))
		return 1;
	return 0;
}
static int tl_read_softversion()
{
	char read_softversion[11 + 1];  // 12 means length
	int try_times = 5;
	memset(tl_data_buf,0,sizeof(tl_data_buf));
	while(tl_GetCID(0x22,0xfda5,tl_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	tl_data_buf[11] = '\0';
	memcpy(read_softversion,tl_data_buf,sizeof(read_softversion));
	printf("%s\r\n",read_softversion);
	return 0;
}
static int tl_check_softversion()
{
	char read_softversion[11 + 1];  // 12 means length
	int try_times = 5;
	memset(tl_data_buf,0,sizeof(tl_data_buf));
	while(tl_GetCID(0x22,0xfda5,tl_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	tl_data_buf[11] = '\0';
	memcpy(read_softversion,tl_data_buf,sizeof(read_softversion));
	printf("Read softversion from ECU : %s\r\n",read_softversion);
	printf("Read softversion from Up : %s\r\n",autoliv.softversion);
	if(memcmp(autoliv.softversion,read_softversion,11))
		return 1;
	return 0;
}
/*checking Resis*/
/*static int tl_check_resis()
{
	int msg_len;
	can_msg_t msg;
	printf("##START##EC-Checking resis...##END##\n");
	if (usdt_GetDiagFirstFrame(&tl_resis, 1, NULL, &msg, &msg_len))
		return 1;
	if (msg_len > 1)
	usdt_GetDiagLeftFrame(tl_msg_buf, msg_len);
	for (int i = 0; i < msg_len; i ++)
		can_msg_print(tl_msg_buf + i, "\n");


	return 0;
}*/

/*clear error code*/
int pdi_clear_dtc(void)
{
	int msg_len;
	can_msg_t msg;
	pdi_startsession();
	if (usdt_GetDiagFirstFrame(&tl_clrdtc_msg, 1, NULL, &msg, &msg_len))
		return 1;
	if (msg.data[1] != (tl_clrdtc_msg.data[1] + 0x40))	// TBD By Me
		return 1;
	return 0;
}

/*get fault from ECU,saved to data,the nr of fault is pnum_fault*/
static int tl_GetFault(char *data, int *pnum_fault)
{
	int i, result = 0;
	if (tl_GetCID(0x22,0xfd80, data))
		return 1;
	//memset(data + 117, 0x00, 10);
	for (i = 0; i < 117; i += 3) {
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
	FUNCTION[0] = Step3;    // as 0303ITAC- head
	FUNCTION[4] = Step4;	// as 0404 head
	FUNCTION[6] = Step6;	// as 0606 head
}
/*********************************************
** add a debounce function to filter the error
** get zero 10 times means target is ok ******
** after test, the status is COMPLETE ********
**********************************************/
void Status_transform(void)
{
	if((!target1_on()) && (SlotA == IDLE)) {  // on + IDLE --> READY
		SlotA = READY;
	}
	if((!target2_on()) && (SlotB == IDLE)) {
		SlotB = READY;
	}
	//2013.9.25, if target on, counter++
	//pass means left position, fail means right position
	if(!target1_on() && (his_a == 0)) {
		his_a = 1;
		//counter_pass_add();
	}
	if(target1_on()) {
		SlotA = IDLE;  // add 10.9 in Suzhou
		his_a = 0;
	}
	if(!target2_on() && (his_b == 0)) {
		his_b = 1;
		//counter_fail_add();
	}
	if(target2_on()) {
		SlotB = IDLE; // add 10.9 in Suzhou
		his_b = 0;
	}
	// maybe no need 
	if(target1_on() && target2_on()) {         // default condition : IDLE
		SlotA = IDLE;
		SlotB = IDLE;
        }
	// maybe no need 
	if(target1_on() && (SlotA == COMPLETE)) {  // off + COMPLETE --> IDLE
		SlotA = IDLE;
	}
	// maybe no need 
	if(target2_on() && (SlotB == COMPLETE)) {	// off + COMPLETE --> IDLE
		SlotB = IDLE;
	}
	/*
	if(target1_on()) {
		if(position == LEFT) {
			error_result = ERROR_CW;
		}
		SlotA = IDLE;  // add 10.9 in Suzhou
		his_a = 0;
	}
	if(target2_on()) {
		if(position == RIGHT) {
			error_result = ERROR_CW;
		}
		SlotB = IDLE; // add 10.9 in Suzhou
		his_b = 0;
	}
	*/
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
/****************************************
** set the can/kline ********************
** pick up the tlrcode ****************** 
** set the relay according to relay file*
** return error information *************
*****************************************/
ERROR_E TestStart(void)
{
	if(position == LEFT) {
		printf("Product A will be tested\n");
		autoliv_send(&pdi_autoliv,"UUTA-ST\r\n"); // for communicate with the upper computer
	}
	else {
		printf("Product B will be tested\n");
		autoliv_send(&pdi_autoliv,"UUTB-ST\r\n");
	}
	pdi_can();								// set the can relay
	pdi_mdelay(50);
	pdi_set_relays(autoliv);              	// set the load relays, may exist bugs
	pdi_mdelay(50);
	pdi_IGN_on();                         	// set the power relay or switch	
	return ERROR_NO;
}
/*******************************************
** Blink the yellow led ********************
** compare tlrcode from ECU and scaner  **
** must send the startsession before read **
** process bar running *********************
*/
ERROR_E Testbarcode(void)
{
	if(error_result != ERROR_NO) {
		return error_result;
	}
	char processbar[2];
	int rate;
	start_action();
	/*delay 1s*/
	for (rate = 5; rate <= 17; rate ++) {
		pdi_mdelay(89);                 // in the delay, pdi_update will be called   
		printf("##START##STATUS-");		// in such format, so the data can be displayed in processtlr
		sprintf(processbar, "%d", rate);
		printf("%s", processbar);
		printf("##END##\n");
	}
	pdi_startsession();                 // startsession before getting message
	//if (tl_check_barcode()) {
	//	error_result = ERROR_BW;
	//	return error_result;
	//}
	printf("####EC-      Checking tlrcode Done...##END##\n");
	return ERROR_NO;
}
ERROR_E TestSN(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (tl_check_sn()) {
		error_result = ERROR_SN;
		return error_result;
	}
	printf("####EC-      Checking SN Done...##END##\n");
	printf("Clear the SN buffer for next test !\r\n");
	memset(SN,0,sizeof(SN));
	return ERROR_NO;
}
ERROR_E TestPSN(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (tl_check_psn()) {
		error_result = ERROR_PSN;
		return error_result;
	}
	printf("####EC-      Checking PSN Done...##END##\n");
	printf("Clear the PSN buffer for next test !\r\n");
	//memset(SN,0,sizeof(SN));
	return ERROR_NO;
}
ERROR_E TestIMU(void)
{
	if((error_result != ERROR_NO) && (error_result != ERROR_ENCODE)){
		return error_result;
	}
	if(autoliv.IMU == '1') {
		printf("Test IMU Can begin \r\n");
		if(!tl_esc_check()) {
			error_result = ERROR_NO;
		}
		else {
			error_result = ERROR_IMU;
		}
	}
	else {
		printf("No IMU Can \r\n");
		error_result = ERROR_NO;
	}	
	return error_result;
}
/**********************************************
** process tlr running ************************
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
	memset(tl_fault_buf,0,sizeof(tl_fault_buf)); // clear this buffer, if not, you will see the bug
	int try_times = 0;
	int result = 0;
	int num_fault = 0;
	/**** when result is 1, must get fault again, oversize 3 times *****
	**** num_fault is not 0, check the error_code ********************** 
	*******************************************************************/
	pdi_startsession();
	do{
		result = tl_GetFault(tl_fault_buf,&num_fault);
		if(++try_times == 4) {
			error_result = ERROR_CW;
			return error_result;
		}
	} while(result == 1);
	if(!num_fault) {
		error_result = ERROR_NO;
		return error_result;
	}
	else {
		printf("##START##EC-");
		printf("num of fault is: %d\n", num_fault);
		for (int i = 0; i < num_fault * 3; i += 3)  // according to size of error code , sometimes 3 bytes 
			printf("0x%02x, 0x%02x, 0x%02x\n", tl_fault_buf[i] & 0xff, tl_fault_buf[i + 1] & 0xff, tl_fault_buf[i + 2] & 0xff);
		printf("##END##\n");
		if(pdi_error_check(tl_fault_buf, &num_fault, autoliv)) { // check whether the error code is allowable
			error_result = ERROR_CODE;
			autoliv_send(&pdi_autoliv,"ERROR\r\n ");  //new protocol add by hyf at shanghai 2014/5/15  ' 'for the upper's convinence
				for(int i = 0; i < num_fault * 3; i++){   //send errorcode to up
					//autoliv_send_char(&pdi_autoliv, tl_fault_buf[i]);
					char a , b;
					a = tl_fault_buf[i] >> 4;
					b = tl_fault_buf[i] & 0x0f;
					if((a >= 0) && (a <= 9)) {
						a += '0';
					} else if((a >= 10) && (a <= 15)) {
						a = a - 10 + 'a';
					}
					if((b >= 0) && (b <= 9)) {
						b += '0';
					} else if((b >= 10) && (b <= 15)) {
						b = b - 10 + 'a';
					}
					autoliv_send_char(&pdi_autoliv, a);
					autoliv_send_char(&pdi_autoliv, b);
				}
				autoliv_send(&pdi_autoliv,"\r\n");
				if(debug_or_product == DEBUG){
				//must have,or cannot clear the historical error 
					for (int rate = 17; rate <= 96; rate ++) {
						pdi_mdelay(89);
					}
					pdi_IGN_on();
					pdi_can();
					pdi_clear_dtc();
				}
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
	pdi_IGN_off();
	int msg_len;
	can_msg_t msg;	
	do {                                // if no extern CAN
		pdi_mdelay(1000);
	} while(!usdt_GetDiagFirstFrame(&tl_start_msg, 1, NULL, &msg, &msg_len));
}
/******************************
** change the status **********
** unlock the fixture *********
** display the led ************
******************************/
void TestStop(void)
{
	//if(debug_or_product == PRODUCT)
	{
		Poweroff();
		if(position == LEFT) {
			switch(error_result) {
				case ERROR_NO:
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-OK");
						autoliv_send(&pdi_autoliv,"0\r\n");
						printf("UUTA-FUNC-OK\r\n");
						break;
				case ERROR_NRP:                                  // not happened forever
						printf("##START##EC-Target A is not on the right position##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
						autoliv_send(&pdi_autoliv,"\r\n");
						printf("UUTA-FUNC-NG\r\n");
						break;
				case ERROR_NCF:                                  // not happened forever
						printf("##START##EC-No This Config File A ##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
						autoliv_send(&pdi_autoliv,"\r\n");
						printf("UUTA-FUNC-NG\r\n");
						break;
				case ERROR_BW:
						printf("##START##EC-barcode A Wrong##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
						autoliv_send_char(&pdi_autoliv, 10);
						autoliv_send(&pdi_autoliv,"\r\n");
						printf("UUTA-FUNC-NG10\r\n");
						break;
				case ERROR_SN:
						printf("##START##EC-SN A Wrong##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
						printf("UUTA-FUNC-NG12\r\n");
						autoliv_send_char(&pdi_autoliv, 12);
						autoliv_send(&pdi_autoliv,"\r\n");
						break;
				case ERROR_SV:
						printf("##START##EC-SV A Wrong##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
						printf("UUTA-FUNC-NG8\r\n");
						autoliv_send_char(&pdi_autoliv, 8);
						autoliv_send(&pdi_autoliv,"\r\n");
						break;
				case ERROR_CW:
						printf("##START##STATUS-Get A CW error!##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
						autoliv_send_char(&pdi_autoliv,1);
						autoliv_send(&pdi_autoliv,"\r\n");
						printf("UUTA-FUNC-NG1\r\n");
						break;
				case ERROR_CODE:
						printf("##START##STATUS-Get A DTC error!##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
						autoliv_send_char(&pdi_autoliv,6);
						autoliv_send(&pdi_autoliv,"\r\n");
						printf("UUTA-FUNC-NG6\r\n");
						break;
				case ERROR_IMU:
						printf("##START##STATUS-Get A IMU error!##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
						autoliv_send_char(&pdi_autoliv, 16);
						autoliv_send(&pdi_autoliv,"\r\n");
						printf("UUTA-FUNC-NG16\r\n");
						break;
				case ERROR_ENCODE:
						printf("##START##EC-These faults are acceptale...##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-OK0\r\n");
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
							autoliv_send(&pdi_autoliv,"UUTB-FUNC-OK");
							autoliv_send(&pdi_autoliv,"0\r\n");
							break;
					case ERROR_NRP:
							printf("##START##EC-Target B is not on the right position##END##\n");
							printf("UUTB-FUNC-NG\r\n");
							autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
							autoliv_send(&pdi_autoliv,"\r\n");
							break;
					case ERROR_NCF:
							printf("##START##EC-No This Config File B##END##\n");
							printf("UUTB-FUNC-NG\r\n");
							autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
							autoliv_send(&pdi_autoliv,"\r\n");
							break;
					case ERROR_BW:
							printf("##START##EC-barcode B Wrong##END##\n");
							printf("UUTB-FUNC-NG11\r\n");
							autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
							autoliv_send_char(&pdi_autoliv, 11);
							autoliv_send(&pdi_autoliv,"\r\n");
							break;
					case ERROR_SN:
							printf("##START##EC-SN B Wrong##END##\n");
							autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
							autoliv_send_char(&pdi_autoliv, 13);
							autoliv_send(&pdi_autoliv,"\r\n");
							printf("UUTB-FUNC-NG13\r\n");
							break;
					case ERROR_SV:
							printf("##START##EC-SV B Wrong##END##\n");
							autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
							autoliv_send_char(&pdi_autoliv, 9);
							autoliv_send(&pdi_autoliv,"\r\n");
							printf("UUTB-FUNC-NG9\r\n");
							break;
					case ERROR_CW:
							printf("##START##STATUS-Get B CW error!##END##\n");
							printf("UUTB-FUNC-NG1\r\n");
							autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
							autoliv_send_char(&pdi_autoliv,1);
							autoliv_send(&pdi_autoliv,"\r\n");
							break;
					case ERROR_IMU:
							printf("##START##STATUS-Get B IMU error!##END##\n");
							autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
							autoliv_send_char(&pdi_autoliv,6);
							autoliv_send(&pdi_autoliv,"\r\n");
							printf("UUTB-FUNC-NG6\r\n");
							break;
					case ERROR_CODE:
							printf("##START##STATUS-Get B DTC error!##END##\n");
							printf("UUTB-FUNC-NG7\r\n");				
							autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
							autoliv_send_char(&pdi_autoliv,7);
							autoliv_send(&pdi_autoliv,"\r\n");
							break;
					case ERROR_ENCODE:
							printf("##START##EC-These faults are acceptale...##END##\n");
							printf("UUTB-FUNC-OK\r\n");
							autoliv_send(&pdi_autoliv,"UUTB-FUNC-OK0\r\n");			
							break;
					default:
							break ;
			}
		}
	}
}
/***************************************
************* in sys_update ************
****************************************/
void DealAfterReceive(FUNCTION_T *FUNCTION)
{
	memset(autoliv_buf,0,sizeof(autoliv_buf));   // Global variable attention!!!
	if(!autoliv_get(&pdi_autoliv,autoliv_buf)) { // All the receive data put in autoliv_buf
		if(autoliv_buf[0] > 0x00 && autoliv_buf[0] < 0x09)
			FUNCTION[autoliv_buf[0]]();                  // Deal with the data
		else 
			printf("Frame Head Error\r\n");
	}
}
/*****************************************************
**only use in upload to itac, so no need in sys_update
******************************************************/
void DealAfterStop(FUNCTION_T *FUNCTION, time_t dead_time)
{
	memset(autoliv_buf,0,sizeof(autoliv_buf));   // Glotll variable attention!!!
	time_t over_time;
	over_time = time_get(dead_time);
	while(time_left(over_time) > 0) {
		if(!autoliv_get(&pdi_autoliv,autoliv_buf)) { // All the receive data put in autoliv_buf
			if (autoliv_buf[2] == 'I') {			 // In the first function 
				FUNCTION[0]();                       // Receiving the "0303ITAC-OK or NG\r\n", retrun
				return ;
			}
		}
	}
	printf("ITAC Overtime \r\n");
	// Although Overtime, still operate the led
	if(error_result == ERROR_NO || error_result == ERROR_ENCODE) {
		pdi_pass_action();
	}
	else {
		pdi_fail_action();
	}
}
/*****************************************
*** Up:Send 0303ECUPDIID\r\n *************
*** Down:Send ECUPDIIDLTXXX\r\n ***********
******************************************/
void Step0(void)
{
	autoliv_send(&pdi_autoliv,"ECUPDIIDLT009\r\n");  // Send to Up
	printf("%s\r\n","ECUPDILT009\r\n");
}
/***********************************************************************************
*** Up: Send 01012f+part_code+relay+length_softversion+softversion+IMU+error+\r\n***
*** Down: Send "DL-CONF-OK" ********************************************************
***********************************************************************************/
void Step1(void)
{
	printf("****************\r\n");
	for(int i = 0; i < length_bcode_part; i++) {
		autoliv.bcode_part[i] = autoliv_buf[i + 3];   // Jump out 03030f
	}
	printf("Rx:part barcode:\r\n");
	printf("%s\r\n",autoliv.bcode_part);
	printf("Rx:relay:\r\n");
	for(int i = 0; i < length_relay; i++) {
		autoliv.relay[i] = autoliv_buf[i + 3 + length_bcode_part]; // Jump out 01010f + bcode_part
		printf("%x ",autoliv.relay[i]);
		if((i + 1) % 6 == 0) { // one line 6 bytes
			printf("\r\n");
		}
	}
	//softversion length
	//added by jiamao.gu in Shanghai 2013.10.29
	//example 3b - '0' = b, means length of software is 12
	autoliv.length_softversion = autoliv_buf[3 + length_bcode_part + length_relay] - '0';
	printf("Rx:softversion length is : %d \r\n",autoliv.length_softversion);
	for(int i = 0; i < autoliv.length_softversion; i++) {
		autoliv.softversion[i] = autoliv_buf[i + 4 + length_bcode_part + length_relay]; // Jump out 01010f + bcode_part + relay
	}
	printf("Rx:softversion:\r\n");
	printf("%s\r\n",autoliv.softversion);
	autoliv.IMU = autoliv_buf[4 + length_bcode_part + length_relay + autoliv.length_softversion];
	printf("Rx:IMU:\r\n");
	printf("%x\r\n",autoliv.IMU);
	autoliv.JAMA = autoliv_buf[5 + length_bcode_part + length_relay + autoliv.length_softversion];
	printf("Rx:JAMA:\r\n");
	printf("%x\r\n",autoliv.JAMA);
	// add at 2014/5/14  length_errorcode
	autoliv.length_errorcode = autoliv_buf[6 + length_bcode_part + length_relay + autoliv.length_softversion ] - '0'; 
	printf("Rx:errorcode length is %d\r\n",autoliv.length_errorcode); 
	printf("Rx:errorable:\r\n");
	// add length_errorcode
	for(int i = 0; i < autoliv.length_errorcode; i++) {
		autoliv.errorable[i] = autoliv_buf[i + 5 + length_bcode_part + length_relay + autoliv.length_softversion + 1 + 1 ];
		printf("%x",autoliv.errorable[i]);
		if((i + 1) % 6 == 0) {
			printf("\r\n");
		}
	}
	int i = 0;
	while((autoliv_buf[i + 5 + length_bcode_part + length_relay + autoliv.length_softversion + autoliv.length_errorcode + 1 + 1 ] != 0x0D) && (autoliv_buf[i + 5 + length_bcode_part + length_relay + autoliv.length_softversion + 2 + 1] != 0x0A)) {
		autoliv.Calibration[i] = autoliv_buf[i + 5 + length_bcode_part + length_relay + autoliv.length_softversion+ autoliv.length_errorcode + 1 + 1 ];
		i++;
	}
	autoliv_send(&pdi_autoliv,"DL-CONF-OK\r\n");
	printf("DL-CONF-OK\r\n");
}
/*************************************************************
**** Up: Send 0202+barcode+\r\n ******************************
**** Down: Send "BARC-OK\r\n" or "BARC-NG\r\n" ***************
*************************************************************/
void Step2(void)
{
	char autoliv_buffer[256];                   // Temp variable
	memset(autoliv_buffer,0,sizeof(autoliv_buffer));
	int i = 0;
	while((autoliv_buf[i + 2] != 0x0D) && (autoliv_buf[i + 3] != 0x0A)) {
		autoliv_buffer[i] = autoliv_buf[i + 2]; 
		i++;
	}
	//add by jiamao.gu in Shanghai 2013.10.28
	//if count is odd, barcode is a, else is b
	//maybe add compare in the future
	static int count = 0;
	if(count % 2 == 0) {
		memcpy(bcodea,autoliv_buffer,Length_barcode);
		printf("Rx:barcodea :\r\n");
		printf("%s\r\n",bcodea);
		autoliv_send(&pdi_autoliv,"BARC-OK\r\n");
		printf("BARC-OK\r\n");		
	}
	else {
		memcpy(bcodeb,autoliv_buffer,Length_barcode);
		printf("Rx:barcodeb :\r\n");
		printf("%s\r\n",bcodeb);
		autoliv_send(&pdi_autoliv,"BARC-OK\r\n");
		printf("BARC-OK\r\n");
	}
	count ++;
	//if(!strncmp(autoliv_buffer,autoliv.bcode_part,length_bcode_part)) {
	//}
	//else {
	//	autoliv_send(&pdi_autoliv,"BARC-NG\r\n");
	//	printf("BARC-NG\r\n");		
	//}
}
/**************************************************************
**** Up: Send "0303ITAC-OK\r\n" or "ITAC-NG"*******************
**** Down: Receive this string ,take the corresponding action**
**************************************************************/
void Step3(void)
{	
	if((autoliv_buf[7] == 'O') && (autoliv_buf[8] == 'K')) {
		printf("Rx:%s\r\n",autoliv_buf);
		printf("PASS\r\n");
		pdi_pass_action();
	}
	if((autoliv_buf[7] == 'N') && (autoliv_buf[8] == 'G')) {
		printf("Rx:%s\r\n",autoliv_buf);
		printf("FAIL\r\n");
		pdi_fail_action();
	}
}
/**************************************************************
**** Up: Send "0404SN\r\n"*******************
**** Down: Receive this string ,take the corresponding action**
**************************************************************/
void Step4(void)
{
	char autoliv_buffer[256];                   // Temp variable
	memset(autoliv_buffer,0,sizeof(autoliv_buffer));
	memset(SN,0,sizeof(SN));
	int i = 0;
	while((autoliv_buf[i + 2] != 0x0D) && (autoliv_buf[i + 3] != 0x0A)) {
		autoliv_buffer[i] = autoliv_buf[i + 2]; 
		i++;
	}
	memcpy(SN,autoliv_buffer,Length_SN);
	printf("SN Read from ITAC : \r\n");
	printf(SN,'\0');
        //add by hyf in  shanghai 2014/5/14 new protocol
        autoliv_send(&pdi_autoliv,"SN\r\n");
}

/**************************************************************
**** Up: Send "0606+mode\r\n"*******************
**** Down: Receive this string ,take the corresponding action**
**************************************************************/
//add by hyf at shanghai 2014/5/15 new protocol
void Step6(void)
{
	if((autoliv_buf[2] == 'D') ) {		
		printf("DEBUG MODE!\r\n");
		autoliv_send(&pdi_autoliv,"MODEOK\r\n");
		debug_or_product = DEBUG;
	}
	if((autoliv_buf[2] == 'P') ) {		
		printf("PRUDUCT MODE!\r\n");
		autoliv_send(&pdi_autoliv,"MODEOK\r\n");
	
	}
}
// add by jiamao.gu in Shanghai 2013.10.28
// tlrcode sended after teststop in about three seconds, so need to wait 
// after receiving barcode, SN sended in about five seconds, so need to wait 
// maybe change over_time in different condition !!!
void Receive_from_Up(time_t over_time)
{
	while(time_left(over_time) > 0) {
		if(SN[0] != 0) { // All the receive data put in autoliv_buf			
			printf("Receive other tlrcode and SN success !\r\n");
			return ;
		}
		sys_update();
	}
}
void pdi_init(FUNCTION_T *FUNCTION)
{
	can_cfg_t cfg_pdi_can = {
		.baud = 500000,
	};
	Function_Init(FUNCTION);              // init the function point array
	pdi_relay_init();                   // mbi5024 init
	autoliv_init(&pdi_autoliv);         // init the uart for Xuce Up computer
	pdi_can_bus->init(&cfg_pdi_can);	// init can
	usdt_Init(pdi_can_bus);				// init protocol
	pdi_drv_Init();						// led,beep
	pre_check_action();					// flash the green1 red1 green2 red2 and beep
	config_init(autoliv);               // init the Config file like relay and limit file 
	pdi_IGN_off();                      // power off the left position
	position = RIGHT;
	pdi_IGN_off();                      // power off the right position
	position = LEFT;
	//autoliv_send(&pdi_autoliv,"ok!!\r\n");// whether the uart ok?
	memset(autoliv_buf,0,sizeof(autoliv_buf));// clear the autoliv_buf, update in pdi_update and deal in DealAfterReceive and DealAfterStop
	memset(bcodea,0,sizeof(bcodea));
	memset(bcodeb,0,sizeof(bcodeb));
}
int main(void)
{
	sys_init();
	pdi_init(Function);
	/*check process*/
	while(1) {
		sys_update();
		// SlotA == READY : target on
		// SN[0] !=0 : receive the SN
		// SlotB != COMPLETE : Product B has been tested
		if((SlotA == READY) && (SN[0] != 0) && (SlotB != COMPLETE) && (position == LEFT)) {
			TestStart();
			Testbarcode();
			//TestSN();
			memset(SN,0,sizeof(SN));
			TestECU();
			//TestIMU();
			TestStop();
			DealAfterStop(Function,3000);          // for upload to itac, overtime is 3000 ms
			error_result = ERROR_NO;               // set default condition
			SlotA = COMPLETE;
			position = RIGHT;
			printf("##START##STATUS-100##END##\n");
			memset(bcodea,0,sizeof(bcodea));			
		}
		if((SlotB == READY) && (SN[0] != 0) && (SlotA != COMPLETE) && (position == RIGHT)) {
			TestStart();
			Testbarcode();
			//TestSN();
			memset(SN,0,sizeof(SN));
			TestECU();
			//TestIMU();
			TestStop();
			DealAfterStop(Function,3000);
			error_result = ERROR_NO;               // set default condition
			SlotB = COMPLETE;
			position = LEFT;
			printf("##START##STATUS-100##END##\n");
			memset(bcodeb,0,sizeof(bcodeb));			
		}
	}
}

#if 1
static int cmd_tl_func(int argc, char *argv[])
{
	int num_fault, i;

	const char *usage = {
		"tl , usage:\r\n"
		"tl fault		read the error code\r\n"
		"tl start     test the tl \r\n"
		"tl sn          read the sn\r\n"
		"tl barcode     read barcode inner\r\n"
		"tl softversion \r\n "
		"tl IMU    read IMU can message\r\n"
		"tl clear       clear the history error\r\n"
		"tl lock x      lock the fixture x\r\n"
		"tl unlock x    unlock the fixture x\r\n"
		"tl printf relay file \r\n"
		"tl get position  \r\n"
		"tl set position x  left or right\r\n"
		"tl set relay \r\n"
	};

	if (argc < 2) {
		printf("%s", usage);
		return 0;
	}

	if (argc == 2) {
		if(argv[1][0] == 'f') {
			pdi_IGN_on();
			pdi_can();
			//pdi_set_relays(autoliv);
			pdi_mdelay(1000);
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			}
			memset(tl_fault_buf, 0, sizeof(tl_fault_buf));
			pdi_startsession();
			if (tl_GetFault(tl_fault_buf, &num_fault))
				printf("##ERROR##\n");
			else {
				printf("##OK##\n");
				printf("num of fault is: %d\n", num_fault);
				for (i = 0; i < num_fault * 3; i += 3)
					printf("0x%02x, 0x%02x, 0x%02x\n", tl_fault_buf[i] & 0xff,
					tl_fault_buf[i + 1] & 0xff, tl_fault_buf[i + 2] & 0xff);
			}
			pdi_IGN_off();
		}
		if(!strcmp(argv[1],"start")) {
			pdi_IGN_on();
			pdi_can();
			pdi_startsession();
		}
		if(!strcmp(argv[1],"softversion")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			}
			pdi_startsession();
			tl_read_softversion();
		}
		if(!strcmp(argv[1],"clear")) {
			if(position == LEFT) {
				printf("Current position is Left\r\n");
			}
			else {
				printf("Current position is Right\r\n");
			}
			//pdi_can();
			pdi_IGN_on();
			pdi_mdelay(3000);
			if(pdi_clear_dtc()) {
				printf("##ERROR##\n");
			} else {
				printf("##OK##\n");
			}
			pdi_IGN_off();
		}
		if(!strcmp(argv[1],"sn")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			}
			pdi_startsession();
			tl_read_sn();
		}
		if(!strcmp(argv[1],"cpn")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			}
			pdi_startsession();
			tl_read_cpn();
		}
		if(!strcmp(argv[1],"barcode")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			}
			pdi_startsession();
			tl_read_barcode();
		}
		if(!strcmp(argv[1],"IMU")) {
			pdi_IGN_on();
			pdi_can();
			for(int i = 0; i < 20; i++) {
				pdi_mdelay(89);
			}
			printf("Check IMU NOW \r\n");
			tl_esc_check();
		}
		if(!strcmp(argv[1],"target")) {
			if(!target1_on()) {
				printf("LEFT TARGET OK \r\n");
			} else {
				printf("LEFT TARGET NOT OK\r\n");
			}
			if(!target2_on()) {
				printf("RIGHT TARGET OK \r\n");
			} else {
				printf("RIGHT TARGET NOT OK\r\n");
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
			//pdi_unlock();
		}
		if(!strcmp(argv[1],"lock")) {
			if(!strcmp(argv[2],"1")){
				position = LEFT;
			}
			if(!strcmp(argv[2],"2")){
				position = RIGHT;
			}
			//pdi_lock();
		}
		if(!strcmp(argv[2],"position")) {
			if(position == LEFT) {
				printf("Current position is Left\r\n");
			}
			else {
				printf("Current position is Right\r\n");
			}
		}
		if(!strcmp(argv[2],"relay")) {
			pdi_set_relays(autoliv);
			char relay_temp [8];
			memset(relay_temp,'0',sizeof(relay_temp));
			for(int i = 0; i < 16; i = i + 2) {
				relay_temp[(i + 1) / 2]=(autoliv.relay[i] & 0x0f ) | ((autoliv.relay[i + 1] & 0x0f)  << 4 );
			}
			for(int i = 0; i < 8; i ++) {
				printf("%x ",relay_temp[i] & 0xff);
			}
			printf("\r\n");
			for(int i = 0; i < 8; i ++) {
				printf("%x ",(~relay_temp[i]) & 0xff);
			}
			printf("\r\n");
			position = (POSITION_E)!position;
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
		if(!strcmp(argv[2],"relay")) {
			printf("Rx:relay :\r\n");
			for(int i = 0; i < length_relay; i++) {
				printf("%x ",autoliv.relay[i]);
				if((i + 1) % 6 == 0) { // one line 6 bytes
					printf("\r\n");
				}
			}
			char relay_temp [8];
			memset(relay_temp,'0',sizeof(relay_temp));
			for(int i = 0; i < 16; i = i + 2) {
				relay_temp[(i + 1) / 2]=(autoliv.relay[i] & 0x0f ) | ((autoliv.relay[i + 1] & 0x0f)  << 4 );
			}
			for(int i = 0; i < 8; i ++) {
				printf("%x ",relay_temp[i] & 0xff);
			}
			printf("\r\n");
			for(int i = 0; i < 8; i ++) {
				printf("%x ",(~relay_temp[i]) & 0xff);
			}
			printf("\r\n");
		}
	}
	return 0;
}
const cmd_t cmd_tl = {"tl", cmd_tl_func, "tl cmd i/f"};
DECLARE_SHELL_CMD(cmd_tl)
#endif