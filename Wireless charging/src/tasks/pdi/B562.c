/*
 *David peng.guo@2011 initial version
 *jiamao.gu@2013 5.29 change
 *the length of b562rcode must larger one byte than the SN: this one time, Max3232 5V is forget, 12V is forget too
 *SN is the b562rcode!!!!
 *add filter in usdt.c 
 *add gpio config in drv.c and usdt.c like CONFIG_PDI_b562_gujiamao 
 *Read error code: buffer must clear before reading and if read 0 0 likewise, break the function
 *in limit file , error expected must captical, even if display in little mode
 *jiamao.gu@2013 9.16 change
 *rewrite the function of check error
 *rewrite the function of set relay 
 *add the communication with Xuce Company's Up Computer(Step0,Step1,Step2,Step3)
 *add the Ping Pang Operation for the Autoliv
 *optimize the b562rcode length, need change once ahead not as usual
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

config_t autoliv;	
#define PDI_DEBUG		0        // Test dubug by putty
#define Length_barcode 12        // TBD for different Product
#define Length_SN      14		 // TBD for different Product
#define ID 0x737
static const autoliv_t pdi_autoliv = {
		.bus = &uart3,
		.data_len = 600,              // Xuce Up computer config file max length < 200, I think
		.dead_time = 20
};
static const can_bus_t* pdi_can_bus = &can1;
//pdi b562 can msg
static const can_msg_t b562_clrdtc_msg =		{ID, 8, {0x04, 0x31, 0x01, 0xF0, 0x05,0, 0, 0}, 0};
//static const can_msg_t b562_resis =			{0x607, 8, {0x03, 0x22, 0xfe, 0x80, 0, 0, 0, 0}, 0};
#if PDI_DEBUG
static const can_msg_t b562_errcode_msg =		{ID, 8, {0x03, 0x19, 0xfd, 0x39, 0, 0, 0, 0}, 0};
static const can_msg_t b562_getsn_msg =			{ID, 8, {0x03, 0x22, 0xfe, 0x8d, 0, 0, 0, 0}, 0};
static const can_msg_t b562_part_msg =			{0x7D2, 8, {0x03, 0x22, 0xF1, 0x00, 0x55, 0x55, 0x55, 0x55}, 0};
#endif
static const can_msg_t b562_reqseed_msg =		{ID, 8, {0x02, 0x27, 0x61, 0, 0, 0, 0, 0}, 0};
static const can_msg_t b562_start_msg =			{ID, 8, {0x02, 0x10, 0x03, 0, 0, 0, 0, 0}, 0};
static can_msg_t b562_msg_buf[32];		// for multi frame buffer
static char b562_data_buf[256];			// data buffer
static char b562_fault_buf[64];			// fault buffer
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
static int b562_GetCID(char sid,short cid, char *data);
static int b562_GetFault(char *data, int *pnum_fault);
// communicate with up computer, drive function 
void Step0(void);
void Step1(void);
void Step2(void);
void Step3(void);   // for receiving ITAC-OK or ITAC-NG
void Step4(void);   // for SN check
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
		{0x05, 0x27, 0x62, 0xff, 0xff, 0, 0, 0},
		0
	}; // TBD By Me
	int try_time_start = 5;
	do {
		usdt_GetDiagFirstFrame(&b562_start_msg, 1, NULL, &msg, &msg_len);		//start session, why twice?	
		try_time_start --;
	}while((msg.data[1]!= 0x50) && (try_time_start != 0)); 
	if(try_time_start == 0) {
		return 1;
	}
#if PDI_DEBUG
	printf("111111\r\n");
	can_msg_print(&b562_start_msg,"\n");
	can_msg_print(&msg, "\n");
#endif
	int try_time_reqseed = 5;
	do {
		usdt_GetDiagFirstFrame(&b562_reqseed_msg, 1, NULL, &msg, &msg_len);		//req seed
		try_time_reqseed --;
	}while((msg.data[1]!= 0x67) && (try_time_reqseed != 0));
	if(try_time_reqseed == 0) {
		return 1;
	}	
#if PDI_DEBUG
	printf("222222\r\n");
	can_msg_print(&b562_reqseed_msg,"\n");
	can_msg_print(&msg, "\n");
#endif
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
	usdt_GetDiagFirstFrame(&b562_getsn_msg, 1, NULL, b562_msg_buf, &msg_len);
	if (msg_len > 1)
		usdt_GetDiagLeftFrame(b562_msg_buf, msg_len);
	for (i = 0; i < msg_len; i ++)
		can_msg_print(b562_msg_buf + i, "\n");

	/*get error code*/
	printf("\nError Code:\n");
	usdt_GetDiagFirstFrame(&b562_errcode_msg, 1, NULL, b562_msg_buf, &msg_len);
	if (msg_len > 1)
		usdt_GetDiagLeftFrame(b562_msg_buf, msg_len);
	for (i = 0; i < msg_len; i++)
		can_msg_print(b562_msg_buf + i, "\n");
	/*tester point*/
	printf("\nPart:\n");
	usdt_GetDiagFirstFrame(&b562_getpart_msg, 1, NULL, b562_msg_buf, &msg_len);
	if (msg_len > 1)
		usdt_GetDiagLeftFrame(b562_msg_buf, msg_len);
	for (i = 0; i < msg_len; i++)
		can_msg_print(b562_msg_buf + i, "\n");

	if (b562_GetFault(b562_fault_buf, &num_fault))
		printf("##ERROR##\n");
	else {
		printf("##OK##\n");
		printf("num of fault is: %d\n", num_fault);
		for (i = 0; i < num_fault*2; i += 2)
			printf("0x%02x, 0x%02x\n", b562_fault_buf[i]&0xff, b562_fault_buf[i+1]&0xff);
	}
	/*clear all*/
	printf("\nClear all:\n");
	if (usdt_GetDiagFirstFrame(&b562_clrdtc_msg, 1, NULL, &msg, &msg_len))	//req seed
		return 1;
	can_msg_print(&msg, "\n");
#endif
#endif

	return 0;
}
//
//whether some bytes in range 
//
static int b562_esc_check()
{
	int a = 0, b = 0;
	unsigned short v1 = 0, v2 = 0, v3 = 0;
	can_msg_t esc_msg;
	time_t deadtime = time_get(100);
	printf("##START##EC-Checking ESC...##END##\n");
	while(time_left(deadtime) > 0) {             
		pdi_can_bus -> recv(&esc_msg);
		switch(esc_msg.id) {
		case 0x130:
			// Update by jiamao.gu 9.29 in Shanghai
			if(a == 1) break;
			esc_msg.data[2] &= 0xf0; //compare the low 4 bits, must clear high 4 bits
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
			}			
			a = 1;
			break;
		case 0x140:
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
static int b562_GetCID(char sid,short cid, char *data)
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
		b562_msg_buf[0] = msg_res;
		if(usdt_GetDiagLeftFrame(b562_msg_buf, msg_len))
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
			memcpy(data, (b562_msg_buf + i)->data + 1, 7);
			data += 7;
		}
	}
	return 0;
}
/* read barcode*/
// return 1 : no date
// retrun 0 : ok
static int b562_read_barcode() 
{
	char read_bcode[Length_barcode + 1];
	int try_times = 5;
	while (b562_GetCID(0x22,0xfd42, b562_data_buf)) {
		try_times --;
		if (try_times < 0)
			return 1;
	}
	memcpy(read_bcode, b562_data_buf, Length_barcode);
	printf("%s",read_bcode); // add '\0' in the end
	return 0;
}
/* checking b562rcode */
// return 1 : no data; not equal
// return 0 : ok
static int b562_check_barcode()
{
	char read_bcode[Length_barcode + 1];
	int try_times = 5;
	printf("##START##EC-Checking barcode...##END##\n");
	while (b562_GetCID(0x22,0xfd42, b562_data_buf)) {
		try_times --;
		if (try_times < 0)
			return 1;
	}
	memcpy(read_bcode, b562_data_buf, Length_barcode);
	printf("##START##RB-");
	printf(read_bcode,'\0');
	printf("##END##\n");
	if(position == LEFT) {
		if (memcmp(read_bcode, bcodea, Length_barcode))  //maybe Length_b562rcode - 1 ??
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
static int b562_read_sn()
{
	char read_sn[Length_SN + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xf111,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[Length_SN] = '\0';
	memcpy(read_sn,b562_data_buf,sizeof(read_sn));
	printf("SN:%s\r\n",read_sn);
	return 0;
}
/* read NVM file name*/
//return 1 : no date
//return 0 : ok
static int b562_read_NVM()
{
	char read_nvm[32 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd44,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[32] = '\0';
	memcpy(read_nvm,b562_data_buf,sizeof(read_nvm));
	printf("NVM:%s\r\n",read_nvm);
	return 0;
}
static int b562_check_NVM()
{
	char read[length_NVM + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd44,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_NVM] = '\0';
	memcpy(read,b562_data_buf,sizeof(read));
	printf("NVM Read from ECU : \r\n");
	printf("%s\r\n",read);
	printf("Compare the NVM :");
	if(memcmp(autoliv.NVM, read, length_NVM))
		return 1;
	return 0;
}
ERROR_E Test_NVM(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_NVM()) {
		error_result = ERROR_NVM;
		return error_result;
	}
	printf("####EC-      Checking NVM Done...##END##\n");
	return ERROR_NO;
}
/* read VIN */
//return 1 : no date
//return 0 : ok
static int b562_read_VIN()
{
	char read_vin[17 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xde00,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[17] = '\0';
	memcpy(read_vin,b562_data_buf,sizeof(read_vin));
	printf("VIN:\r\n");
	for(int i = 0; i < 17; i++) {
		printf("%x",read_vin[i]);
	}
	printf("\r\n");
	return 0;
}
static int b562_check_VIN()
{
	char read[length_VIN + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xde00,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_VIN] = '\0';
	memcpy(read,b562_data_buf,sizeof(read));
	printf("VIN Read from ECU : \r\n");
	for(int i = 0; i < length_VIN; i++) {
		printf("%x",read[i]);
	}
	printf("\r\n");
	printf("Compare the VIN :");
	if(memcmp(autoliv.VIN, read, length_VIN))
		return 1;
	return 0;
}
ERROR_E Test_VIN(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_VIN()) {
		error_result = ERROR_VIN;
		return error_result;
	}
	printf("####EC-      Checking VIN Done...##END##\n");
	return ERROR_NO;
}
/* read ROC */
//return 1 : no date
//return 0 : ok
static int b562_read_ROC()
{
	char read_roc[8 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xde01,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[8] = '\0';
	memcpy(read_roc,b562_data_buf,sizeof(read_roc));
	printf("ROC:\r\n");
	for(int i = 0; i < 8; i++) {
		printf("%x",read_roc[i]);
	}
	printf("\r\n");
	printf("%s\r\n",read_roc);
	return 0;
}
static int b562_check_ROC()
{
	char read[length_ROC + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xde01,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_VIN] = '\0';
	memcpy(read,b562_data_buf,sizeof(read));
	printf("ROC Read from ECU : \r\n");
	for(int i = 0; i < length_ROC; i++) {
		printf("%x",read[i]);
	}
	printf("\r\n");
	printf("Compare the ROC :");
	if(memcmp(autoliv.ROC, read, length_ROC))
		return 1;
	return 0;
}
ERROR_E Test_ROC(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_ROC()) {
		error_result = ERROR_ROC;
		return error_result;
	}
	printf("####EC-      Checking ROC Done...##END##\n");
	return ERROR_NO;
}
/* read BE */
//return 1 : no date
//return 0 : ok
static int b562_read_BE()
{
	char read_BE;
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xde03,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	read_BE = b562_data_buf[0];
	printf("BE:0x%x\r\n",read_BE);
	return 0;
}
static int b562_check_BE()
{
	char read;
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xde03,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_BE] = '\0';
	read = b562_data_buf[0];
	printf("BE Read from ECU : \r\n");
	printf("%x\r\n",read);
	printf("Compare the BE :");
	if(autoliv.BE != read)
		return 1;
	return 0;
}
ERROR_E Test_BE(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_BE()) {
		error_result = ERROR_BE;
		return error_result;
	}
	printf("####EC-      Checking BE Done...##END##\n");
	return ERROR_NO;
}
/* read RCM */
//return 1 : no date
//return 0 : ok
static int b562_read_RCM()
{
	char read_RCM[2];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xde04,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	read_RCM[0] = b562_data_buf[0];
	read_RCM[1] = b562_data_buf[1];
	printf("RCM:0x%x%x\r\n",read_RCM[0],read_RCM[1]);
	return 0;
}
//
static int b562_check_RCM()
{
	char read[length_RCM];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xde04,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_RCM] = '\0';
	read[0] = b562_data_buf[0];
	read[1] = b562_data_buf[1];
	printf("RCM Read from ECU : \r\n");
	printf("%x %x\r\n",read[0],read[1]);
	printf("Compare the RCM :");
	if((autoliv.RCM[0] != read[0]) || (autoliv.RCM[1] != read[1]))
		return 1;
	return 0;
}
ERROR_E Test_RCM(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_RCM()) {
		error_result = ERROR_RCM;
		return error_result;
	}
	printf("####EC-      Checking RCM Done...##END##\n");
	return ERROR_NO;
}
/* read OCS */
//return 1 : no date
//return 0 : ok
static int b562_read_OCS()
{
	char read_OCS[2];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xde05,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	read_OCS[0] = b562_data_buf[0];
	read_OCS[1] = b562_data_buf[1];
	printf("OCS:0x%x%x\r\n",read_OCS[0],read_OCS[1]);
	return 0;
}
static int b562_check_OCS()
{
	char read[length_OCS];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xde05,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_OCS] = '\0';
	read[0] = b562_data_buf[0];
	read[1] = b562_data_buf[1];
	printf("OCS Read from ECU : \r\n");
	printf("%x %x\r\n",read[0],read[1]);
	printf("Compare the OCS :");
	if((autoliv.OCS[0] != read[0]) || (autoliv.OCS[1] != read[1]))
		return 1;
	return 0;
}
ERROR_E Test_OCS(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_OCS()) {
		error_result = ERROR_OCS;
		return error_result;
	}
	printf("####EC-      Checking OCS Done...##END##\n");
	return ERROR_NO;
}
/* read NVM file name*/
//return 1 : no date
//return 0 : ok
static int b562_read_APPS()
{
	char read_apps[32 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd41,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[32] = '\0';
	memcpy(read_apps,b562_data_buf,sizeof(read_apps));
	printf("APPS:%s\r\n",read_apps);
	return 0;
}
static int b562_check_APPS()
{
	char read[length_APPS + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd41,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_APPS] = '\0';
	memcpy(read,b562_data_buf,sizeof(read));
	printf("APPS Read from ECU : \r\n");
	printf("%s\r\n",read);
	printf("Compare the APPS :");
	if(memcmp(autoliv.APPS, read, length_APPS))
		return 1;
	return 0;
}
ERROR_E Test_APPS(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_APPS()) {
		error_result = ERROR_APPS;
		return error_result;
	}
	printf("####EC-      Checking APPS Done...##END##\n");
	return ERROR_NO;
}
/* read ASI*/
//return 1 : no date
//return 0 : ok
static int b562_read_ASI()
{
	char read_asi[12 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd42,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[12] = '\0';
	memcpy(read_asi,b562_data_buf,sizeof(read_asi));
	printf("Application Software ID:%s\r\n",read_asi);
	return 0;
}
static int b562_check_ASI()
{
	char read[length_ASI + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd42,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_ASI] = '\0';
	memcpy(read,b562_data_buf,sizeof(read));
	printf("ASI Read from ECU : \r\n");
	printf("%s\r\n",read);
	printf("Compare the ASI :");
	if(memcmp(autoliv.ASI, read, length_ASI))
		return 1;
	return 0;
}
ERROR_E Test_ASI(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_ASI()) {
		error_result = ERROR_ASI;
		return error_result;
	}
	printf("####EC-      Checking ASI Done...##END##\n");
	return ERROR_NO;
}
/* read Variant1*/
//return 1 : no date
//return 0 : ok
static int b562_read_Variant1()
{
	char read_variant1[32 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd96,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[32] = '\0';
	memcpy(read_variant1,b562_data_buf,sizeof(read_variant1));
	printf("Variant1 Calibration File name:%s\r\n",read_variant1);
	return 0;
}
static int b562_check_Variant1()
{
	char read[length_VAR1 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd96,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_VAR1] = '\0';
	memcpy(read,b562_data_buf,sizeof(read));
	printf("VAR1 Read from ECU : \r\n");
	printf("%s\r\n",read);
	printf("Compare the VAR1 :");
	if(memcmp(autoliv.VAR1, read, length_VAR1))
		return 1;
	return 0;
}
ERROR_E Test_Variant1(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_Variant1()) {
		error_result = ERROR_VAR1;
		return error_result;
	}
	printf("####EC-      Checking VAR1 Done...##END##\n");
	return ERROR_NO;
}
/* read Variant2*/
//return 1 : no date
//return 0 : ok
static int b562_read_Variant2()
{
	char read_variant2[32 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd97,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[32] = '\0';
	memcpy(read_variant2,b562_data_buf,sizeof(read_variant2));
	printf("Variant2 Calibration File name:%s\r\n",read_variant2);
	return 0;
}
static int b562_check_Variant2()
{
	char read[length_VAR2 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd97,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_VAR2] = '\0';
	memcpy(read,b562_data_buf,sizeof(read));
	printf("VAR2 Read from ECU : \r\n");
	printf("%s\r\n",read);
	printf("Compare the VAR2 :");
	if(memcmp(autoliv.VAR2, read, length_VAR2))
		return 1;
	return 0;
}
ERROR_E Test_Variant2(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_Variant2()) {
		error_result = ERROR_VAR2;
		return error_result;
	}
	printf("####EC-      Checking VAR2 Done...##END##\n");
	return ERROR_NO;
}
/* read Variant3*/
//return 1 : no date
//return 0 : ok
static int b562_read_Variant3()
{
	char read_variant3[32 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd98,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[32] = '\0';
	memcpy(read_variant3,b562_data_buf,sizeof(read_variant3));
	printf("Variant3 Calibration File name:%s\r\n",read_variant3);
	return 0;
}
static int b562_check_Variant3()
{
	char read[length_VAR3 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd98,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_VAR3] = '\0';
	memcpy(read,b562_data_buf,sizeof(read));
	printf("VAR3 Read from ECU : \r\n");
	printf("%s\r\n",read);
	printf("Compare the VAR3 :");
	if(memcmp(autoliv.VAR3, read, length_VAR3))
		return 1;
	return 0;
}
ERROR_E Test_Variant3(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_Variant3()) {
		error_result = ERROR_VAR3;
		return error_result;
	}
	printf("####EC-      Checking VAR3 Done...##END##\n");
	return ERROR_NO;
}
/* read Variant1AEF*/
//return 1 : no date
//return 0 : ok
static int b562_read_Variant1AEF()
{
	char read_variant1aef[32 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd99,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[32] = '\0';
	memcpy(read_variant1aef,b562_data_buf,sizeof(read_variant1aef));
	printf("Variant1 Calibration AEF name:%s\r\n",read_variant1aef);
	return 0;
}
static int b562_check_Variant1AEF()
{
	char read[length_VAR1AEF + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd99,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_VAR1AEF] = '\0';
	memcpy(read,b562_data_buf,sizeof(read));
	printf("VAR1AEF Read from ECU : \r\n");
	printf("%s\r\n",read);
	printf("Compare the VAR1AEF :");
	if(memcmp(autoliv.VAR1AEF, read, length_VAR1AEF))
		return 1;
	return 0;
}
ERROR_E Test_Variant1AEF(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_Variant1AEF()) {
		error_result = ERROR_VAR1AEF;
		return error_result;
	}
	printf("####EC-      Checking VAR1AEF Done...##END##\n");
	return ERROR_NO;
}
/* read Variant2AEF*/
//return 1 : no date
//return 0 : ok
static int b562_read_Variant2AEF()
{
	char read_variant2aef[32 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd9a,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[32] = '\0';
	memcpy(read_variant2aef,b562_data_buf,sizeof(read_variant2aef));
	printf("Variant2 Calibration AEF name:%s\r\n",read_variant2aef);
	return 0;
}
static int b562_check_Variant2AEF()
{
	char read[length_VAR2AEF + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd9a,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_VAR2AEF] = '\0';
	memcpy(read,b562_data_buf,sizeof(read));
	printf("VAR2AEF Read from ECU : \r\n");
	printf("%s\r\n",read);
	printf("Compare the VAR2AEF :");
	if(memcmp(autoliv.VAR2AEF, read, length_VAR2AEF))
		return 1;
	return 0;
}
ERROR_E Test_Variant2AEF(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_Variant2AEF()) {
		error_result = ERROR_VAR2AEF;
		return error_result;
	}
	printf("####EC-      Checking VAR2AEF Done...##END##\n");
	return ERROR_NO;
}
/* read Variant3AEF*/
//return 1 : no date
//return 0 : ok
static int b562_read_Variant3AEF()
{
	char read_variant3aef[32 + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd9b,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[32] = '\0';
	memcpy(read_variant3aef,b562_data_buf,sizeof(read_variant3aef));
	printf("Variant3 Calibration AEF name:%s\r\n",read_variant3aef);
	return 0;
}
static int b562_check_Variant3AEF()
{
	char read[length_VAR3AEF + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd9b,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[length_VAR2AEF] = '\0';
	memcpy(read,b562_data_buf,sizeof(read));
	printf("VAR2AEF Read from ECU : \r\n");
	printf("%s\r\n",read);
	printf("Compare the VAR2AEF :");
	if(memcmp(autoliv.VAR2AEF, read, length_VAR2AEF))
		return 1;
	return 0;
}
ERROR_E Test_Variant3AEF(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_Variant3AEF()) {
		error_result = ERROR_VAR3AEF;
		return error_result;
	}
	printf("####EC-      Checking VAR3AEF Done...##END##\n");
	return ERROR_NO;
}
/* equal Serial Number */
//return 1 : not equal
//return 0 : ok
static int b562_check_sn()
{
	char read_sn[Length_SN + 1];
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xf111,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[Length_SN] = '\0';
	memcpy(read_sn,b562_data_buf,sizeof(read_sn));
	printf("SN Read from ECU : \r\n");
	printf("%s\r\n",read_sn);
	printf("Compare the SN :");
	if(memcmp(SN, read_sn, Length_SN))
		return 1;
	return 0;
}
#define L 12
static int b562_read_softversion()
{
	char read_softversion[L + 1];  // 12 means length
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd42,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[L] = '\0';
	memcpy(read_softversion,b562_data_buf,sizeof(read_softversion));
	printf("%s\r\n",read_softversion);
	return 0;
}
static int b562_check_softversion()
{
	char read_softversion[11 + 1];  // 12 means length
	int try_times = 5;
	memset(b562_data_buf,0,sizeof(b562_data_buf));
	while(b562_GetCID(0x22,0xfd42,b562_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	b562_data_buf[11] = '\0';
	memcpy(read_softversion,b562_data_buf,sizeof(read_softversion));
	printf("Read softversion from ECU : %s\r\n",read_softversion);
	printf("Read softversion from Up : %s\r\n",autoliv.softversion);
	if(memcmp(autoliv.softversion,read_softversion,11))
		return 1;
	return 0;
}
/*clear error code*/
int pdi_clear_dtc(void)
{
	int msg_len;
	can_msg_t msg;
	pdi_startsession();
	if (usdt_GetDiagFirstFrame(&b562_clrdtc_msg, 1, NULL, &msg, &msg_len))
		return 1;
	if (msg.data[1] != (b562_clrdtc_msg.data[1] + 0x40))	// TBD By Me
		return 1;
	return 0;
}

/*get fault from ECU,saved to data,the nr of fault is pnum_fault*/
static int b562_GetFault(char *data, int *pnum_fault)
{
	int i, result = 0;
	if (b562_GetCID(0x19,0x028b, data))
		return 1;
	//memset(data + 117, 0x00, 10);
	for (i = 0; i < 117; i += 4) {
		if((data[i] | data[i+1] | data[i+2] | data[i+3]) == 0)
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
		counter_pass_add();
	}
	if(target1_on()) {
		if(position == LEFT) {
			error_result = ERROR_CW;
		}
		SlotA = IDLE;  // add 10.9 in Suzhou
		his_a = 0;
	}
	if(!target2_on() && (his_b == 0)) {
		his_b = 1;
		counter_fail_add();
	}
	if(target2_on()) {
		if(position == RIGHT) {
			error_result = ERROR_CW;
		}
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
** pick up the b562rcode ****************** 
** set the relay according to relay file*
** return error information *************
*****************************************/
ERROR_E TestStart(void)
{
	if(position == LEFT) {
		printf("Product A will be tested\n");
		autoliv_send(&pdi_autoliv,"UUTA-ST"); // for communicate with the upper computer
	}
	else {
		printf("Product B will be tested\n");
		autoliv_send(&pdi_autoliv,"UUTB-ST");
	}
	pdi_can();								// set the can relay
	pdi_mdelay(50);
	pdi_set_relays(autoliv);              	// set the load relays, may exist bugs
	pdi_mdelay(50);
	pdi_IGN_on();                         	// set the power relay or switch
	error_result = ERROR_NO;
	return ERROR_NO;
}
/*******************************************
** Blink the yellow led ********************
** compare b562rcode from ECU and scaner  **
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
		printf("##START##STATUS-");		// in such format, so the data can be displayed in processb562r
		sprintf(processbar, "%d", rate);
		printf("%s", processbar);
		printf("##END##\n");
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_barcode()) {
		error_result = ERROR_BW;
		return error_result;
	}
	printf("####EC-      Checking barcode Done...##END##\n");
	return ERROR_NO;
}
ERROR_E TestSN(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	pdi_startsession();                 // startsession before getting message
	if (b562_check_sn()) {
		error_result = ERROR_SN;
		return error_result;
	}
	printf("####EC-      Checking SN Done...##END##\n");
	printf("Clear the SN buffer for next test !\r\n");
	memset(SN,0,sizeof(SN));
	return ERROR_NO;
}
ERROR_E TestJama(void)
{
	if(error_result != ERROR_NO) {
		return error_result;
	}
	if(autoliv.JAMA == '1') {
		printf("Test JAMA begin\r\n");
		if(!JAMA_on()) {
			printf("JAMA OK\r\n");
		} else {
			printf("JAMA not Ok\r\n");
			error_result = ERROR_JAMA;
			return error_result;
		}
	} else {
		printf("NO JAMA\r\n");
		if(!JAMA_on()) {
			error_result = ERROR_JAMA;
			return error_result;
		}
	}
	return error_result;
}
ERROR_E TestIMU(void)
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	if(autoliv.IMU == '1') {
		printf("Test IMU Can begin \r\n");
		if(!b562_esc_check()) {
			error_result = ERROR_NO;
		}
		else {
			error_result = ERROR_IMU;
		}
	} else {
		printf("No IMU Can \r\n");
		error_result = ERROR_NO;
	}	
	return error_result;
}
/**********************************************
** process b562r running ************************
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
	if(error_result != ERROR_NO) {
		return error_result;
	}
	memset(b562_fault_buf,0,sizeof(b562_fault_buf)); // clear this buffer, if not, you will see the bug
	int try_times = 0;
	int result = 0;
	int num_fault = 0;
	/**** when result is 1, must get fault again, oversize 3 times *****
	**** num_fault is not 0, check the error_code ********************** 
	*******************************************************************/
	pdi_startsession();
	do{
		result = b562_GetFault(b562_fault_buf,&num_fault);
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
		for (int i = 0; i < num_fault * 4; i += 4)  // according to size of error code , sometimes 3 bytes 
			printf("0x%02x, 0x%02x,0x%02x,0x%02x\n", b562_fault_buf[i]&0xff,b562_fault_buf[i+1]&0xff,b562_fault_buf[i+2]&0xff,b562_fault_buf[i+3]&0xff);
		printf("##END##\n");
		if(pdi_error_check(b562_fault_buf, &num_fault, autoliv)) { // check whether the error code is allowable
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
	pdi_IGN_off();
	int msg_len;
	can_msg_t msg;	
	do {                                // if no extern CAN
		pdi_mdelay(1000);
	} while(!usdt_GetDiagFirstFrame(&b562_start_msg, 1, NULL, &msg, &msg_len));
}
/******************************
** change the status **********
** unlock the fixture *********
** display the led ************
******************************/
void TestStop(void)
{
	Poweroff();
	if(position == LEFT) {
		switch(error_result) {
			case ERROR_NO:
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-OK");
				printf("UUTA-FUNC-OK\r\n");
				break;
			case ERROR_NRP:                                  // not happened forever
				printf("##START##EC-Target A is not on the right position##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				printf("UUTA-FUNC-NG\r\n");
				break;
			case ERROR_NCF:                                  // not happened forever
				printf("##START##EC-No This Config File A ##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				printf("UUTA-FUNC-NG\r\n");
				break;
			case ERROR_BW:
				printf("##START##EC-barcode A Wrong##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 10);
				printf("UUTA-FUNC-NG10\r\n");
				break;
			case ERROR_SN:
				printf("##START##EC-SN A Wrong##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				printf("UUTA-FUNC-NG12\r\n");
				autoliv_send_char(&pdi_autoliv, 12);
				break;
			case ERROR_SV:
				printf("##START##EC-SV A Wrong##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				printf("UUTA-FUNC-NG8\r\n");
				autoliv_send_char(&pdi_autoliv, 8);
				break;
			case ERROR_CW:
				printf("##START##STATUS-Get A CW error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv,1);
				printf("UUTA-FUNC-NG1\r\n");
				break;
			case ERROR_CW1:
				printf("##START##STATUS-Get A CW error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv,1);
				printf("UUTA-FUNC-NG1\r\n");
				break;
			case ERROR_CODE:
				printf("##START##STATUS-Get A DTC error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv,6);
				printf("UUTA-FUNC-NG6\r\n");
				break;
			case ERROR_IMU:
				printf("##START##STATUS-Get A IMU error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 16);
				printf("UUTA-FUNC-NG16\r\n");
				break;
			case ERROR_JAMA:
				printf("##START##STATUS-Get A JAMA error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 21);
				printf("UUTA-FUNC-NG21\r\n");
				break;
			case ERROR_NVM:
				printf("##START##STATUS-Get A NVM error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 26);
				printf("UUTA-FUNC-NG26\r\n");
				break;
			case ERROR_VIN:
				printf("##START##STATUS-Get A VIN error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 27);
				printf("UUTA-FUNC-NG27\r\n");
				break;
			case ERROR_ROC:
				printf("##START##STATUS-Get A ROC error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 28);
				printf("UUTA-FUNC-NG28\r\n");
				break;
			case ERROR_BE:
				printf("##START##STATUS-Get A BE error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 29);
				printf("UUTA-FUNC-NG29\r\n");
				break;
			case ERROR_RCM:
				printf("##START##STATUS-Get A RCM error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 30);
				printf("UUTA-FUNC-NG30\r\n");
				break;
			case ERROR_OCS:
				printf("##START##STATUS-Get A OCS error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 31);
				printf("UUTA-FUNC-NG31\r\n");
				break;
			case ERROR_APPS:
				printf("##START##STATUS-Get A APPS error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 32);
				printf("UUTA-FUNC-NG32\r\n");
				break;
			case ERROR_ASI:
				printf("##START##STATUS-Get A ASI error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 33);
				printf("UUTA-FUNC-NG33\r\n");
				break;
			case ERROR_VAR1:
				printf("##START##STATUS-Get A VAR1 error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 34);
				printf("UUTA-FUNC-NG34\r\n");
				break;
			case ERROR_VAR2:
				printf("##START##STATUS-Get A VAR2 error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 35);
				printf("UUTA-FUNC-NG35\r\n");
				break;
			case ERROR_VAR3:
				printf("##START##STATUS-Get A VAR3 error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 36);
				printf("UUTA-FUNC-NG36\r\n");
				break;
			case ERROR_VAR1AEF:
				printf("##START##STATUS-Get A VAR1AEF error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 37);
				printf("UUTA-FUNC-NG37\r\n");
				break;
			case ERROR_VAR2AEF:
				printf("##START##STATUS-Get A VAR2AEF error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 38);
				printf("UUTA-FUNC-NG38\r\n");
				break;
			case ERROR_VAR3AEF:
				printf("##START##STATUS-Get A VAR3AEF error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 39);
				printf("UUTA-FUNC-NG39\r\n");
				break;
			case ERROR_TEC:
				printf("##START##STATUS-Get A TEC error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 40);
				printf("UUTA-FUNC-NG40\r\n");
				break;
			case ERROR_ENCODE:
				printf("##START##EC-These faults are acceptale...##END##\n");
				autoliv_send(&pdi_autoliv,"UUTA-FUNC-OK");
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
				break;
			case ERROR_NRP:
				printf("##START##EC-Target B is not on the right position##END##\n");
				printf("UUTB-FUNC-NG\r\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");			
				break;
			case ERROR_NCF:
				printf("##START##EC-No This Config File B##END##\n");
				printf("UUTB-FUNC-NG\r\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				break;
			case ERROR_BW:
				printf("##START##EC-barcode B Wrong##END##\n");
				printf("UUTB-FUNC-NG11\r\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 11);
				break;
			case ERROR_SN:
				printf("##START##EC-SN B Wrong##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 13);
				printf("UUTB-FUNC-NG13\r\n");
				break;
			case ERROR_SV:
				printf("##START##EC-SV B Wrong##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 9);
				printf("UUTB-FUNC-NG9\r\n");
				break;
			case ERROR_CW:
				printf("##START##STATUS-Get B CW error!##END##\n");
				printf("UUTB-FUNC-NG1\r\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv,1);
				break;
			case ERROR_CW2:
				printf("##START##STATUS-Get B CW error!##END##\n");
				printf("UUTB-FUNC-NG1\r\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv,1);
				break;
			case ERROR_IMU:
				printf("##START##STATUS-Get B IMU error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv,6);
				printf("UUTB-FUNC-NG6\r\n");
				break;
			case ERROR_JAMA:
				printf("##START##STATUS-Get B JAMA error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 21);
				printf("UUTB-FUNC-NG21\r\n");
				break;
			case ERROR_NVM:
				printf("##START##STATUS-Get B NVM error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 26);
				printf("UUTB-FUNC-NG26\r\n");
				break;
			case ERROR_VIN:
				printf("##START##STATUS-Get B VIN error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 27);
				printf("UUTB-FUNC-NG27\r\n");
				break;
			case ERROR_ROC:
				printf("##START##STATUS-Get B ROC error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 28);
				printf("UUTB-FUNC-NG28\r\n");
				break;
			case ERROR_BE:
				printf("##START##STATUS-Get B BE error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 29);
				printf("UUTB-FUNC-NG29\r\n");
				break;
			case ERROR_RCM:
				printf("##START##STATUS-Get B RCM error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 30);
				printf("UUTB-FUNC-NG30\r\n");
				break;
			case ERROR_OCS:
				printf("##START##STATUS-Get B OCS error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 31);
				printf("UUTB-FUNC-NG31\r\n");
				break;
			case ERROR_APPS:
				printf("##START##STATUS-Get B APPS error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 32);
				printf("UUTB-FUNC-NG32\r\n");
				break;
			case ERROR_ASI:
				printf("##START##STATUS-Get B ASI error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 33);
				printf("UUTB-FUNC-NG33\r\n");
				break;
			case ERROR_VAR1:
				printf("##START##STATUS-Get B VAR1 error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 34);
				printf("UUTB-FUNC-NG34\r\n");
				break;
			case ERROR_VAR2:
				printf("##START##STATUS-Get B VAR2 error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 35);
				printf("UUTB-FUNC-NG35\r\n");
				break;
			case ERROR_VAR3:
				printf("##START##STATUS-Get B VAR3 error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 36);
				printf("UUTB-FUNC-NG36\r\n");
				break;
			case ERROR_VAR1AEF:
				printf("##START##STATUS-Get B VAR1AEF error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 37);
				printf("UUTB-FUNC-NG37\r\n");
				break;
			case ERROR_VAR2AEF:
				printf("##START##STATUS-Get B VAR2AEF error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 38);
				printf("UUTB-FUNC-NG38\r\n");
				break;
			case ERROR_VAR3AEF:
				printf("##START##STATUS-Get B VAR3AEF error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 39);
				printf("UUTB-FUNC-NG39\r\n");
				break;
			case ERROR_TEC:
				printf("##START##STATUS-Get B TEC error!##END##\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv, 40);
				printf("UUTB-FUNC-NG40\r\n");
				break;
			case ERROR_CODE:
				printf("##START##STATUS-Get B DTC error!##END##\n");
				printf("UUTB-FUNC-NG7\r\n");				
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
				autoliv_send_char(&pdi_autoliv,7);
				break;
			case ERROR_ENCODE:
				printf("##START##EC-These faults are acceptale...##END##\n");
				printf("UUTB-FUNC-OK\r\n");
				autoliv_send(&pdi_autoliv,"UUTB-FUNC-OK");			
				break;
			default:
				break ;
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
	memset(autoliv_buf,0,sizeof(autoliv_buf));   // Global variable attention!!!
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
	autoliv_send(&pdi_autoliv,"ECUPDIIDLT004");  // Send to Up
	printf("%s\r\n","ECUPDILT004");
}
/***********************************************************************************
*** Up: Send 01010f+part_code+relay+length_softversion+softversion+IMU+error+\r\n***
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
	printf("RX:NVM FILE:\r\n");
	for(int i = 0; i < length_NVM; i++) {
		autoliv.NVM[i] = autoliv_buf[i + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
	}
	printf("%s\r\n",autoliv.NVM);
	printf("RX:VIN:\r\n");
	for(int i = 0; i < length_VIN * 2; i += 2) {
		char a = autoliv_buf[i + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion] - 0x37;
		char b = autoliv_buf[i + 1 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion] - 0x37;
		autoliv.VIN[(i + 1) / 2] =  b << 4 | a;
		printf("%x\t",autoliv.VIN[(i + 1) / 2]);
	}
	printf("\r\n");
	printf("RX:ROC:\r\n");
	for(int i = 0; i < length_ROC * 2; i += 2) {
		char a = autoliv_buf[i + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		char b = autoliv_buf[i + 1 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		if((a <= '9') && (a >= '0')) {
			a -= 0x30;
		} else if((a <= 'Z') && (a >= 'A')) {
			a -= 0x37;
		}
		if((b <= '9') && (b >= '0')) {
			b -= 0x30;
		} else if((b <= 'Z') && (b >= 'A')) {
			b -= 0x37;
		}
		autoliv.ROC[(i + 1) / 2] = a << 4 | b; 
		printf("%x\t",autoliv.ROC[(i + 1) / 2]);
	}
	printf("\r\n");
	printf("RX:BE:\r\n");
	for(int i = 0; i < length_BE * 2; i += 2) {
		char a = autoliv_buf[i + length_ROC *2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		char b = autoliv_buf[i + 1 + length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		if((a <= '9') && (a >= '0')) {
			a -= 0x30;
		} else if((a <= 'Z') && (a >= 'A')) {
			a -= 0x37;
		}
		if((b <= '9') && (b >= '0')) {
			b -= 0x30;
		} else if((b <= 'Z') && (b >= 'A')) {
			b -= 0x37;
		}
		autoliv.BE = a << 4 | b; 
		printf("%x\r\n",autoliv.BE);
	}
	printf("RX:RCM:\r\n");
	for(int i = 0; i < length_RCM * 2; i += 2) {
		char a = autoliv_buf[i + length_BE *2 + length_ROC *2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		char b = autoliv_buf[i + 1 + length_BE *2 + length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		if((a <= '9') && (a >= '0')) {
			a -= 0x30;
		} else if((a <= 'Z') && (a >= 'A')) {
			a -= 0x37;
		}
		if((b <= '9') && (b >= '0')) {
			b -= 0x30;
		} else if((b <= 'Z') && (b >= 'A')) {
			b -= 0x37;
		}
		autoliv.RCM[(i + 1) / 2] = a << 4 | b; 
		printf("%x\t",autoliv.RCM[(i + 1) / 2]);
	}
	printf("\r\n");
	printf("RX:OCS:\r\n");
	for(int i = 0; i < length_OCS * 2; i += 2) {
		char a = autoliv_buf[i + length_RCM * 2+ length_BE *2 + length_ROC *2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		char b = autoliv_buf[i + 1 + length_RCM * 2 + length_BE *2 + length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		if((a <= '9') && (a >= '0')) {
			a -= 0x30;
		} else if((a <= 'Z') && (a >= 'A')) {
			a -= 0x37;
		}
		if((b <= '9') && (b >= '0')) {
			b -= 0x30;
		} else if((b <= 'Z') && (b >= 'A')) {
			b -= 0x37;
		}
		autoliv.OCS[(i + 1) / 2] = a << 4 | b; 
		printf("%x\t",autoliv.OCS[(i + 1) / 2]);
	}
	printf("\r\n");
	printf("RX:APPS:\r\n");
	for(int i = 0; i < length_APPS; i++) {
		autoliv.APPS[i] = autoliv_buf[i + length_OCS * 2 + length_RCM * 2+ length_BE *2 + length_ROC *2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
	}
	printf("%s\r\n",autoliv.APPS);
	printf("RX:ASI:\r\n");
	for(int i = 0; i < length_ASI; i++) {
		autoliv.ASI[i] = autoliv_buf[i +length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
	}
	printf("%s\r\n",autoliv.ASI);
	printf("RX:VAR1\r\n");
	for(int i = 0; i < length_VAR1; i++) {
		autoliv.VAR1[i] = autoliv_buf[i + length_ASI + length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
	}
	printf("%s\r\n",autoliv.VAR1);
	printf("Rx:VAR2\r\n");
	for(int i = 0; i < length_VAR2; i++) {
		autoliv.VAR2[i] = autoliv_buf[i + length_VAR1 + length_ASI + length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		printf("%x\t",autoliv.VAR2[i]);
	}
	printf("\r\n");
	printf("Rx:VAR3\r\n");
	for(int i = 0; i < length_VAR3; i++) {
		autoliv.VAR3[i] = autoliv_buf[i + length_VAR2 + length_VAR1 + length_ASI + length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		printf("%x\t",autoliv.VAR3[i]);
	}
	printf("\r\n");
	printf("RX:VAR1AEF\r\n");
	for(int i = 0; i < length_VAR1AEF; i++) {
		autoliv.VAR1AEF[i] = autoliv_buf[i + length_VAR3 + length_VAR2 + length_VAR1 + length_ASI + length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
	}
	printf("%s\r\n",autoliv.VAR1AEF);
	printf("Rx:VAR2AEF\r\n");
	for(int i = 0; i < length_VAR2AEF; i++) {
		autoliv.VAR2AEF[i] = autoliv_buf[i + length_VAR1AEF + length_VAR3 + length_VAR2 + length_VAR1 + length_ASI + length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		printf("%x\t",autoliv.VAR2AEF[i]);
	}
	printf("\r\n");
	printf("Rx:VAR3AEF\r\n");
	for(int i = 0; i < length_VAR3AEF; i++) {
		autoliv.VAR3AEF[i] = autoliv_buf[i + length_VAR2AEF + length_VAR1AEF + length_VAR3 +length_VAR2 + length_VAR1 + length_ASI + length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		printf("%x\t",autoliv.VAR3AEF[i]);
	}
	printf("\r\n");
	printf("RX:TEC:\r\n");
	for(int i = 0; i < length_TEC * 2; i += 2) {
		char a = autoliv_buf[i + length_VAR3AEF + length_VAR2AEF + length_VAR1AEF + length_VAR3 +length_VAR2 + length_VAR1 + length_ASI + length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		char b = autoliv_buf[i + 1 + length_VAR3AEF + length_VAR2AEF + length_VAR1AEF + length_VAR3 +length_VAR2 + length_VAR1 + length_ASI + length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		if((a <= '9') && (a >= '0')) {
			a -= 0x30;
		} else if((a <= 'Z') && (a >= 'A')) {
			a -= 0x37;
		}
		if((b <= '9') && (b >= '0')) {
			b -= 0x30;
		} else if((b <= 'Z') && (b >= 'A')) {
			b -= 0x37;
		}
		autoliv.TEC = a << 4 | b; 
		printf("%x\r\n",autoliv.TEC);
	}
	//int i = 0; 
	printf("Rx:errorable:\r\n");
	/*while((autoliv_buf[i + length_TEC * 2 + length_VAR3AEF + length_VAR2AEF + length_VAR1AEF + length_VAR3 +length_VAR2 + length_VAR1 + length_ASI + length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion + 2] != 0x0D) && (autoliv_buf[i + length_TEC *2 + length_VAR3AEF + length_VAR2AEF + length_VAR1AEF + length_VAR3 +length_VAR2 + length_VAR1 + length_ASI + length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion + 3] != 0x0A)) {
		autoliv.errorable[i] = autoliv_buf[i + length_TEC * 2 + length_VAR3AEF + length_VAR2AEF + length_VAR1AEF + length_VAR3 +length_VAR2 + length_VAR1 + length_ASI + length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion ];
		printf("%x",autoliv.errorable[i]);
		if((i + 1) % 6 == 0) {
			printf("\r\n");
		}
		i++;
	}*/
	for(int i = 0; i < length_errorable; i++) {
		autoliv.errorable[i] = autoliv_buf[i + length_TEC * 2 + length_VAR3AEF + length_VAR2AEF + length_VAR1AEF + length_VAR3 +length_VAR2 + length_VAR1 + length_ASI + length_APPS + length_OCS * 2 + length_RCM * 2+ length_BE *2 + \
									length_ROC * 2 + length_VIN * 2 + length_NVM + 6 + length_bcode_part + length_relay + autoliv.length_softversion];
		printf("%x",autoliv.errorable[i]);
	}
	printf("\r\n");
	autoliv_send(&pdi_autoliv,"DL-CONF-OK");
	printf("DL-CONF-OK");
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
		autoliv_send(&pdi_autoliv,"BARC-OK");
		printf("BARC-OK\r\n");		
	}
	else {
		memcpy(bcodeb,autoliv_buffer,Length_barcode);
		printf("Rx:barcodeb :\r\n");
		printf("%s\r\n",bcodeb);
		autoliv_send(&pdi_autoliv,"BARC-OK");
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
}
// add by jiamao.gu in Shanghai 2013.10.28
// b562rcode sended after teststop in about three seconds, so need to wait 
// after receiving barcode, SN sended in about five seconds, so need to wait 
// maybe change over_time in different condition !!!
void Receive_from_Up(time_t over_time)
{
	while(time_left(over_time) > 0) {
		if(SN[0] != 0) { // All the receive data put in autoliv_buf			
			printf("Receive other b562rcode and SN success !\r\n");
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
	Function_Init(FUNCTION);              // init the function point array
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
		if((SlotA == READY) && SN[0] != 0 && (SlotB != COMPLETE)) {
			TestStart();
			//Testbarcode();
			Test_NVM();
			Test_VIN();
			Test_ROC();
			Test_BE();
			Test_RCM();
			Test_OCS();
			Test_APPS();
			Test_ASI();
			Test_Variant1();
			//Test_Variant2();
			//Test_Variant3();
			Test_Variant1AEF();
			//TestSN();
			//TestIMU();
			TestECU();
			TestStop();
			DealAfterStop(Function,3000);          // for upload to itac, overtime is 3000 ms
			error_result = ERROR_NO;               // set default condition
			SlotA = COMPLETE;
			position = RIGHT;
			printf("##START##STATUS-100##END##\n");
			memset(bcodea,0,sizeof(bcodea));
			memset(SN,0,sizeof(SN));
		}
		if((SlotB == READY) && SN[0] != 0 && (SlotA != COMPLETE)) {
			TestStart();
			//Testbarcode();
			Test_NVM();
			Test_VIN();
			Test_ROC();
			Test_BE();
			Test_RCM();
			Test_OCS();
			Test_APPS();
			Test_ASI();
			Test_Variant1();
			//Test_Variant2();
			//Test_Variant3();
			Test_Variant1AEF();
			//TestSN();
			//TestIMU();
			TestECU();
			TestStop();
			DealAfterStop(Function,3000);
			error_result = ERROR_NO;               // set default condition
			SlotB = COMPLETE;
			position = LEFT;
			printf("##START##STATUS-100##END##\n");
			memset(bcodeb,0,sizeof(bcodeb));
			memset(SN,0,sizeof(SN));			
		}				
	}
}

#if 1
static int cmd_b562_func(int argc, char *argv[])
{
	int num_fault, i;

	const char *usage = {
		"b562 , usage:\r\n"
		"b562 fault		read the error code\r\n"
		"b562 jama \r\n"
		"b562 target \r\n"
		"b562 start     test the b562 \r\n"
		"b562 sn          read the sn\r\n"
		"b562 nvm\r\n"
		"b562 vin\r\n"
		"b562 roc\r\n"
		"b562 be\r\n"
		"b562 rcm\r\n"
		"b562 ocs\r\n"
		"b562 apps\r\n"
		"b562 asi\r\n"
		"b562 var1\r\n"
		"b562 var2\r\n"
		"b562 var3\r\n"
		"b562 var1aef\r\n"
		"b562 var2aef\r\n"
		"b562 var3aef\r\n"
		"b562 barcode     read barcode inner\r\n"
		"b562 IMU    read IMU can message\r\n"
		"b562 clear       clear the history error\r\n"
		"b562 lock x      lock the fixture x\r\n"
		"b562 unlock x    unlock the fixture x\r\n"
		"b562 printf relay file \r\n"
		"b562 get position  \r\n"
		"b562 set position x  left or right\r\n"
		"b562 set relay \r\n"
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
				//pdi_mdelay(228);	//delay 18s
			}
			pdi_startsession();
			memset(b562_fault_buf,0,sizeof(b562_fault_buf)); 	
			if (b562_GetFault(b562_fault_buf, &num_fault))
				printf("##ERROR##\n");
			else {
				printf("##OK##\n");
				printf("num of fault is: %d\n", num_fault);
				for (i = 0; i < num_fault*4; i += 4)
					printf("0x%02x,0x%02x,0x%02x,0x%02x\n", b562_fault_buf[i]&0xff,
					b562_fault_buf[i+1]&0xff,b562_fault_buf[i+2]&0xff,b562_fault_buf[i+3]&0xff);
			}
			pdi_IGN_off();
		}
		if(!strcmp(argv[1],"start")) {
			pdi_IGN_on();
			pdi_can();
			pdi_startsession();
		}
		if(!strcmp(argv[1],"jama")) {
			if(!JAMA_on()) {
				printf("JAMA ON\r\n");
			} else {
				printf("JAMA OFF\r\n");
			}
		}
		if(!strcmp(argv[1],"target")) {
			if(!target2_on()) {
				printf("RIGHT TARGET OK \r\n");
			} else {
				printf("FAIL\r\n");
			}
		}
		if(!strcmp(argv[1],"nvm")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_NVM();
		}
		if(!strcmp(argv[1],"vin")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_VIN();
		}
		if(!strcmp(argv[1],"roc")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_ROC();
		}
		if(!strcmp(argv[1],"be")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_BE();
		}
		if(!strcmp(argv[1],"rcm")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_RCM();
		}
		if(!strcmp(argv[1],"ocs")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_OCS();
		}
		if(!strcmp(argv[1],"apps")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_APPS();
		}
		if(!strcmp(argv[1],"asi")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_ASI();
		}
		if(!strcmp(argv[1],"var1")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_Variant1();
		}
		if(!strcmp(argv[1],"var2")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_Variant2();
		}
		if(!strcmp(argv[1],"var3")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_Variant3();
		}
		if(!strcmp(argv[1],"var1aef")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_Variant1AEF();
		}
		if(!strcmp(argv[1],"var2aef")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_Variant2AEF();
		}
		if(!strcmp(argv[1],"var3aef")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_Variant3AEF();
		}
		if(!strcmp(argv[1],"softversion")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			} 
			pdi_startsession();
			b562_read_softversion();
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
			pdi_mdelay(3000);
			if(pdi_clear_dtc()) {
				printf("##ERROR##\n");
			}
			else {
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
			b562_read_sn();
		}
		if(!strcmp(argv[1],"barcode")) {
			pdi_IGN_on();
			pdi_can();
			/*delay 7s*/
			for (int rate = 17; rate <= 96; rate ++) {
				pdi_mdelay(89);
			}
			pdi_startsession();
			b562_read_barcode();
		}
		if(!strcmp(argv[1],"IMU")) {
			pdi_IGN_on();
			pdi_can();
			for(int i = 0; i < 20; i++) {
				pdi_mdelay(89);
			}
			printf("Check IMU NOW \r\n");
			b562_esc_check();
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
const cmd_t cmd_b562 = {"b562", cmd_b562_func, "b562 cmd i/f"};
DECLARE_SHELL_CMD(cmd_b562)
#endif