/*
 *David peng.guo@2011 initial version
 *jiamao.gu@2013 5.29 change
 *the length of rclercode must larger one byte than the SN: this one time, Max3232 5V is forget, 12V is forget too
 *SN is the rclercode!!!!
 *add filter in usdt.c 
 *add gpio config in drv.c and usdt.c like CONFIG_PDI_rcle_gujiamao 
 *Read error code: buffer must clear before reading and if read 0 0 likewise, break the function
 *in limit file , error expected must captical, even if display in little mode
 *jiamao.gu@2013 9.16 change
 *rewrite the function of check error
 *rewrite the function of set relay 
 *add the communication with Xuce Company's Up Computer(Step0,Step1,Step2,Step3)
 *add the Ping Pang Operation for the Autoliv
 *optimize the rclercode length, need change once ahead not as usual
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
#include "pwm.h"
#include "priv/usdt.h"
#include "ulp/debug.h"
#include "shell/cmd.h"
#include "common/debounce.h"
#include "pdi.h"

config_t autoliv;	
#define PDI_DEBUG		1        // Test dubug by putty
#define Length_barcode 12        // TBD for different Product
#define Length_SN      18		 // TBD for different Product
#define ID 0x7e5
static CAN_OR_KLINE_E can_or_kline = CAN;
static DEBUG_OR_PRODUCT_E debug_or_product = PRODUCT;
static TYPE_E product_type = Geely;	//current product
static const autoliv_t pdi_autoliv = {
		.bus = &uart3,
		.data_len = 600,              // Xuce Up computer config file max length < 200, I think
		.dead_time = 20
};
static const kline_t pdi_kbus = {
		.bus = &uart2,
		.data_len = 100,              
		.dead_time = 100
};
static const can_bus_t* pdi_can_bus = &can1;
//pdi rcle can msg
static can_msg_t rcle_msg_buf[32];		// for multi frame buffer
static char rcle_data_buf[256];			// data buffer
static char rcle_fault_buf[64];			// fault buffer
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
can_msg_t can_msg1 = {0x68,1,{0},0};//20
can_msg_t can_msg2 = {0x70,8,{0,0,0,0,0,0,0,0},0};//10
can_msg_t can_msg3 = {0x94,8,{0,2,0,0,0,0,0,0},0};//10
can_msg_t can_msg4 = {0x88,1,{0},0};//20
// communicate with up computer, drive function 
void Step0(void);
void Step1(void);
void Step2(void);
void Step3(void);   // for receiving ITAC-OK or ITAC-NG
void Step4(void);   // for SN check
void Step6(void);   // for mode change
void Step8(void);
// communicate with up computer, app function
void DealAfterReceive(FUNCTION_T *FUNCITON); // Step0 Step1 Step2 step4
void DealAfterStop(FUNCTION_T *FUNCITON, time_t deadline);// Step3 
static int rcle_GetMessage(unsigned int address, char *data, int nbData);
// get left or right position for other c file like pdi.c and drive.c
POSITION_E *Position_Get(void)
{
	return &position;
}
TYPE_E	*Product_Type_Get(void)
{
	return &product_type;
}
void TIME2_Init()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_Init(&NVIC_InitStructure);  	
	pwm_cfg_t cfg;
	const pwm_bus_t *pwm = &pwm2;
	cfg.hz = 1000;
	cfg.fs = 100;
	pwm->init(&cfg);
}
void TIM2_IRQHandler(void)
{
	static int counter = 0;
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) {       //
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);     
		counter ++;
		if(counter == 10) {
			pdi_can_bus -> send(&can_msg2);
			pdi_can_bus -> send(&can_msg3);
		} else if(counter == 20) {
			pdi_can_bus -> send(&can_msg1);
			pdi_can_bus -> send(&can_msg4);
			counter = 0;
		}
	}
}
/*start session*/
int pdi_startsession(void)
{
	int msg_len;
	can_msg_t msg;
	can_msg_t rcle_reqseed_msg =		{ID, 8, {0x02, 0x27, 0xc1, 0, 0, 0, 0, 0}, 0};
	can_msg_t rcle_start_msg =			{ID, 8, {0x02, 0x10, 0xc1, 0, 0, 0, 0, 0}, 0};
	can_msg_t sendkey_msg = {ID, 8, {0x06, 0x27, 0xc2, 0xff, 0xff, 0xff, 0xff, 0},0}; // TBD By Me
	if(product_type == Geely) {
		rcle_reqseed_msg.id = 0x7e5;
		rcle_start_msg.id = 0x7e5;
		sendkey_msg.id = 0x7e5;
	} else if((product_type == GW1) || (product_type == GW2)){
		rcle_reqseed_msg.id = 0x76a;
		rcle_start_msg.id = 0x76a;
		sendkey_msg.id = 0x76a;
	} 
	int try_time_start = 5;
	do {
		usdt_GetDiagFirstFrame(&rcle_start_msg, 1, NULL, &msg, &msg_len);		//start session, why twice?	
		try_time_start --;
	}while((msg.data[1]!= 0x50) && (try_time_start != 0)); 
	if(try_time_start == 0) {
		return 1;
	}
#if PDI_DEBUG
	printf("111111\r\n");
	can_msg_print(&rcle_start_msg,"\n");
	can_msg_print(&msg, "\n");
#endif
	int try_time_reqseed = 5;
	do {
		usdt_GetDiagFirstFrame(&rcle_reqseed_msg, 1, NULL, &msg, &msg_len);		//req seed
		try_time_reqseed --;
	}while((msg.data[1]!= 0x67) && (try_time_reqseed != 0));
	if(try_time_reqseed == 0) {
		return 1;
	}	
#if PDI_DEBUG
	printf("222222\r\n");
	can_msg_print(&rcle_reqseed_msg,"\n");
	can_msg_print(&msg, "\n");
#endif
	unsigned int seed, result;
	seed = msg.data[6] + (msg.data[5] << 8) + (msg.data[4] << 16) + (msg.data[3] << 24);
	result = seed + 0xB1234c12;
	sendkey_msg.data[3] = (result & 0xFF000000) >> 24;
	sendkey_msg.data[4] = (result & 0xFF0000) >> 16;
	sendkey_msg.data[5] = (result & 0xFF00) >> 8;
	sendkey_msg.data[6] = (result & 0xFF);
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
	return 0;
}
//
int rcle_open_squence(void)
{
	char open[] = "!EBCD4TEST!";
	char a[256];
	memset(a,0,sizeof(a));
	int b = 0;
	int try_time_open = 5;
	do {
		kline_communicate(&pdi_kbus, open, a, &b);
		try_time_open --;
	}while((b == 0) && (try_time_open != 0));
	if(try_time_open == 0) {
		return 1;
	}else {
		return 0;
	}
}
//
int rcle_close_squence(void)
{
	char close[] = "!ACU4END!";
	char a[256];
	memset(a,0,sizeof(a));
	int b = 0;
	int try_time_close = 5;
	do {
		kline_communicate(&pdi_kbus, close, a, &b);
		try_time_close --;
	}while((b == 0) && (try_time_close != 0));
	if(try_time_close == 0) {
		return 1;
	}else {
		printf("%d\r\n",b);
		return 0;
	}
}
//
int rcle_erase_squence(void)
{
	char erase[] = "!ERASE_ERRORS!";
	char a[256];
	memset(a,0,sizeof(a));
	int b = 0;
	int try_time_erase = 5;
	rcle_open_squence();
	do {
		kline_communicate(&pdi_kbus, erase, a, &b);
		try_time_erase --;
	}while((b == 0) && (try_time_erase != 0));
	if(try_time_erase == 0) {
		return 1;
	}else {
		printf("%d\r\n",b);
		return 0;
	}
}
//
int rcle_kline_fault(char *data, int *num, int *num1)
{
	char error_squence_haima[] = "R:2010114C";	//!!! 100F9C
	char error_squence_gw3[] = "R:20100F9C";
	char error_squence_ck[] = "R:20100F9C";
	char error_squence_gc[] = "R:20100F9C";
	int try_time_error = 5;
	do {
		if(product_type == Haima) {
			kline_communicate(&pdi_kbus, error_squence_haima, data, num);
		} else if(product_type == GW3) {
			kline_communicate(&pdi_kbus, error_squence_gw3, data, num);
		} else if(product_type == CK) {
			kline_communicate(&pdi_kbus, error_squence_ck, data, num);
		} else if((product_type == GC) || (product_type == LHD) || (product_type == SL)) {
			kline_communicate(&pdi_kbus, error_squence_gc, data, num);
		}
		try_time_error --;
	} while((*num == 0) && (try_time_error != 0));
	if(try_time_error == 0) {
		return 1;
	}else {
		for(int i = 3; i < 3 + 2 * (*num); i += 4) {
			if((data[i] | data[i+1] | data[i+2] | data[i+3]) == '0')
				break;
			(*num1)++;
		}
		printf("num of fault is %d\r\n",(*num1));
		return 0;
	}
}
//
int rcle_read_lock(char *data, int *num)
{
	char lock_sequence_haima[] = "R:05008348";
	char lock_sequence_gw3[] = "R:05008304";
	char lock_sequence_ck[] = "R:05008304";
	char lock_sequence_gc[] = "R:05008304";
	int try_time_lock = 5;
	int rtn = 0;
	if(can_or_kline == KLINE) {
		do {
			rcle_open_squence();
			if(product_type == Haima) {
				kline_communicate(&pdi_kbus, lock_sequence_haima, data, num);
			} else if(product_type == GW3) {
				kline_communicate(&pdi_kbus, lock_sequence_gw3, data, num);
			} else if(product_type == CK) {
				kline_communicate(&pdi_kbus, lock_sequence_ck, data, num);
			} else if((product_type == GC) || (product_type == LHD) || (product_type == SL)) {
				kline_communicate(&pdi_kbus, lock_sequence_gc, data, num);
			}
			try_time_lock--;
		} while((*num == 0) && (try_time_lock != 0));
		if(try_time_lock == 0) {
			return 1;
		}
                rcle_close_squence();
	} else if(can_or_kline == CAN) {
		do {
			if(product_type == Geely) {
				int address = 0x008346;
				int length = 5;
				rtn = rcle_GetMessage(address,data,length);
			} else if(product_type == GW1) {
				int address = 0x008345;
				int length = 5;
				rtn = rcle_GetMessage(address,data,length);
			} else if(product_type == GW2) {
				int address = 0x00834D;
				int length = 5;
				rtn = rcle_GetMessage(address,data,length);
			} 
			try_time_lock--;
		} while((rtn == 1) && (try_time_lock != 0));
		if(try_time_lock == 0) {
			return 1;
		}
	}
	return 0;
}
//
ERROR_E TestLock()
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	int len = 0;
	if(can_or_kline == KLINE) {
		if(rcle_read_lock(rcle_fault_buf,&len)) {
			error_result = ERROR_CW;
			return error_result;
		}
		if(rcle_fault_buf[5] != 0x30) {
			error_result = ERROR_LOCK;
		} else {
			error_result = ERROR_NO;
                        printf("Lock oK\r\n");
		}
	} else if((can_or_kline == CAN)) {
		if(pdi_startsession()) {
			error_result = ERROR_CW;
			return error_result;
		}
		if(rcle_read_lock(rcle_fault_buf,&len)) {
			error_result = ERROR_CW;
			return error_result;
		}
                memset(rcle_fault_buf,0,sizeof(rcle_fault_buf));
		if((rcle_fault_buf[5] != 0x0) && (rcle_fault_buf[5] != 0x30)) {
			error_result = ERROR_LOCK;
		} else {
			error_result = ERROR_NO;
		}
	}
	return error_result;
}
int rcle_read_nvm(char *data, int *num, char *data1)
{
	char nvm1_sequence_haima[] = "R:10008303";
	char nvm2_sequence_haima[] = "R:0200830D";
	char nvm1_sequence_gw3[] = "R:100082BF"; //ch041 
	char nvm2_sequence_gw3[] = "R:020082C9";
	char nvm1_sequence_ck[] = "R:100082BF"; //ck
	char nvm2_sequence_ck[] = "R:020082C9";
	char nvm1_sequence_gc[] = "R:100082BF"; //gc
	char nvm2_sequence_gc[] = "R:020082C9";
	if(can_or_kline == KLINE) {
		rcle_open_squence();
		int try_time = 5;
		do {
			if(product_type == Haima) {
				kline_communicate(&pdi_kbus, nvm1_sequence_haima, data, num);
			} else if(product_type == GW3) {
				kline_communicate(&pdi_kbus, nvm1_sequence_gw3, data, num);
			} else if(product_type == CK) {
				kline_communicate(&pdi_kbus, nvm1_sequence_ck, data, num);
			} else if((product_type == GC) || (product_type == LHD) || (product_type == SL)) {
				kline_communicate(&pdi_kbus, nvm1_sequence_gc, data, num);
			}
			try_time--;
		} while((*num == 0) && (try_time != 0));
		if(try_time == 0) {
			return 1;
		} 
		for(int i = 3; i < 23; i += 2) {
			char a = data[i];
			char b = data[i+1];
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
			data1[(i - 3) / 2] = (a & 0x0f) << 4 | (b & 0x0f); 
			//printf("%c\t",data1[(i - 3) / 2]);
		}
		rcle_open_squence();
		try_time = 5;
		do {
			if(product_type == Haima) {
				kline_communicate(&pdi_kbus, nvm2_sequence_haima, data, num);
			} else if(product_type == GW3) {
				kline_communicate(&pdi_kbus, nvm2_sequence_gw3, data, num);
			} else if(product_type == CK) {
				kline_communicate(&pdi_kbus, nvm2_sequence_ck, data, num);
			} else if((product_type == GC) || (product_type == LHD) || (product_type == SL)) {
				kline_communicate(&pdi_kbus, nvm2_sequence_gc, data, num);
			}
			try_time--;
		} while((*num == 0) && (try_time != 0));
		if(try_time == 0) {
			return 1;
		}
		for(int i = 3; i < 7; i += 2) {
			char a = data[i];
			char b = data[i+1];
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
			data1[10 + (i - 3) / 2] = (a & 0x0f) << 4 | (b & 0x0f); 
			//printf("%c\t",data1[10 + (i - 3) / 2]);
		}
		//printf("\r\n!!!!\r\n");
	} else if((can_or_kline == CAN)) {
		int rtn = 0;
		int try_time = 5;
		pdi_startsession();
		do {
			if(product_type == Geely) {
				int address = 0x008308;
				int length = 10;
				rtn = rcle_GetMessage(address,data,length);
			} else if(product_type == GW1) {
				int address = 0x008308;
				int length = 10;
				rtn = rcle_GetMessage(address,data,length);
			} else if(product_type == GW2) {
				int address = 0x008308;
				int length = 10;
				rtn = rcle_GetMessage(address,data,length);
			} 
			try_time--;
		} while((rtn == 1) && (try_time != 0));
		if(try_time == 0) {
			return 1;
		} 
		for(int i = 0; i < 10;i ++) {
			data1[i] = data[i];
		}
		try_time = 5;
		pdi_startsession();
		do {
			if(product_type == Geely) {
				int address = 0x008312;
				int length = 2;
				rtn = rcle_GetMessage(address,data,length);
			} else if(product_type == GW1) {
				int address = 0x008312;
				int length = 2;
				rtn = rcle_GetMessage(address,data,length);
			} else if(product_type == GW2) {
				int address = 0x008312;
				int length = 2;
				rtn = rcle_GetMessage(address,data,length);
			} 
			try_time--;
		} while((rtn == 1) && (try_time != 0));
		if(try_time == 0) {
			return 1;
		} 
		for(int i = 0; i < 2;i ++) {
			data1[10 + i] = data[i];
		}
	}
	return 0;
}

//
ERROR_E TestNVM()
{
	if(error_result != ERROR_NO){
		return error_result;
	}
	int len = 0;
	char temp[length_NVM];
	memset(temp,0,length_NVM);
	if(rcle_read_nvm(rcle_fault_buf,&len,temp)) {
		error_result = ERROR_CW;
		return error_result;
	}
        rcle_close_squence();
	printf("read nvm from ecu %s\r\n",temp);
	printf("get nvm from upper %s\r\n",autoliv.NVM);
	if(memcmp(autoliv.NVM,temp,length_NVM)) {
		error_result = ERROR_NVM;
		return error_result;
        }
	error_result = ERROR_NO;
	return error_result;
}
//16 bit sn
#define HaimaSN_len 16
int rcle_kline_sn(char *data, int *num)
{
	char sn_squence[] = "R:1000834F";	//EEP_ADR_ERR_INHIBITION_P
	int try_time_sn = 5;
	do {
		kline_communicate(&pdi_kbus, sn_squence, data, num);
		try_time_sn --;
	} while((*num == 0) && (try_time_sn != 0));
	if(try_time_sn == 0) {
		return 1;
	}else {
		return 0;
	}
}

/*get contents of cid,saved to data*/
static int rcle_GetCID(char sid,short cid, char *data)
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
		rcle_msg_buf[0] = msg_res;
		if(usdt_GetDiagLeftFrame(rcle_msg_buf, msg_len))
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
			memcpy(data, (rcle_msg_buf + i)->data + 1, 7);
			data += 7;
		}
	}
	return 0;
}
static int rcle_GetMessage(unsigned int address, char *data, int nbData)
{
	can_msg_t msg_res,pdi_send_msg = {ID,8,{0,0,0,0,0,0,0,0},0};
	if(product_type == Geely) {
		pdi_send_msg.id = 0x7e5;
		pdi_send_msg.data[0] = 0x07;
		pdi_send_msg.data[1] = 0x23;
		// !!
		pdi_send_msg.data[2] = (char)((address & 0xFF000000) >> 24);
		pdi_send_msg.data[3] = (char)((address & 0xFF0000) >> 16);
		pdi_send_msg.data[4] = (char)((address & 0xFF00) >> 8);
		pdi_send_msg.data[5] = (char)(address & 0xFF);
		// !!
		pdi_send_msg.data[6] = (char)(nbData & 0xFF00) >> 8;
		pdi_send_msg.data[7] = (char)(nbData & 0xFF);
	} else if((product_type == GW1) || (product_type == GW2)) {
		pdi_send_msg.id = 0x76a;
		pdi_send_msg.data[0] = 0x07;
		pdi_send_msg.data[1] = 0x23;
		// !!
		pdi_send_msg.data[2] = (char)((address & 0xFF000000) >> 24);
		pdi_send_msg.data[3] = (char)((address & 0xFF0000) >> 16);
		pdi_send_msg.data[4] = (char)((address & 0xFF00) >> 8);
		pdi_send_msg.data[5] = (char)(address & 0xFF);
		// !!
		pdi_send_msg.data[6] = (char)(nbData & 0xFF00) >> 8;
		pdi_send_msg.data[7] = (char)(nbData & 0xFF);
	} 
	int i = 0, msg_len;
	printf("444444\r\n");
	can_msg_print(&pdi_send_msg,"\n");
	if (usdt_GetDiagFirstFrame(&pdi_send_msg, 1, NULL, &msg_res, &msg_len))
		return 1;
	can_msg_print(&msg_res, "\n");
	if (msg_len > 1) {
		rcle_msg_buf[0] = msg_res;
		if(usdt_GetDiagLeftFrame(rcle_msg_buf, msg_len))
			return 1;
	}
	if (msg_len == 1) {
		if (msg_res.data[1] == (pdi_send_msg.data[1] + 0x40)) {
			if((product_type == GW4))
				memcpy(data, (msg_res.data + 4), msg_res.data[0] - 3);
			else if((product_type == GW1)||(product_type == Geely)||(product_type == GW2))
				memcpy(data, (msg_res.data + 2), 2); //??
		} else
			return 1;
	} else if (msg_len > 1) {
		if((product_type == GW3)) {
			memcpy(data, (msg_res.data + 5), 3);
			data += 3;
		} else if((product_type == Geely) || (product_type == GW2) || (product_type == GW1)) {
			memcpy(data, (msg_res.data + 3), 5);
			data +=5;
		}
		for (i = 1; i < msg_len; i++) {
			memcpy(data, (rcle_msg_buf + i)->data + 1, 7);
			data += 7;
		}
	}
	return 0;
}
/* read barcode*/
// return 1 : no date
// retrun 0 : ok
static int rcle_read_barcode() 
{
	char read_bcode[Length_barcode + 1];
	int try_times = 5;
	while (rcle_GetCID(0x22,0xfd42, rcle_data_buf)) {
		try_times --;
		if (try_times < 0)
			return 1;
	}
	memcpy(read_bcode, rcle_data_buf, Length_barcode);
	printf("%s",read_bcode); // add '\0' in the end
	return 0;
}
/* checking rclercode */
// return 1 : no data; not equal
// return 0 : ok
static int rcle_check_barcode()
{
	char read_bcode[Length_barcode + 1];
	int try_times = 5;
	printf("##START##EC-Checking barcode...##END##\n");
	while (rcle_GetCID(0x22,0xfd42, rcle_data_buf)) {
		try_times --;
		if (try_times < 0)
			return 1;
	}
	memcpy(read_bcode, rcle_data_buf, Length_barcode);
	printf("##START##RB-");
	printf(read_bcode,'\0');
	printf("##END##\n");
	if(position == LEFT) {
		if (memcmp(read_bcode, bcodea, Length_barcode))  //maybe Length_rclercode - 1 ??
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
static int rcle_read_sn(char *data, int *num, char *data1)
{
	char sn_sequence_haima[] = "R:1400834F";
	char sn_sequence_gw3[] = "R:2000830B"; //ch041 
	char sn_sequence_ck[] = "R:2000830B"; //ck
	char sn_sequence_gc[] = "R:2000830B"; //gc
	if(can_or_kline == KLINE) {
		rcle_open_squence();
		int try_time = 5;
		do {
			if(product_type == Haima) {
				kline_communicate(&pdi_kbus, sn_sequence_haima, data, num);
			} else if(product_type == GW3) {
				kline_communicate(&pdi_kbus, sn_sequence_gw3, data, num);
			} else if(product_type == CK) {
				kline_communicate(&pdi_kbus, sn_sequence_ck, data, num);
			} else if((product_type == GC) || (product_type == LHD) || (product_type == SL)) {
				kline_communicate(&pdi_kbus, sn_sequence_gc, data, num);
			}
			try_time--;
		} while((*num == 0) && (try_time != 0));
		if(try_time == 0) {
			return 1;
		} 
		for(int i = 3; i < 39; i += 2) {
			char a = data[i];
			char b = data[i+1];
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
			data1[(i - 3) / 2] = (a & 0x0f) << 4 | (b & 0x0f); 
			//printf("%c\t",data1[(i - 3) / 2]);
		}
	        rcle_close_squence();
	} else if(can_or_kline == CAN) {
		int rtn = 0;
		int try_time = 5;
		pdi_startsession();
		do {
			if(product_type == Geely) {
				int address = 0x00834D;
				int length = 20;
				rtn = rcle_GetMessage(address,data,length);
			} else if(product_type == GW1) {	//8loop
				int address = 0x008354;
				int length = 20;
				rtn = rcle_GetMessage(address,data,length);
			} else if(product_type == GW2) {	//4loop
				int address = 0x008354;
				int length = 20;
				rtn = rcle_GetMessage(address,data,length);
			} 
			try_time--;
		} while((rtn == 1) && (try_time != 0));
		if(try_time == 0) {
			return 1;
		} 
		for(int i = 0; i < 18;i ++) {
			data1[i] = data[i];
		}
	}
	return 0;
}
/* equal Serial Number */
//return 1 : not equal
//return 0 : ok
static int rcle_check_sn()
{
	char read_sn[Length_SN + 1];
	int len;
	memset(rcle_data_buf,0,sizeof(rcle_data_buf));
	rcle_read_sn(rcle_data_buf,&len,read_sn);
	printf("SN Read from ECU : \r\n");
	printf("%s\r\n",read_sn);
	printf("Compare the SN :");
	if(memcmp(SN, read_sn, Length_SN))
		return 1;
	return 0;
}
ERROR_E TestSN(void)
{
	if(error_result != ERROR_NO) {
		return error_result;
	}
	printf("Start to check SN\r\n");
	if(rcle_check_sn()) {
		error_result = ERROR_SN;
	} else {
		error_result = ERROR_NO;
		printf("SN OK\r\n");
	}
        memset(SN,0,sizeof(SN));
	return error_result;
}
#define L 12
static int rcle_read_softversion()
{
	char read_softversion[L + 1];  // 12 means length
	int try_times = 5;
	memset(rcle_data_buf,0,sizeof(rcle_data_buf));
	while(rcle_GetCID(0x22,0xfd42,rcle_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	rcle_data_buf[L] = '\0';
	memcpy(read_softversion,rcle_data_buf,sizeof(read_softversion));
	printf("%s\r\n",read_softversion);
	return 0;
}
static int rcle_check_softversion()
{
	char read_softversion[11 + 1];  // 12 means length
	int try_times = 5;
	memset(rcle_data_buf,0,sizeof(rcle_data_buf));
	while(rcle_GetCID(0x22,0xfd42,rcle_data_buf)) {
		try_times --;
		if(try_times < 0) {
			return 1;
		}
	}
	rcle_data_buf[11] = '\0';
	memcpy(read_softversion,rcle_data_buf,sizeof(read_softversion));
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
	can_msg_t rcle_clrdtc_msg =		{ID, 8, {0x03, 0x22, 0xF1, 0x00, 0,0, 0, 0}, 0};
	if(product_type == Geely) {
		rcle_clrdtc_msg.id = 0x7e5;
	} else if((product_type == GW1) || (product_type == GW2)) {
		rcle_clrdtc_msg.id = 0x76a;
		rcle_clrdtc_msg.data[0] = 0x02;
		rcle_clrdtc_msg.data[1] = 0x3b;
		rcle_clrdtc_msg.data[2] = 0xf1;
	} 
	pdi_startsession();
	if (usdt_GetDiagFirstFrame(&rcle_clrdtc_msg, 1, NULL, &msg, &msg_len))
		return 1;
	printf("4444444\r\n");
	can_msg_print(&rcle_clrdtc_msg,"\n");
	can_msg_print(&msg, "\n");
	if (msg.data[1] != (rcle_clrdtc_msg.data[1] + 0x40))	// TBD By Me
		return 1;
	return 0;
}
/*get fault from ECU,saved to data,the nr of fault is pnum_fault*/
static int rcle_GetFault(char *data, int *pnum_fault)
{
	int i, result = 0;
	//if (rcle_GetCID(0x19,0x028b, data))
	//	return 1;
	int address = 0;
	int length = 0;
	if(product_type == Geely) { 
		address = 0x10101C;
		length = 40;
	} else if((product_type == GW1) || (product_type == GW2)) {
		//address = 0x100CE8;	//ERR_astErrorBuffer
		address = 0x0083f0;	//EEP_ADR_ERR_ERROR
		length = 56;
	} 
	if(rcle_GetMessage(address,data,length))
		return 1;
	//memset(data + 117, 0x00, 10);
	if(product_type == Geely) {
		for (i = 0; i < 117; i += 2) {
			if((data[i] | data[i+1]) == 0)
				break;
			if(data[i] == (char)(0xaa))    // maybe in null address, the data is 0xaa, to be confirmed
				break;
			result ++;
		}
		*pnum_fault = result;
	} else if(product_type == GW1) {
		for(int j = 16;j < 120; j += 4) {
			if((data[j] | data[j+1] | data[j+2] | data[j+3]) == 0)
				break;
			result ++;
		}
		*pnum_fault = result;
	} else if(product_type == GW2) {
		for(int j = 0;j < 120; j += 4) {
			if((data[j] | data[j+1] | data[j+2] | data[j+3]) == 0)
				break;
			result ++;
		}
		*pnum_fault = result;
	}
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
	FUNCTION[8] = Step8;	//as 0808 head
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
** pick up the rclercode ****************** 
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
	if(can_or_kline == KLINE) {
		pdi_kline();
	} else if(can_or_kline == CAN) {
		pdi_can();
	}								// set the can relay
	pdi_mdelay(50);
	pdi_set_relays(autoliv);              	// set the load relays, may exist bugs
	pdi_mdelay(50);
	pdi_IGN_on();                         	// set the power relay or switch
	error_result = ERROR_NO;
	return ERROR_NO;
}
/*******************************************
** Blink the yellow led ********************
** compare rclercode from ECU and scaner  **
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
		printf("##START##STATUS-");		// in such format, so the data can be displayed in processrcler
		sprintf(processbar, "%d", rate);
		printf("%s", processbar);
		printf("##END##\n");
	}
	/*pdi_startsession();                 // startsession before getting message
	if (rcle_check_barcode()) {
		error_result = ERROR_BW;
		return error_result;
	}
	printf("####EC-      Checking barcode Done...##END##\n");*/
	return ERROR_NO;
}
/**********************************************
** process running ************************
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
	memset(rcle_fault_buf,0,sizeof(rcle_fault_buf)); // clear this buffer, if not, you will see the bug
	int try_times = 0;
	int result = 0;
	int num_fault = 0;
	int num_fault1 = 0;
	/**** when result is 1, must get fault again, oversize 3 times *****
	**** num_fault is not 0, check the error_code ********************** 
	*******************************************************************/
	if(can_or_kline == KLINE) {
		if(rcle_open_squence()) {
			error_result = ERROR_CW;
			return error_result;
		}
		if(rcle_kline_fault(rcle_fault_buf, &num_fault, &num_fault1)) {
			error_result = ERROR_CW;
			return error_result;
		}
		//:nn+ddddd...+cc\r\n>
		if(!num_fault1) {
			if(product_type == Haima) {
				error_result = ERROR_CURRENT;
				return error_result;
			}
			error_result = ERROR_NO;
			return error_result;
		} else {
			printf("##START##EC-");
			printf("num of fault is: %d\n", num_fault1);
			//two bytes into one byte
			char temp[50];
			char senderrorcode[70];
			if((num_fault1 == 1) && (rcle_fault_buf[3] == 'B') && (rcle_fault_buf[4] == '9')) {
				error_result = ERROR_NO;
				return error_result;
			}
			memset(temp, 0, sizeof(temp));
			for(int i = 0; i < 2 * num_fault; i++) {
				char a = rcle_fault_buf[3 + i];
				senderrorcode[i] = a;
				char b = rcle_fault_buf[4 + i];
				senderrorcode[i + 1] = b;
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
				temp[(i + 1) / 2] = a << 4 | b;
			}
			for(int i = 0; i < num_fault1 * 2; i += 2) {
				printf("0x%x	0x%x\r\n", temp[i], temp[i + 1]);
			}
			printf("##END##\n");
			if(pdi_error_check(temp, &num_fault1, autoliv)) { // check whether the error code is allowable
				error_result = ERROR_CODE;
				autoliv_send(&pdi_autoliv,"ERROR\r\n ");  //new protocol add by hyf at shanghai 2014/5/15  ' 'for the upper's convinence
				pdi_mdelay(500);
				for(int i = 0; i < num_fault1 * 4; i++){   //send errorcode to up
					autoliv_send_char(&pdi_autoliv, senderrorcode[i]);
				}
				autoliv_send(&pdi_autoliv,"\r\n");		
				if(debug_or_product == DEBUG){                       //clear the historical error                                                        
					//must have,or cannot clear the historical error 
					for (int rate = 17; rate <= 96; rate ++) {
						pdi_mdelay(89);
					}
					pdi_IGN_on();			
					if(can_or_kline == CAN) {
						pdi_can();
						pdi_clear_dtc();
					} else {
						pdi_kline();
						rcle_open_squence();
						if(rcle_erase_squence()) {
							printf("erase fail\r\n");
						} else {
							printf("erase ok\r\n");
						}
					}
				}
			}
			else {
				error_result = ERROR_ENCODE;
			}
			return error_result;
		}
	}else {
		pdi_startsession();
		do{
			result = rcle_GetFault(rcle_fault_buf,&num_fault);
			if(++try_times == 4) {
				error_result = ERROR_CW;
				return error_result;
			}
		} while(result == 1);
		if(!num_fault) {
			if(product_type == Geely) {
				error_result = ERROR_CURRENT;
				return error_result;
			}
			error_result = ERROR_NO;
			return error_result;
		} else {
			printf("##START##EC-");
			if(pdi_error_check(rcle_fault_buf, &num_fault, autoliv)) { // check whether the error code is allowable
				error_result = ERROR_CODE;
				autoliv_send(&pdi_autoliv,"ERROR\r\n");
				pdi_mdelay(500);
				if((product_type == Geely)) {
					printf("##OK##\n");
					printf("num of fault is: %d\n", num_fault);
					if((num_fault == 1) && (rcle_fault_buf[0] == 0xb6) && (rcle_fault_buf[1] == 0x10)) {
						error_result = ERROR_NO;
						autoliv_send(&pdi_autoliv,"\r\n");
						return error_result;
					}
					for(int i = 0; i < num_fault*2; i += 2) {
						printf("0x%02x,0x%02x\n", rcle_fault_buf[i]&0xff,
						rcle_fault_buf[i+1]&0xff);
					}
					for(int i = 0; i < num_fault * 2; i ++) {
						char a , b;
						a = rcle_fault_buf[i] >> 4;
						b = rcle_fault_buf[i] & 0x0f;
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
				} else if(product_type == GW1) {
					printf("##OK##\n");
					printf("num of fault is: %d\n", num_fault);
					for(int i = 16; i < 16 + num_fault*4; i += 4) {
						printf("0x%02x,0x%02x,0x%02x,0x%02x\n", rcle_fault_buf[i]&0xff,
						rcle_fault_buf[i+1]&0xff,rcle_fault_buf[i+2]&0xff,rcle_fault_buf[i+3]&0xff);
					}
					for(int i = 16; i < 16 + num_fault * 4; i ++) {
						char a , b;
						a = rcle_fault_buf[i] >> 4;
						b = rcle_fault_buf[i] & 0x0f;
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
				} else if(product_type == GW2) {
					printf("##OK##\n");
					printf("num of fault is: %d\n", num_fault);
					for(int i = 0; i < num_fault*4; i += 4) {
						printf("0x%02x,0x%02x,0x%02x,0x%02x\n", rcle_fault_buf[i]&0xff,
						rcle_fault_buf[i+1]&0xff,rcle_fault_buf[i+2]&0xff,rcle_fault_buf[i+3]&0xff);
					}
					for(int i = 0; i < num_fault * 4; i ++) {
						char a , b;
						a = rcle_fault_buf[i] >> 4;
						b = rcle_fault_buf[i] & 0x0f;
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
				}
				autoliv_send(&pdi_autoliv,"\r\n");	
				if(debug_or_product == DEBUG){                       //clear the historical error                                                        
					//must have,or cannot clear the historical error 
					for (int rate = 17; rate <= 96; rate ++) {
						pdi_mdelay(89);
					}
					pdi_IGN_on();			
					if(can_or_kline == CAN) {
						pdi_can();
						pdi_clear_dtc();
					} else {
						pdi_kline();
						if(rcle_erase_squence()) {
							printf("erase fail\r\n");
						} else {
							printf("erase ok\r\n");
						}
					}
				}
			} else {
				error_result = ERROR_ENCODE;
			}
			return error_result;
		}    
	}
}
/********************************************************
********After Power off, check the can communication*****
*********************************************************/
void Poweroff()
{	
	pdi_IGN_off();
	if(can_or_kline == CAN) {
		can_msg_t rcle_start_msg =			{ID, 8, {0x02, 0x10, 0xc1, 0, 0, 0, 0, 0}, 0};
		if(product_type == Geely) {
			rcle_start_msg.id = 0x7e5;
		} else if(product_type == GW1) {
			rcle_start_msg.id = 0x76a;
		}
		int msg_len;
		can_msg_t msg;	
		do {                                // if no extern CAN
			pdi_mdelay(1000);
		} while(!usdt_GetDiagFirstFrame(&rcle_start_msg, 1, NULL, &msg, &msg_len));
	} else {
		pdi_mdelay(1000);
	} while(!rcle_open_squence());
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
				case ERROR_NVM:
						printf("##START##EC-NVM A Wrong##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
						autoliv_send_char(&pdi_autoliv, 33);
						autoliv_send(&pdi_autoliv,"\r\n");
						printf("UUTA-FUNC-NG33\r\n");
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
				case ERROR_CW1:
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
				case ERROR_JAMA:
						printf("##START##STATUS-Get A JAMA error!##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
						autoliv_send_char(&pdi_autoliv, 21);
						autoliv_send(&pdi_autoliv,"\r\n");
						printf("UUTA-FUNC-NG21\r\n");
						break;
				case ERROR_LOCK:
						printf("##START##STATUS-Get A LOCK error!##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG");
						autoliv_send_char(&pdi_autoliv, 31);
						autoliv_send(&pdi_autoliv,"\r\n");
						printf("UUTA-FUNC-NG31\r\n");
						break;	
				case ERROR_ENCODE:
						printf("##START##EC-These faults are acceptale...##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-OK0\r\n");
						printf("UUTA-FUNC-OK\r\n");
						break;
				case ERROR_CURRENT:
						printf("##START##STATUS-Get A current error!##END##\n");
						autoliv_send(&pdi_autoliv,"UUTA-FUNC-NG18\r\n");
						printf("UUTA-FUNC-NG18\r\n");
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
					case ERROR_NVM:
						printf("##START##EC-NVM B Wrong##END##\n");
						autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
						autoliv_send_char(&pdi_autoliv, 33);
						autoliv_send(&pdi_autoliv,"\r\n");
						printf("UUTB-FUNC-NG33\r\n");
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
					case ERROR_CW2:
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
					case ERROR_JAMA:
						printf("##START##STATUS-Get B JAMA error!##END##\n");
						autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
						autoliv_send_char(&pdi_autoliv, 21);
						autoliv_send(&pdi_autoliv,"\r\n");
						printf("UUTB-FUNC-NG21\r\n");
						break;
					case ERROR_LOCK:
						printf("##START##STATUS-Get B LOCK error!##END##\n");
						autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG");
						autoliv_send_char(&pdi_autoliv, 31);
						autoliv_send(&pdi_autoliv,"\r\n");
						printf("UUTB-FUNC-NG31\r\n");
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
				case ERROR_CURRENT:
						printf("##START##STATUS-Get B current error!##END##\n");
						autoliv_send(&pdi_autoliv,"UUTB-FUNC-NG18\r\n");
						printf("UUTB-FUNC-NG18\r\n");
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
	memset(autoliv_buf,0,sizeof(autoliv_buf));   // Global variable attention!!!
	/*time_t over_time;
	over_time = time_get(dead_time);
	while(time_left(over_time) > 0) {
		if(!autoliv_get(&pdi_autoliv,autoliv_buf)) { // All the receive data put in autoliv_buf
			if (autoliv_buf[2] == 'I') {			 // In the first function 
				FUNCTION[0]();                       // Receiving the "0303ITAC-OK or NG\r\n", retrun
				return ;
			}
		}
	}
	printf("ITAC Overtime \r\n");*/
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
	autoliv_send(&pdi_autoliv,"ECUPDIIDLT006\r\n");  // Send to Up
	printf("%s\r\n","ECUPDILT006");
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
	// add by hyf at 2014/5/14  length_errorcode
	autoliv.length_errorcode = autoliv_buf[6 + length_bcode_part + length_relay + autoliv.length_softversion ] - '0'; 
	printf("Rx:errorcode length is %d\r\n",autoliv.length_errorcode); 
	printf("Rx:errorable:\r\n");
	// add length_errorcode
	for(int i = 0; i < autoliv.length_errorcode; i++) {
		autoliv.errorable[i] = autoliv_buf[i + 5 + length_bcode_part + length_relay + autoliv.length_softversion + 1 + 1];
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
	autoliv_send(&pdi_autoliv,"DL-CONF-EXT\r\n");
	printf("DL-CONF-EXT\r\n");
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
/**************************************************************
**** Up: Send "0808+ext\r\n"*******************
**** Down: Receive this string ,take the corresponding action**
**************************************************************/
//add by hyf at shanghai 2014/5/23 new protocol
void Step8(void)
{
	printf("******888******\r\n");
	if((autoliv_buf[2] == 'A') && (autoliv_buf[3] == 'B') && (autoliv_buf[4] == '0') && (autoliv_buf[5] == '1')) {
		printf("This product is AB01,belong to Haima,bus is Kline\r\n");
		can_or_kline = KLINE;
		product_type = Haima;
	} else if((autoliv_buf[2] == 'G') && (autoliv_buf[3] == 'W') && (autoliv_buf[4] == '1')) {
		printf("This prodcut is CHB, GW1,bus is can\r\n");
		can_or_kline = CAN;
		product_type = GW1;
	} else if((autoliv_buf[2] == 'G') && (autoliv_buf[3] == 'W') && (autoliv_buf[4] == '3')) {
		printf("This prodcut is CHX, GW3,bus is kline\r\n");
		can_or_kline = KLINE;
		product_type = GW3;
	} else if((autoliv_buf[2] == 'G') && (autoliv_buf[3] == 'C')) {
		printf("This product is GC, bus is kline\r\n");
		can_or_kline = KLINE;
		product_type = GC;
	} else if((autoliv_buf[2] == 'C') && (autoliv_buf[3] == 'K')) {
		printf("This product is GK, bus is kline\r\n");
		can_or_kline = KLINE;
		product_type = CK;
	} else if((autoliv_buf[2] == 'L') && (autoliv_buf[3] == 'H') && (autoliv_buf[4] == 'D')) {
		printf("This product is LHD, bus is kline\r\n");
		can_or_kline = KLINE;
		product_type = LHD;
	} else if((autoliv_buf[2] == 'S') && (autoliv_buf[3] == 'L')) {
		printf("This product is SL, bus is kline\r\n");
		can_or_kline = KLINE;
		product_type = SL;
	} else if((autoliv_buf[2] == 'G') && (autoliv_buf[3] == 'E') && (autoliv_buf[4] == 'E') && (autoliv_buf[5] == 'L')) {
		printf("This product is Geely, bus is can\r\n");
		can_or_kline = CAN;
		product_type = Geely;
	}
	for(int i = 0; i < length_NVM; i++) {
		autoliv.NVM[i] = autoliv_buf[i + 6];
	}
	autoliv_send(&pdi_autoliv,"DL-CONF-OK\r\n");
	printf("DL-CONF-OK\r\n");
}
// add by jiamao.gu in Shanghai 2013.10.28
// rclercode sended after teststop in about three seconds, so need to wait 
// after receiving barcode, SN sended in about five seconds, so need to wait 
// maybe change over_time in different condition !!!
void Receive_from_Up(time_t over_time)
{
	while(time_left(over_time) > 0) {
		if(SN[0] != 0) { // All the receive data put in autoliv_buf			
			printf("Receive other rclercode and SN success !\r\n");
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
	kline_init(&pdi_kbus);
	pdi_can_bus->init(&cfg_pdi_can);	// init can
	usdt_Init(pdi_can_bus);				// init protocol
	pdi_drv_Init();						// led,beep
	pre_check_action();					// flash the green1 red1 green2 red2 and beep
	config_init(autoliv);               // init the Config file like relay and limit file 
	pdi_IGN_off();                      // power off the left position
	position = RIGHT;
	pdi_IGN_off();                      // power off the right position
	position = LEFT;
	TIME2_Init();
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
	error_result = ERROR_NO;
	uart3.putchar('!');
	/*check process*/
	while(1) {
		sys_update();
		// SlotA == READY : target on
		// SN[0] !=0 : receive the SN
		// SlotB != COMPLETE : Product B has been tested
		if((SlotA == READY) && SN[0] != 0 && (SlotB != COMPLETE) && (position == LEFT)) {
			TestStart();
			Testbarcode();
			TestSN();
			TestLock();
			TestNVM();
			TestECU();
			TestStop();
			DealAfterStop(Function,3000);          // for upload to itac, overtime is 3000 ms
			error_result = ERROR_NO;               // set default condition
			SlotA = COMPLETE;
			position = RIGHT;
			printf("##START##STATUS-100##END##\n");
			memset(bcodea,0,sizeof(bcodea));
			//memset(SN,0,sizeof(SN));
		}
		if((SlotB == READY) && SN[0] != 0 && (SlotA != COMPLETE) && (position == RIGHT)) {
			TestStart();
			Testbarcode();
			TestSN();
			TestLock();
			TestNVM();
			TestECU();
			TestStop();
			DealAfterStop(Function,3000);
			error_result = ERROR_NO;               // set default condition
			SlotB = COMPLETE;
			position = LEFT;
			printf("##START##STATUS-100##END##\n");
			memset(bcodeb,0,sizeof(bcodeb));
			//memset(SN,0,sizeof(SN));			
		}				
	}
}

#if 1
static int cmd_rcle_func(int argc, char *argv[])
{
	const char *usage = {
		"rcle , usage:\r\n"
		"rcle fault		read the error code\r\n"
		"rcle barcode\r\n"
		"rcle ecu\r\n"
		"rcle sn          read the sn\r\n"
		"rcle nvm\r\n"
		"rcle clear       clear the history error\r\n"
		"rcle read lock	\r\n"
		"rcle lock x      lock the fixture x\r\n"
		"rcle unlock x    unlock the fixture x\r\n"
		"rcle printf relay file \r\n"
		"rcle get position  \r\n"
		"rcle set position x  left or right\r\n"
		"rcle set relay \r\n"
		"rcle set kline or can \r\n"
		"rcle print bus \r\n"
	};

	if (argc < 2) {
		printf("%s", usage);
		return 0;
	}

	if (argc == 2) {
		if(argv[1][0] == 'f') {
			char buffer[100];
			memset(buffer,0,sizeof(buffer));
			int len = 0;
			int len1 = 0;
			pdi_IGN_on();
			if(can_or_kline == KLINE) {
				pdi_kline();
				for (int rate = 17; rate <= 96; rate ++) {
					pdi_mdelay(89);
				}
				if(rcle_open_squence())
					return 1;
				if(rcle_kline_fault(buffer,&len,&len1))
					return 1;
				char buffer_1[100];
				memset(buffer_1, 0, sizeof(buffer_1));
				for(int i = 0; i < 2 * len; i += 2) {
					char a = buffer[3 + i];
					char b = buffer[4 + i];
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
					buffer_1[(i + 1) / 2] = a << 4 | b;
				}
				for(int i = 0; i < len1 * 2; i += 2) {
					printf("0x%x	0x%x\r\n", buffer_1[i], buffer_1[i+1]);
				}
			} else if(can_or_kline == CAN){
				pdi_can();
				for (int rate = 17; rate <= 96; rate ++) {
					pdi_mdelay(89);				
				}
				pdi_startsession();
				if(rcle_GetFault(rcle_fault_buf, &len1))
					printf("##ERROR##\n");
				else{
					if((product_type == Geely)) {
						printf("##OK##\n");
						printf("num of fault is: %d\n", len1);
						for(int i = 0; i < len1*2; i += 2)
							printf("0x%02x,0x%02x\n", rcle_fault_buf[i]&0xff,
							rcle_fault_buf[i+1]&0xff);
					} else if(product_type == GW1) {
						printf("##OK##\n");
						printf("num of fault is: %d\n", len1);
						for(int i = 16; i < 16 + len1*4; i += 4)
							printf("0x%02x,0x%02x,0x%02x,0x%02x\n", rcle_fault_buf[i]&0xff,
							rcle_fault_buf[i+1]&0xff,rcle_fault_buf[i+2]&0xff,rcle_fault_buf[i+3]&0xff);
					} else if(product_type == GW2) {
						printf("##OK##\n");
						printf("num of fault is: %d\n", len1);
						for(int i = 0; i < len1*4; i += 4)
							printf("0x%02x,0x%02x,0x%02x,0x%02x\n", rcle_fault_buf[i]&0xff,
							rcle_fault_buf[i+1]&0xff,rcle_fault_buf[i+2]&0xff,rcle_fault_buf[i+3]&0xff);
					}
				}			
			}
			pdi_IGN_off();
		}
		if(!strcmp(argv[1],"nvm")) {
			char buffer[100];
			char buffer1[100];
			memset(buffer,0,sizeof(buffer));
			int len = 0;
			pdi_IGN_on();
			if(can_or_kline == KLINE) {
				pdi_kline();
				for (int rate = 17; rate <= 96; rate ++) {
					pdi_mdelay(89);				
				}
				if(!rcle_open_squence()) {
					rcle_read_nvm(buffer,&len,buffer1);
					for(int i = 0; i < 12; i++) {
						printf("%c",buffer1[i]);
					}
				}
			} else if(can_or_kline == CAN) {
				pdi_can();
				for (int rate = 17; rate <= 96; rate ++) {
					pdi_mdelay(89);				
				}
				pdi_startsession();
				rcle_read_nvm(buffer,&len,buffer1);
				for(int i = 0 ; i < 12; i ++) {
					printf("%c\r\n",buffer1[i]);
				}
			}
		}
		if(!strcmp(argv[1],"ecu")) {
			pdi_IGN_on();
			pdi_kline();
			TestECU();
		}
		if(!strcmp(argv[1],"clear")) {
			pdi_IGN_on();			
			if(can_or_kline == CAN) {
				pdi_can();
				for (int rate = 1; rate <= 16; rate ++) {
					pdi_mdelay(89);				
				}
				if(pdi_clear_dtc()) {
					printf("clear fail\r\n");
				} else {
					printf("clear ok\r\n");
				}
			} else {
				pdi_kline();
				for (int rate = 1; rate <= 16; rate ++) {
					pdi_mdelay(89);				
				}
				rcle_open_squence();
				if(rcle_erase_squence()) {
					printf("erase fail\r\n");
				} else {
					printf("erase ok\r\n");
				}
			}
			pdi_IGN_off();
		}
		if(!strcmp(argv[1],"sn")) {
			char buffer[100];
			char buffer_sn[18];
			memset(buffer,0,sizeof(buffer));
			memset(buffer_sn,0,sizeof(buffer_sn));
			int len = 0;
			pdi_IGN_on();
			if(can_or_kline == KLINE) {
				pdi_kline();
				for (int rate = 17; rate <= 96; rate ++) {
					pdi_mdelay(89);
				}
				rcle_open_squence();
			} else {
				pdi_can();
				for (int rate = 17; rate <= 96; rate ++) {
					pdi_mdelay(89);
				}
				pdi_startsession();
			}
			rcle_read_sn(buffer,&len,buffer_sn);
			for(int i = 0; i < 18; i++) {
				printf("%x",buffer_sn[i]);
			}
			printf("\r\n");
			pdi_IGN_off();
		}
		if(!strcmp(argv[1],"start")) {
			pdi_IGN_on();	
			if(can_or_kline == CAN) {
				pdi_can();
				pdi_startsession();
			} else {
				pdi_kline();
				if(rcle_open_squence()) {
					printf("open fail\r\n");
				} else {
					printf("open ok\r\n");
				}
				if(rcle_close_squence()) {
					printf("close fail\r\n");
				} else {
					printf("close ok!\r\n");
				}
			}
		}
		if(!strcmp(argv[1],"target")) {
			if(!target2_on()) {
				printf("RIGHT TARGET OK \r\n");
			} else {
				printf("FAIL\r\n");
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
	if(argc == 3) {
		if(!strcmp(argv[1],"set") && !strcmp(argv[2],"kline")) {
			can_or_kline = KLINE;
			printf("set kline bus ok!\r\n");
		}
		if(!strcmp(argv[1],"set") && !strcmp(argv[2],"can")) {
			can_or_kline = CAN;
			printf("set can bus ok!\r\n");
		}
		if(!strcmp(argv[1],"print") && !strcmp(argv[2],"bus")) {
			if(can_or_kline == CAN) {
				printf("Bus is CAN!\r\n");
			} else if(can_or_kline == KLINE) {
				printf("Bus is KLINE!\r\n");
			}
		}
		if(!strcmp(argv[1],"read") && !strcmp(argv[2],"lock")) {
			char buffer[100];
			int len = 0;
			pdi_IGN_on();
			if(can_or_kline == KLINE) {
				pdi_kline();
				for (int rate = 17; rate <= 96; rate ++) {
					pdi_mdelay(89);				
				}
				if(!rcle_open_squence()) {
					rcle_read_lock(buffer, &len);
				}
				for(int i = 0 ; i < 10; i ++) {
					printf("%x\r\n",buffer[i]);
				}
			} else if(can_or_kline == CAN) {
				pdi_can();
				for (int rate = 17; rate <= 96; rate ++) {
					pdi_mdelay(89);				
				}
				pdi_startsession();
				rcle_read_lock(buffer,&len);
				for(int i = 0 ; i < 10; i ++) {
					printf("%x\r\n",buffer[i]);
				}
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
		if((!strcmp(argv[1],"set")) && (!strcmp(argv[2],"type"))) {
			if(!strcmp(argv[3],"haima")) {
				can_or_kline = KLINE;
				product_type = Haima;
				printf("haima\r\n");
			} else if(!strcmp(argv[3],"geely")) {
				can_or_kline = CAN;
				product_type = Geely;
				printf("geely\r\n");
			} else if(!strcmp(argv[3],"gw1")) {
				can_or_kline = CAN;
				product_type = GW1;
				printf("gw1 CHB011\r\n");
			} else if(!strcmp(argv[3],"gw2")) {
				can_or_kline = CAN;
				product_type = GW2;
				printf("gw2 CHB021\r\n");
			} else if(!strcmp(argv[3],"gw3")) {
				can_or_kline = KLINE;
				product_type = GW3;
				printf("gw3\r\n");
			} else if(!strcmp(argv[3],"gc")) {
				can_or_kline = KLINE;
				product_type = GC;
				printf("gc\r\n");
			} else if(!strcmp(argv[3],"ck")) {
				can_or_kline = KLINE;
				product_type = CK;
				printf("ck\r\n");
			} else if(!strcmp(argv[3],"lhd")) {
				can_or_kline = KLINE;
				product_type = LHD;
				printf("lhd\r\n");
			} else if(!strcmp(argv[3],"sl")) {
				can_or_kline = KLINE;
				product_type = SL;
				printf("sl\r\n");
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
const cmd_t cmd_rcle = {"rcle", cmd_rcle_func, "rcle cmd i/f"};
DECLARE_SHELL_CMD(cmd_rcle)
#endif