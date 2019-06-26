/*
 *	David peng.guo@2011 initial version
 *      jiamao.gu@2013 5.29 change
 *      the length of barcode must larger one byte than the SN: this one time, Max3232 5V is forget, 12V is forget too
 *      SN is the barcode!!!!
 *      add filter in usdt.c 
 *      add gpio config in drv.c and usdt.c like CONFIG_PDI_M16_gujiamao 
 *      Read error code: buffer must clear before reading and if read 0 0 likewise, break the function
 *      in limit file , error expected must captical, even if display in little mode
 */

#include <string.h>
#include "config.h"
#include "sys/task.h"
#include "ulp_time.h"
#include "can.h"
#include "drv.h"
#include "ls1203.h"
#include "cfg.h"
#include "priv/usdt.h"
#include "ulp/debug.h"
#include "shell/cmd.h"
#include "pdi.h"
#include "stdio.h"


#define PDI_DEBUG	1

//pdi M16 can msg
static const can_msg_t m16_clrdtc_msg =		{0x746, 8, {0x04, 0x2e, 0xfe, 0x90, 0xaa, 0, 0, 0}, 0};
static const can_msg_t m16_resis =		{0x746, 8,  {0x03, 0x22, 0xFD, 0x81, 0, 0, 0, 0}, 0};
#if PDI_DEBUG
static const can_msg_t m16_errcode_msg =	{0x746, 8, {0x03, 0x22, 0xfe, 0x80, 0, 0, 0, 0}, 0};
static const can_msg_t m16_connector_msg =	{0x746, 8, {0x03, 0x22, 0xfd, 0xa2, 0, 0, 0, 0}, 0};
static const can_msg_t m16_getsn_msg =		{0x746, 8, {0x03, 0x22, 0xfe, 0x8d, 0, 0, 0, 0}, 0};
#endif
static const can_msg_t m16_getpart_msg =	{0x746, 8, {0x03, 0x22, 0xfe, 0x8b, 0, 0, 0, 0}, 0}; // soft version
static const can_msg_t m16_reqseed_msg =	{0x746, 8, {0x02, 0x27, 0x7D, 0, 0, 0, 0, 0}, 0};   //0x7d meas access mode
static const can_msg_t m16_start_msg =		{0x746, 8, {0x02, 0x10, 0x65, 0, 0, 0, 0, 0}, 0};  // 0x10 0x65 means diagnostic mode

static const ls1203_t pdi_ls1203 = {
		.bus = &uart2,
		.data_len = 15,
		.dead_time = 20,
};

static const can_bus_t* pdi_can_bus = &can1;
static can_msg_t m16_msg_buf[32];		//for multi frame buffer
static char m16_data_buf[256];			//data buffer
static char m16_fault_buf[64];			//fault buffer
static char bcode_1[16];

//get data from ECU
static int m16_GetCID(short cid, char *data);
static int m16_GetFault(char *data, int * pnum_fault);

/**************************************************************************/
/************         Local funcitons                         *************/
/**************************************************************************/

/*delay ms(ms)*/
int pdi_mdelay(int ms)
{
	int left;
	time_t deadline = time_get(ms);
	do {
		left = time_left(deadline);
		if(left >= 10) { //system update period is expected less than 10ms
			ulp_update();
		}
	} while(left > 0);

	return 0;
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
	can_msg_t sendkey_msg = {0x746, 8, {0x04, 0x27, 0x7E, 0xff, 0xff, 0, 0, 0}, 0}; //TBD

	if (usdt_GetDiagFirstFrame(&m16_start_msg, 1, NULL, &msg, &msg_len))		//start session
		return 1;
#if PDI_DEBUG
	can_msg_print(&msg, "\n");
#endif
        
        if (usdt_GetDiagFirstFrame(&m16_start_msg, 1, NULL, &msg, &msg_len))		//start session
		return 1;
#if PDI_DEBUG
	can_msg_print(&msg, "\n");
#endif
        
	if (usdt_GetDiagFirstFrame(&m16_reqseed_msg, 1, NULL, &msg, &msg_len))	//req seed
		return 1;
#if PDI_DEBUG
	can_msg_print(&msg, "\n");
#endif

	/*calculate the key from seed*/
	seed[0] = (unsigned char)msg.data[3];
	seed[1] = (unsigned char)msg.data[4];
	result = seed[0] ^ seed[1];
	result ^= 0x23;  // 0x34
	result ^= 0x9a;  //0xab
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

//#if PDI_DEBUG
#if 1
	/*get serial number*/
	printf("\nSN Code:\n");
	usdt_GetDiagFirstFrame(&m16_getsn_msg, 1, NULL, m16_msg_buf, &msg_len);
	if (msg_len > 1)
		usdt_GetDiagLeftFrame(m16_msg_buf, msg_len);
	for (i = 0; i < msg_len; i ++)
		can_msg_print(m16_msg_buf + i, "\n");

	/*get error code*/
	printf("\nError Code:\n");
	usdt_GetDiagFirstFrame(&m16_errcode_msg, 1, NULL, m16_msg_buf, &msg_len);
	if (msg_len > 1)
		usdt_GetDiagLeftFrame(m16_msg_buf, msg_len);
	for (i = 0; i < msg_len; i++)
		can_msg_print(m16_msg_buf + i, "\n");
	/*tester point*/
	printf("\nPart:\n");
	usdt_GetDiagFirstFrame(&m16_getpart_msg, 1, NULL, m16_msg_buf, &msg_len);
	if (msg_len > 1)
		usdt_GetDiagLeftFrame(m16_msg_buf, msg_len);
	for (i = 0; i < msg_len; i++)
		can_msg_print(m16_msg_buf + i, "\n");

	if (m16_GetFault(m16_fault_buf, &num_fault))
		printf("##ERROR##\n");
	else {
		printf("##OK##\n");
		printf("num of fault is: %d\n", num_fault);
		for (i = 0; i < num_fault*2; i += 2)
			printf("0x%02x, 0x%02x\n", m16_fault_buf[i]&0xff, m16_fault_buf[i+1]&0xff);
	}
	/*clear all*/
	//printf("\nClear all:\n");
	//if (usdt_GetDiagFirstFrame(&m16_clrdtc_msg, 1, NULL, &msg, &msg_len))	//req seed
	//	return 1;
	//can_msg_print(&msg, "\n");
#endif

	return 0;
}
static int m16_error_check(char *fault, int *num, const struct pdi_cfg_s *sr)
{
	int i, j, flag;
	char m16_fault_temp[2];
	const struct pdi_rule_s* pdi_cfg_rule;
	for(i = 0; i < *num; i ++) {
		flag = 0;
		m16_fault_temp[0] = fault[i*2];
		m16_fault_temp[1] = fault[(i*2) + 1];
		for(j = 0; j < sr -> nr_of_rules; j ++) {
			pdi_cfg_rule = pdi_rule_get(sr, j);
			if(&pdi_cfg_rule == NULL) return 1;
			switch(pdi_cfg_rule -> type) {
				case PDI_RULE_ERROR:
					if(pdi_verify(pdi_cfg_rule, m16_fault_temp) == 0) {
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

/*get contents of cid,saved to data*/
static int m16_GetCID(short cid, char *data)
{
	can_msg_t msg_res, pdi_send_msg = {0x746, 8, {0x03, 0x22, 0, 0, 0, 0, 0, 0}, 0};
	int i = 0, msg_len;

	pdi_send_msg.data[2] = (char)(cid >> 8);
	pdi_send_msg.data[3] = (char)(cid & 0x00ff);
	if (usdt_GetDiagFirstFrame(&pdi_send_msg, 1, NULL, &msg_res, &msg_len))
		return 1;
        can_msg_print(&msg_res, "\n");
	if (msg_len > 1) {
		m16_msg_buf[0] = msg_res;
		if(usdt_GetDiagLeftFrame(m16_msg_buf, msg_len))
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
			memcpy(data, (m16_msg_buf + i)->data + 1, 7);
			data += 7;
		}
	}

	return 0;
}

/*checking JAMA*/
static int m16_check_JAMA(const struct pdi_cfg_s *sr)
{
	const struct pdi_rule_s* pdi_cfg_rule;
	int i;
	for(i = 0; i < sr->nr_of_rules; i ++) {
		pdi_cfg_rule = pdi_rule_get(sr, i);
		if (&pdi_cfg_rule == NULL) {
			printf("##START##EC-no JAMA rule...##END##\n");
			return 1;
		}
        memset(m16_data_buf,0,sizeof(m16_data_buf));
		switch(pdi_cfg_rule->type) {
		case PDI_RULE_JAMA:
			printf("##START##EC-Checking JAMA...##END##\n");
			if(JAMA_on())
				m16_data_buf[0] = 0x00;			//JAMA absent
			else m16_data_buf[0] = 0x01;			//JAMA present
                        if(pdi_verify(pdi_cfg_rule, m16_data_buf) == 0)
			continue;
		else return 1;
			break;
		case PDI_RULE_UNDEF:
			return 1;
		}
                /*for(int i=0;i<20;i++)
                    printf("%2x",rc_data_buf[i]);*/

	}

	return 0;
}

/*checking Part NO.*/
static int m16_check_partnr()
{
	int msg_len;
	char part_data[600];               //maybe soft version too long but not in use
	can_msg_t msg_res;
	printf("##START##EC-Checking part NO.##END##\n");
	if(usdt_GetDiagFirstFrame(&m16_getpart_msg, 1, NULL, &msg_res, &msg_len))
		return 1;
	if(msg_len > 1) {
		m16_msg_buf[0] = msg_res;
		if(usdt_GetDiagLeftFrame(m16_msg_buf, msg_len))
			return 1;
	}
	/*pickup partnumber*/
	memcpy(part_data, (msg_res.data + 4), 4);
	memcpy((part_data + 4) ,(m16_msg_buf + 1)->data + 1 ,1);
        for(int i = 0; i < 80; i++)
          printf("%x",part_data[i]);
        
        printf("*********************\n");
        

	/*check partnumber*/
	//if(memcmp(part_data, bcode_1 + 2, 3))
	//	return 1;
	//if((part_data[3] != 0x30) || (part_data[4] != 0x41))
	//	return 1;

	return 0;
}

/*checking barcode*/
static int m16_check_barcode()
{
	char read_bcode[15];
	printf("##START##EC-Checking barcode...##END##\n");
	if(m16_GetCID(0xfe8d, m16_data_buf))
		return 1;

	memcpy(read_bcode, m16_data_buf, 15);
	printf("##START##RB-");
	printf(read_bcode,"\0");
	printf("##END##\n");

	if (memcmp(m16_data_buf, bcode_1, 14))
		return 1;

	return 0;
}

/*checking Resis*/
static int m16_check_resis()
{
	int msg_len;
	can_msg_t msg;
	printf("##START##EC-Checking resis...##END##\n");
	if (usdt_GetDiagFirstFrame(&m16_resis, 1, NULL, &msg, &msg_len))
		return 1;
	if (msg_len > 1)
	usdt_GetDiagLeftFrame(m16_msg_buf, msg_len);
	for (int i = 0; i < msg_len; i ++)
		can_msg_print(m16_msg_buf + i, "\n");


	return 0;
}

/*clear error code*/
int pdi_clear_dtc(void)
{
	int msg_len;
	can_msg_t msg;

	pdi_mdelay(3000);
	pdi_startsession();
	if (usdt_GetDiagFirstFrame(&m16_clrdtc_msg, 1, NULL, &msg, &msg_len))
		return 1;
	if (msg.data[1] != 0x6E)	/*positive response is 0x7b*/
		return 1;
	return 0;
}

/*get fault from ECU,saved to data,the nr of fault is pnum_fault*/
static int m16_GetFault(char *data, int * pnum_fault)
{
	int i, result = 0;

	if(m16_GetCID(0xfe80, data))
		return 1;

	memset(data + 117, 0x00, 10);

	for (i = 0; i < 117; i += 2) {
		if (data[i] | data[i+1])
			result ++;
                else break;
	}

	* pnum_fault = result;

	return 0;
}
int main(void)
{
	can_cfg_t cfg_pdi_can = {
		.baud = 500000,
	};

	const struct pdi_cfg_s* pdi_cfg_file;
	char bcode[16], temp[2], bcode_M16[2];
        char abc;
	int num_fault, rate;

	/*init all*/
	ulp_init();
	pdi_drv_Init();						//led,beep

        /*for(int i=0;i<10;i++)
        {
          GPIOE->ODR |= 1<<15;
          GPIOE->ODR &= ~(1<<15);
        }*/
	pdi_relay_init();
	ls1203_Init(&pdi_ls1203);			//scanner
	pdi_can_bus->init(&cfg_pdi_can);	//init can
	usdt_Init(pdi_can_bus);

	/*init action*/
	pre_check_action();

	/*check process*/
	while(1) {
		 memset(bcode, 0, sizeof(bcode));
                 ls1203_Clear(&pdi_ls1203,abc);
		 while(ls1203_Read(&pdi_ls1203, bcode)) {
			 ulp_update();
			 if(target_on())
			 start_botton_on();
			 else start_botton_off();
		 }
		 /*action start checking*/
		 start_action();

		 /*barcode processing*/
		 bcode[15] = '\0';
		 memcpy(bcode_1, bcode, 14);
		 printf("##START##SB-");
		 printf(bcode,"\0");
		 printf("##END##\n");
                 bcode_M16[0] = bcode[9];
                 bcode_M16[1] = '\0';
		 //bcode[5] = '\0';

		 printf("##START##STATUS-5##END##\n");

		 /*check ECU ready*/
		 if(!target_on()) {
			 pdi_noton_action();
			 printf("##START##EC-Target is not on the right position##END##\n");
			 continue;
		 }

		// /*get cfg file */
		 pdi_cfg_file = pdi_cfg_get(bcode_M16);

		// /*whether or not the config file exist*/
		 if(pdi_cfg_file == NULL) {
			 pdi_fail_action();
			 printf("##START##EC-No This Config File##END##\n");
			 continue;
		 }

		// /*if ECU on,set relays*/
		 pdi_set_relays(pdi_cfg_file);

		/*ECU power on*/
		pdi_IGN_on();

		/*delay 1s*/
		for (rate = 5; rate <= 17; rate ++) {
			pdi_mdelay(89);                      //89
			printf("##START##STATUS-");
			sprintf(temp, "%d", rate);
			printf("%s", temp);
			printf("##END##\n");
		}

		pdi_startsession();

#if 1

                //pdi_mdelay(200);
		/*check barcode*/
		if(m16_check_barcode()) {
			pdi_fail_action();
			printf("##START##EC-Barcode Wrong##END##\n");
			continue;
		}
		printf("####EC-      Checking barcode Done...##END##\n");

		/*check jama*/
		/*if(m16_check_JAMA(pdi_cfg_file)) {
			pdi_fail_action();
			printf("##START##EC-JAMA wrong...##END##\n");
			continue;
		}
		printf("##START##EC-      Checking JAMA Done...##END##\n");*/

		/*check part NO.*/
		/*if(rc_check_partnr()) {
			pdi_fail_action();
			printf("##START##EC-PartNumber Wrong...##END##\n");
			continue;
		}
		printf("##START##EC-      Checking part NO. Done...##END##\n");*/

		printf("##START##EC-Waiting for ECU ready...##END##\n");
 #endif
		/*delay 7s*/
		for (rate = 17; rate <= 96; rate ++) {
			pdi_mdelay(89);
			printf("##START##STATUS-");
			sprintf(temp, "%d", rate);
			printf("%s", temp);
			printf("##END##\n");
		}

                //printf("##START##EC-Waiting for Read soft version...##END##\n");
		//pdi_startsession();
                //m16_check_partnr();
		//m16_check_resis();
		pdi_startsession();
                memset(m16_fault_buf,0,64);
		for(int i = 3; i > 0; i --) {
			int result = m16_GetFault(m16_fault_buf, &num_fault);
			if(result != 0 && i == 1) {
				printf("##START##STATUS-Get DTC error!##END##\n");
				pdi_fail_action();
				goto ERR_x;
			}
			if(result != 0)
				continue;
			if(num_fault) {
				printf("##START##EC-");
				printf("num of fault is: %d\n", num_fault);
				for (i = 0; i < num_fault*2; i += 2)
					printf("0x%02x, 0x%02x\n", m16_fault_buf[i]&0xff,
				m16_fault_buf[i+1]&0xff);
				printf("##END##\n");
				/*if(num_fault == 2) break;
					else{
						pdi_fail_action();
					goto ERR_x;
					}*/
				if(m16_error_check(m16_fault_buf, &num_fault, pdi_cfg_file)) {
					pdi_fail_action();
					goto ERR_x;
				} else {
					printf("##START##EC-These faults are acceptale...##END##\n");
					break;
					}
			} else break;
		}

		pdi_pass_action();

		ERR_x:
			printf("##START##STATUS-100##END##\n");
	}
}

#if 1
static int cmd_m16_func(int argc, char *argv[])
{
	int num_fault, i;

	const char *usage = {
		"m16 , usage:\n"
		"m16 fault		read the error code\n"
	};

	if (argc < 2) {
		printf("%s", usage);
		return 0;
	}

	if(argc == 2) {
		if(argv[1][0] == 'f') {
			pdi_IGN_on();
			pdi_mdelay(100);
			pdi_startsession();
			if (m16_GetFault(m16_fault_buf, &num_fault))
				printf("##ERROR##\n");
			else {
				printf("##OK##\n");
				printf("num of fault is: %d\n", num_fault);
				for (i = 0; i < num_fault*2; i += 2)
					printf("0x%02x,0x%02x\n", m16_fault_buf[i]&0xff,
					m16_fault_buf[i+1]&0xff);
			}
			pdi_IGN_off();
		}
                if(argv[1][0] == 's') {
                      pdi_IGN_on();  
                      pdi_startsession();
                      //pdi_IGN_on();  
                }
	}

	return 0;
}

const cmd_t cmd_m16 = {"m16", cmd_m16_func, "m16 cmd i/f"};
DECLARE_SHELL_CMD(cmd_m16)
#endif