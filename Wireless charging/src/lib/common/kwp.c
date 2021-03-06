/*
 * 	miaofng@2010 initial version
 *		This file is used to provide data link level i/f for ISO14230/keyword2000 protocol
 */

#include "config.h"
#include "common/kwp.h"
#include "uart.h"
#include <stdlib.h>
#include "sys/sys.h"
#include "queue.h"
#include "task.h"
#include "ulp_time.h"

#define __DEBUG
#define __DEBUG_FRAME

#ifdef __DEBUG
#include <stdarg.h>
#include "shell/cmd.h"
#include "common/print.h"
#include <stdio.h>
#include <string.h>
#define debug(...) do { \
	print(__VA_ARGS__); \
} while(0)
#else
#define debug(...)
#endif

#ifdef __DEBUG_FRAME
#define debugf debug
#else
#define debugf(...)
#endif

/*1-> fmt|len, 2-> fmt + len, 4-> fmt + addr + len, default 4*/
static uart_bus_t *kwp_uart;
static char kwp_head_tn;
static char kwp_head_rn;
static char kwp_tar;
static char kwp_src;
static char kwp_frame[260];
static kwp_err_t kwp_err;

#define tFRAME(pbuf) ((pbuf) - kwp_head_tn)
#define tPBUF(frame) ((frame) + kwp_head_tn)
#define tPLEN(frame) ((kwp_head_tn & 0x01)? ((*(frame + 0)) & 0x3f) : (*((frame) + kwp_head_tn - 1)))

#define rFRAME(pbuf) ((pbuf) - kwp_head_rn)
#define rPBUF(frame) ((frame) + kwp_head_rn)
#define rPLEN(frame) ((kwp_head_rn & 0x01)? ((*(frame + 0)) & 0x3f) : (*((frame) + kwp_head_rn - 1)))

int kwp_Init(uart_bus_t *uart)
{
	uart_cfg_t cfg = UART_CFG_DEF;

	kwp_uart = uart;
	kwp_head_tn = 4;
	kwp_head_rn = 3;
	kwp_tar = KWP_DEVICE_ID;
	kwp_src = KWP_TESTER_ID;
	
	cfg.baud = KWP_BAUD;
	kwp_uart -> init(&cfg);
	return 0;
}

void kwp_SetAddr(char tar, char src)
{
	kwp_tar = tar;
	kwp_src = src;
}

void kwp_SetFormat(char kb1, char kb2)
{
	char len = 4;
	
	if(!(kb1 & KWP_KB_HB1)) {
		//addr in header is not supported
		len -= 2;
	}
	
	if(!(kb1 & KWP_KB_AL1)) {
		//addtional len byte is not supported
		len -= 1;
	}
	
	kwp_head_tn = len;
}

int kwp_EstablishComm(void)
{
	//fast init is the only wakeup protocol supported now
	kwp_uart -> wake(WAKE_EN);
	kwp_uart -> wake(WAKE_HI);
	mdelay(300);

	//set low
	kwp_uart -> wake(WAKE_LO);
	//vTaskDelay(KWP_FAST_INIT_MS * 1000 / CONFIG_TICK_HZ);
	mdelay(KWP_FAST_INIT_MS);
	
	//set high
	kwp_uart -> wake(WAKE_HI);
	//vTaskDelay(KWP_FAST_INIT_MS * 1000 / CONFIG_TICK_HZ);
	mdelay(KWP_FAST_INIT_MS);
	
	//reset
	kwp_uart -> wake(WAKE_RS);
	
	//start comm
	kwp_StartComm(0, 0); //flush serial port
	return kwp_StartComm(0, 0);
}

void *kwp_malloc(int n)
{
	int len = kwp_head_tn;
	char *pbuf = kwp_frame + 4; //static allocation only
	char *f = tFRAME(pbuf);
	
	*f ++ = KBP_FMT_ADR_PHY;
	if(len > 2) {
		*f ++ = kwp_tar;
		*f ++ = kwp_src;
	}
	if(len == 2 || len == 4) {
		*f ++ = 0;
	}
	
	return f;
}

void kwp_free(void *p)
{
	//nothing needs to be done here
}

static char kwp_cksum(const char *p, int n)
{
	int i;
	char cksum = 0;
	
	for(i = 0; i < n; i++ ) {
		cksum += *p;
		p ++;
	}

	return (cksum);
}

int kwp_send(char *tbuf, int tn, int ms)
{
	time_t timeout;
	int n = kwp_head_tn;
	char *f = tFRAME(tbuf);
	
	if(n == 1 || n == 3) { //len in fmt
		f[0] += tn;
	}
	else { //len byte
		f[n - 1] = tn;
	}
	
	n += tn;
	f[n] = kwp_cksum(f, n);
	n ++;

	//flush input fifo
	while(kwp_uart -> poll()) {
		kwp_uart -> getchar();
	}

	//send the frame to bus
	debugf("kwp tx(%02d):", n);
	for(int i = 0; i < n; i ++) {
		kwp_uart -> putchar(f[i]);
		debugf(" %02x", f[i]);
		
		//read echo back
		timeout = time_get(ms);
		while(! kwp_uart -> poll() ) {
			if(time_left(timeout) < 0)
				break;
		};

		if(kwp_uart -> poll())
			kwp_uart -> getchar();
	}
	debugf("\n");
	
	return 0;
}

int kwp_check(char *pbuf, int ofs)
{
	char *p = pbuf + ofs;
	int err;
	
	/*success?*/
	err = 0;
	kwp_err.rid = pbuf[0];
	kwp_err.sid = 0;
	kwp_err.code = 0;
	
	/*fail?*/
	if(p[0] == SID_ERR) {
		if(p[2] == RCR_RP) {
			//reponse pending ....
			err = RCR_RP;
		}
		else {
			err = -1;
			kwp_err.sid = p[1];
			kwp_err.code = p[2];
			kwp_free(pbuf);
		}
	}

	if(err < 0) {
		debug("\nNegative Response!\n");
	}
	
	return err;
}

int kwp_recv(char *pbuf, int ms)
{
	int bytes, len, ofs;
	time_t timeout;
	int ret = -1;
	char *f = rFRAME(pbuf);
	
	ofs = 0;
	len = 0;
	timeout = time_get(ms);
	bytes = 0;
	while(1) {
		//timeout?
		if(time_left(timeout) < 0) {
			kwp_err.sid = pbuf[0];
			kwp_err.rid = 0;
			kwp_err.code = 0xfd;
			kwp_free(pbuf);
			break;
		}
		
		//try to receive from bus
		while(kwp_uart -> poll()) {
			f[bytes] = kwp_uart -> getchar();
			bytes ++;
		}
		
		//display
		debugf("\rkwp rx(%02d):", bytes);
		for(int i = 0; i < bytes; i ++) {
			debugf(" %02x", f[i]);
		}

		//full head received
		if((bytes >= ofs + kwp_head_rn) && (len == 0)) {
			len = rPLEN(f + ofs) + kwp_head_rn + 1;
		}
		
		//full frame received
		if(len && (bytes >= ofs + len)) {
			ret = kwp_check(pbuf, ofs);
			if(ret <= 0) {
				if(ofs) {
					memcpy(f, f + ofs, len);
				}
				break;
			}
			
			ofs += len;
			len = 0;
		}
	}
	debugf("\n");
	
	return ret;
}

int kwp_GetLastErr(char *rid, char *sid, char *code)
{
	if(rid)
		*rid = kwp_err.rid;
	if(sid)
		*sid = kwp_err.sid;
	if(code)
		*code = kwp_err.code;
		
	return 0;
}

/*
The 81 Op-Code is used to start communications with an ECU.  Communications will remain established while 
activity exists on the link.  If the link is inactive for five seconds the ECU will revert back to a waiting for 
communications state.  There are no negative responses allowed for the Start Communication request.
*/
int kwp_StartComm(char *kb1, char *kb2)
{	
	char *pbuf;

	debug("->%s\n", __FUNCTION__);

	pbuf = kwp_malloc(2);
	pbuf[0] = SID_81;
	kwp_send(pbuf, 1, KWP_RECV_TIMEOUT_MS);
	if(kwp_recv(pbuf, KWP_RECV_TIMEOUT_MS))
		return -1;

	if(kb1)
		*kb1 = pbuf[1];
	if(kb2)
		*kb2 = pbuf[2];

	kwp_free(pbuf);
	return 0;
}

/*
The 82 Op-Code is used to stop communications with an ECU.  There is no negative response allowed for the Stop 
Communication request.
*/
int kwp_StopComm(void)
{
	char *pbuf;

	debug("->%s\n", __FUNCTION__);

	pbuf = kwp_malloc(3);
	pbuf[0] = SID_82;
	kwp_send(pbuf, 1, KWP_RECV_TIMEOUT_MS);
	if(kwp_recv(pbuf, KWP_RECV_TIMEOUT_MS))
		return -1;
		
	kwp_free(pbuf);
	return 0;
}

/*
The 83 Op-Code is used to access the communications parameters.  This Op-Code is issued after a security request (
Op-Code 27) has passed.  This Op-Code is performed in two steps first, the optimum timing parameters for use during 
a reprogramming event are read from the ECU.  Second,  the optimum timing parameters are put into a service request 
83 and sent to the ECU. 
*/
int kwp_AccessCommPara(void)
{
	char *pbuf;
	char p2min, p2max, p3min, p3max, p4min;

	debug("->%s\n", __FUNCTION__);

	pbuf = kwp_malloc(7);
	pbuf[0] = SID_83;
	pbuf[1] = 0;
	kwp_send(pbuf, 2, KWP_RECV_TIMEOUT_MS);
	if(kwp_recv(pbuf, KWP_RECV_TIMEOUT_MS))
		return -1;

	p2min = pbuf[2];
	p2max = pbuf[3];
	p3min = pbuf[4];
	p3max = pbuf[5];
	p4min = pbuf[6];
	
	kwp_free(pbuf);
	pbuf = kwp_malloc(6);
	pbuf[0] = SID_83;
	pbuf[1] = 0x03;
	pbuf[2] = p2min;
	pbuf[3] = p2max;
	pbuf[4] = p3min;
	pbuf[5] = p3max;
	pbuf[6] = p4min;
	kwp_send(pbuf, 7, KWP_RECV_TIMEOUT_MS);
	if(kwp_recv(pbuf, KWP_RECV_TIMEOUT_MS))
		return -1;
		
	kwp_free(pbuf);
	return 0;
}

/*
The 10 Op-Code is used to inform devices on the serial data link that the tool is ready to start diagnostic procedures
 using the link and the devices on the link may need to alter their normal operation. 
 para baud: $00->invalid, $01->9600, $02->19200, $03->38400, $04->57600, $05->115200
*/
int kwp_StartDiag(char mode, char baud)
{
	char *pbuf;
	int n;

	debug("->%s\n", __FUNCTION__);

	n = (baud > 0) ? 3 : 2;
	pbuf = kwp_malloc(3);
		
	pbuf[0] = SID_10;
	pbuf[1] = mode;
	pbuf[2] = baud;
	kwp_send(pbuf, n, KWP_RECV_TIMEOUT_MS);
	if(kwp_recv(pbuf, KWP_RECV_TIMEOUT_MS))
		return -1;
		
	kwp_free(pbuf);
	return 0;
}

/*
The 27 Op-Code is used to open an ECU's memory for reprogramming.
access mode = 1/3, no key is needed
access mode = 2/4, key must be provided
*/
int kwp_SecurityAccessRequest(char mode, short key, int *seed)
{
	char *pbuf;
	int n;

	debug("->%s\n", __FUNCTION__);

	n = (mode & 0x01) ? 2 : 4;
	pbuf = kwp_malloc(4);
		
	pbuf[0] = SID_27;
	pbuf[1] = mode;
	if(!(mode & 0x01)) {
		pbuf[2] = (char)(key >> 8);
		pbuf[3] = (char)(key >> 0);
	}
	kwp_send(pbuf, n, KWP_RECV_TIMEOUT_MS);
	if(kwp_recv(pbuf, KWP_RECV_TIMEOUT_MS))
		return -1;

	if((mode & 0x01) && seed) {
		(*seed) = pbuf[2];
		(*seed) <<= 8;
		(*seed) += pbuf[3];
	}
	
	kwp_free(pbuf);
	return 0;
}

/*
The 34 Op-Code will build a standard Service Request 34 to send to an ECU.  A request 34 will 
prepare the ECU to receive data from the serial data link.  Request 34s are used in conjunction 
with service request mode 36s (Op-Codes 90, 91, and 93) to download information to ECU's.
plen := max allowed data len
*/
int kwp_RequestToDnload(char fmt, int addr, int size, char *plen)
{
	char *pbuf;
	int n = 1;

	debug("->%s\n", __FUNCTION__);

	pbuf = kwp_malloc(8);
	pbuf[0] = SID_34;
	if(size != 0) {
		pbuf[1] = (char)(addr >> 16);
		pbuf[2] = (char)(addr >>  8);
		pbuf[3] = (char)(addr >>  0);
		pbuf[4] = fmt;
		pbuf[5] = (char)(size >> 16);
		pbuf[6] = (char)(size >>  8);
		pbuf[7] = (char)(size >>  0);	
		n = 8;
	}
	kwp_send(pbuf, n, KWP_RECV_TIMEOUT_MS);
	if(kwp_recv(pbuf, KWP_RECV_TIMEOUT_MS))
		return -1;
		
	if(plen)
		*plen = pbuf[1];

	kwp_free(pbuf);	
	return 0;
}

/*
The 36 Op-Code is used by the client to transfer data either from the client to the server 
(download) or form the server to the client (upload).  The data transfer is defined in the 
preceding requestDownload or requestUpload service. The upload service is not implemented.  
The data is included in the parameter(s) transferRequestParameter in the transfer request message(s).
*/
int kwp_TransferData(int addr, int alen, int size, char *data)
{
	char *pbuf;
	int i;

	debug("->%s\n", __FUNCTION__);

	pbuf = kwp_malloc(4 + size);
	pbuf[0] = SID_36;
	for(i = alen; i > 0; i --) {
		pbuf[i] = (char) addr;
		addr >>= 8;
	}
	i = alen + 1;
	memcpy(&pbuf[i], data, size);
	kwp_send(pbuf, i + size, KWP_RECV_TIMEOUT_MS);
	if(kwp_recv(pbuf, KWP_RECV_TIMEOUT_MS))
		return -1;

	kwp_free(pbuf);
	return 0;
}

/*
The 37 Op-Code will build a standard Service Request 37 to send to an ECU.  A request 37 
will Request Transfer Exit.  A Request 37 may be required at the end of a reprogramming 
session after all of the Transfer Data requests are completed. 
*/
int kwp_RequestTransferExit(void)
{
	char *pbuf;

	debug("->%s\n", __FUNCTION__);
	
	pbuf = kwp_malloc(1);
	pbuf[0] = SID_37;	
	kwp_send(pbuf, 1, KWP_RECV_TIMEOUT_MS);
	if(kwp_recv(pbuf, KWP_RECV_TIMEOUT_MS))
		return -1;

	kwp_free(pbuf);
	return 0;
}

/*
The 38 Op-Code will build a standard service request 38 to send to an ECU.   This Op-Code is used to start
 a  Device Specific Control Routine, from the Utility File, that was downloaded to RAM by an Op-Code 90.  
 The address information for the service request is read from the routine in the Utility File.  This Op-Code 
 also supports the option to pass in Routine Entry Options from a Data Routine.  For details on how to use this 
 feature see the Action Fields and the Pseudo Code.  The routine that contains the Routine Entry Options should 
 be formatted with a zero load address and a valid length (see description of Data Routines).  The data portion 
 of the routine should only contain the Routine Entry Options to include in the request with no additional data.  
 If the routine execution results in the ECU responding with two bytes of additional data then Op-Code 78 
 should be used instead of Op-Code 38.
*/
int kwp_StartRoutineByAddr(int addr)
{
	char *pbuf;

	debug("->%s\n", __FUNCTION__);

	pbuf = kwp_malloc(4);
	pbuf[0] = SID_38;
	pbuf[1] = (char)(addr >> 16);
	pbuf[2] = (char)(addr >>  8);
	pbuf[3] = (char)(addr >>  0);	
	kwp_send(pbuf, 4, KWP_RECV_TIMEOUT_MS);
	if(kwp_recv(pbuf, KWP_RECV_TIMEOUT_MS))
		return -1;

	kwp_free(pbuf);
	return 0;
}

int kwp_StartRoutineByLocalId(int id, char *para, int n)
{
	char *pbuf;
	int i = n;

	debug("->%s\n", __FUNCTION__);

	pbuf = kwp_malloc(n + 3);
	pbuf[0] = SID_31;
	pbuf[1] = (char) id;
	i --;
	while(i >= 0) {
		pbuf[2 + i] = para[i];
	}
	kwp_send(pbuf, n + 2, KWP_RECV_TIMEOUT_MS);
	if(kwp_recv(pbuf, KWP_RECV_TIMEOUT_MS))
		return -1;

	kwp_free(pbuf);
	return 0;
}

int kwp_EcuReset(char mode)
{
	char *pbuf;

	debug("->%s\n", __FUNCTION__);

	pbuf = kwp_malloc(3);
	pbuf[0] = SID_11;
	pbuf[1] = mode;
	kwp_send(pbuf, 2, KWP_RECV_TIMEOUT_MS);
	if(kwp_recv(pbuf, KWP_RECV_TIMEOUT_MS))
		return -1;

	kwp_free(pbuf);
	return 0;	
}

#ifdef __DEBUG
/*
this routine is only for debug purpose
*/
int kwp_debug(int n, char *data)
{
	int i, bytes;
	char *pbuf, *f;
	time_t timeout;
	
	pbuf = kwp_malloc(n);
	for(i = 0; i < n; i ++) {
		pbuf[i] = data[i];
	}
	
	kwp_send(pbuf, n, KWP_RECV_TIMEOUT_MS);
	
	//display received data
	f = rFRAME(pbuf);
	timeout = time_get(2000);
	bytes = 0;
	while(1) {
		if(time_left(timeout) < 0) {
			break;
		}
		
		//try to receive from bus
		while(kwp_uart -> poll()) {
			f[bytes] = kwp_uart -> getchar();
			bytes ++;
		}
		
		print("\rkwp rx(%02d):", bytes);
		for(int i = 0; i < bytes; i ++) {
			print(" %02x", f[i]);
		}
	}
	
	kwp_free(pbuf);
	return 0;
}

static char htoc(char *buf)
{
	char h, l;

	h = buf[0];
	h = (h >= 'a') ? (h - 'a' + 10) : h;
	h = (h >= 'A') ? (h - 'A' + 10) : h;
	h = (h >= '0') ? (h - '0') : h;

	l = buf[1];
	l = (l >= 'a') ? (l - 'a' + 10) : l;
	l = (l >= 'A') ? (l - 'A' + 10) : l;
	l = (l >= '0') ? (l - '0') : l;

	return (h << 4) | l;
}

enum {
	CMD_WAKE,
	CMD_TRANS,
};

struct cmd_kwp_msg {
	char cmd;
	char bytes;
	char *para;
};

static xQueueHandle cmd_kwp_queue;
static void cmd_kwp_task(void *pvParameters)
{
	struct cmd_kwp_msg msg;
	while(xQueueReceive(cmd_kwp_queue, &msg, portMAX_DELAY) == pdPASS) {
		if(msg.cmd == CMD_WAKE) {
			kwp_Init(&uart1);
			kwp_EstablishComm();
		}
		else {
			kwp_debug(msg.bytes, msg.para);
			sys_free(msg.para);
		}
	}
}

static int cmd_kwp_func(int argc, char *argv[])
{
	int n, i;
	char *pbuf;
	struct cmd_kwp_msg msg;
	const char usage[] = { \
		" usage:\n" \
		" kwp start\n"
		" kwp d0 d1 d2 ... dn	d0..n is only the data field\n" \
	};

	if(argc == 1) {
		printf(usage);
		return 0;
	}

	if(argc > 1) {
		if(!strcmp(argv[1], "start")) { //kwp start?
			msg.cmd = CMD_WAKE;
		}
		else {
			n = argc - 1;
			msg.cmd = CMD_TRANS;
			msg.bytes = (char) n;
			pbuf = sys_malloc(msg.bytes);
			for(i = 0; i < n; i ++)
				pbuf[i] = htoc(argv[i + 1]);
			msg.para = pbuf;
		}
		
		if(!cmd_kwp_queue) { //create the task and queue
			cmd_kwp_queue = xQueueCreate(1, sizeof(struct cmd_kwp_msg));
			xTaskCreate(cmd_kwp_task, (signed portCHAR *) "cmd_kwp", 128, NULL, tskIDLE_PRIORITY + 1, NULL);	
		}
		
		if(xQueueSend(cmd_kwp_queue, &msg, 0) != pdPASS) {
			printf("Busy now! try later ...\n");
		}
	}

	return 0;
}

const cmd_t cmd_kwp = {"kwp", cmd_kwp_func, "kwp tx/rx commands"};
DECLARE_SHELL_CMD(cmd_kwp)
#endif
