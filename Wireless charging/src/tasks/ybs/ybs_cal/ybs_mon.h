/*
 * 	miaofng@2013-3-27 initial version
 */

#ifndef __YBS_MON_H_
#define __YBS_MON_H_

#include "uart.h"
#include "stm32f10x.h"

#define ybs_uart uart2
#define ybs_uart_sel() do { \
	GPIO_WriteBit(GPIOE, GPIO_Pin_0, Bit_RESET); \
	GPIO_WriteBit(GPIOE, GPIO_Pin_1, Bit_RESET); \
} while(0)

#define IN_RANGE(x, min, max) (((x) >= (min)) & ((x) < (max)))
#define OV_RANGE(x, min, max) (!IN_RANGE(x, min, max))

enum {
	OK = 0,
	BUSY,
	E_PARA,
	E_MCD_READ,
	E_SGM_READ,
	E_MGF_OVER,
	E_POS_OVER,
	E_LVL_OVER,

	E_YBS_COMM, /*ybs communication general fail*/
	E_YBS_FRAME, /*ybs communication frame error*/
	E_YBS_TIMEOUT, /*ybs communication error*/
	E_YBS_RESPONSE, /*ybs sensor error*/

	E_CAL_PRECHECK_DET,
	E_CAL_PRECHECK_ASIG,
};

int monitor_init(void);
int sgm_read(float *gf);
int sgm_read_until_stable(float *gf, int ms);

int mov_i(int distance, float gf_max); /*for initially positioning purpose*/
int mov_n(int steps);
int mov_p(int pos); //mov to absolute position
int mov_f(float target_gf, int gf_settling_ms);
int mov_p_measure(int pos, float *gf_sgm, float *gf_ybs);

#endif
