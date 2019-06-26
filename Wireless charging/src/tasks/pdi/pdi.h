#ifndef __PDI_PDI_H_
#define __PDI_PDI_H_
#include "ls1203.h"
#include "can.h"
//public
void pdi_fail_action();
void pdi_pass_action();
void pdi_noton_action();
void counter_fail_add();
void counter_pass_add();
void pre_check_action();
void start_action();
void pdi_relay_init();
void pdi_set_relays();

//private in main
int pdi_startsession();
int pdi_clear_dtc();
int pdi_mdelay(int);

typedef enum {
    LEFT = 0,
    RIGHT = !LEFT
} POSITION_E;
POSITION_E *Position_Get(void);
typedef enum {
	Haima = 0,	//Kline
	Geely = 1,	//CAN
	GW1 = 2,	//CHB011 8lp CHB021 8lp new CHB021
	GW2 = 3,	//old CHB021 4lp
	GW3 = 4,	//CH041 4lp
	GW4 = 5,
	GC = 6,         //KLINE
	CK = 7,         //kline
	LHD = 8,         //kline
	SL = 9,         //kline
} TYPE_E;
TYPE_E	*Product_Type_Get(void);
typedef enum {
    ERROR_NO = 0,
    ERROR_NRP,  // not right position, can not happen in new pdi
    ERROR_NCF,  // no config file
    ERROR_BW,   //barcode wrong:compare the extern barcode with inner barcode
    ERROR_CW,
    ERROR_CW1,
    ERROR_CW2,
    ERROR_SN,
    ERROR_SV,
    ERROR_IMU,
    ERROR_JAMA,
    ERROR_NVM,
    ERROR_VIN,
    ERROR_ROC,
    ERROR_BE,
    ERROR_RCM,
    ERROR_OCS,
    ERROR_APPS,
    ERROR_ASI,
    ERROR_VAR1,
    ERROR_VAR2,
    ERROR_VAR3,
    ERROR_VAR1AEF,
    ERROR_VAR2AEF,
    ERROR_VAR3AEF,
    ERROR_TEC,
    ERROR_LOCK,
    ERROR_CODE, //exist error code
    ERROR_ENCODE, //enable error code
    ERROR_POWEROFF, // power off error
    ERROR_CURRENT, // power off error
    ERROR_PSN, // 
} ERROR_E;
typedef enum {
    StatusOK = 0,
    StatusNOK = !StatusOK
} STATUS_E;
typedef enum  {
	IDLE = 0,
	READY,
	COMPLETE
} FIXTURE_E;

typedef struct {
	uart_bus_t *bus;
	int data_len;
	time_t dead_time;
} autoliv_t;
typedef struct {
	uart_bus_t *bus;
	int data_len;
	time_t dead_time;
} kline_t;
typedef enum {
	DEBUG = 0,
	PRODUCT = !DEBUG
} DEBUG_OR_PRODUCT_E;
#define length_bcode_part 4
#define length_relay 48
#define max_length_softversion 20
#define length_errorable 20
#define length_NVM	12	//32
#define length_VIN	17
#define length_ROC	8
#define length_BE	1
#define length_RCM	2
#define length_OCS	2
#define length_APPS	32	
#define length_ASI	12
#define length_VAR1	29  //32
#define length_VAR2	32
#define length_VAR3	32
#define length_VAR1AEF	32	
#define length_VAR2AEF	32
#define length_VAR3AEF	32
#define length_TEC		1
#define length_Calibration   20
#define length_cpn 10
typedef struct {
	char bcode_part[length_bcode_part + 1];     //5 bytes
	char relay[length_relay + 1];        //48 bytes
	char length_softversion;
	char softversion[max_length_softversion + 1]; //20 bytes
	char IMU;			//1 byte
	char JAMA;          //1 byte
	char length_errorcode;    //1 byte
	char NVM[length_NVM + 1];
	char VIN[length_VIN + 1];
	char ROC[length_ROC + 1];
	char BE;
	char RCM[length_RCM + 1];
	char OCS[length_OCS + 1];
	char APPS[length_APPS + 1];
	char ASI[length_ASI + 1];
	char VAR1[length_VAR1 + 1];
	char VAR2[length_VAR2 + 1];
	char VAR3[length_VAR3 + 1];
	char VAR1AEF[length_VAR1AEF + 1];
	char VAR2AEF[length_VAR2AEF + 1];
	char VAR3AEF[length_VAR3AEF + 1];
	char TEC;
	char errorable[length_errorable + 1]; //max 20 bytes
	char Calibration[length_Calibration + 1];
	char cpn[length_cpn + 1];
} config_t;
//uart3
void config_init(config_t file);
void autoliv_init(const autoliv_t *chip);
void autoliv_send(const autoliv_t *chip, char *buffer);
void autoliv_send_char(const autoliv_t *chip, char a);
int autoliv_get(const autoliv_t *chip,char *rx_data);
void kline_init(const kline_t *chip);
void AddACU3Checksum (char *wrMsg);
int pdi_error_check(char *fault, int *num, const config_t sr);
int kline_communicate(const kline_t *chip, char *data, char *rxdata, int *length);
void CalculateKey (unsigned char accessLevel, unsigned char challenge[8],unsigned char secKeyOut[3]);
typedef enum {
	STEP_NOK = 0,
	STEP0_OK,
	STEP1_OK,
	STEP2_OK,
	STEP3_OK
} COMPLETE_E;
typedef enum {
	CAN = 0,
	KLINE = !CAN
} CAN_OR_KLINE_E;
typedef void (*FUNCTION_T)(void);
#endif