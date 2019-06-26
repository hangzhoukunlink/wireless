#ifndef RSU_H
#define RSU_H

#ifndef LENGTH_MAX
#define LENGTH_MAX 256
#endif

struct rsu_shell_s {
	char cmd_buffer[LENGTH_MAX];
	short cmd_idx;
};
typedef union {
	char BYTE;
	struct ByteHalfByte {
		unsigned BYTEL : 4;
		unsigned BYTEH : 4;
	} BYTE_HBYTE;
	struct ByteBit {
		unsigned BIT1 : 1;
		unsigned BIT2 : 1;
		unsigned BIT3 : 1;
		unsigned BIT4 : 1;
		unsigned BIT5 : 1;
		unsigned BIT6 : 1;
		unsigned BIT7 : 1;
		unsigned BIT8 : 1;
	} BYTEBIT;
} FIXTURE_STATUS_T;

typedef union {
	struct ThreeChar {
		char a;
		char b;
		char c;
	} THREE_CHAR;
	struct ByteFlag {
		unsigned FLAG0 : 1;
		unsigned FLAG1 : 2;
		unsigned FLAG2 : 2;
		unsigned FLAG3 : 2;
		unsigned FLAG4 : 2;
		unsigned FLAG5 : 2;
		unsigned FLAG6 : 2;
		unsigned FLAG7 : 2;
		unsigned FLAG8 : 2;
		unsigned UNUSE : 7;
	} BYTE_FLAG;
} MSG_T;
typedef enum {
	EXIST_YES = 0,
	EXIST_NO = !EXIST_YES
} EXIST_E;
typedef struct {
	EXIST_E p[4];
	int counter;
} EXIST_T;
typedef struct {
	short a[16];
	float b[16];
	short sum;
	float average;
} ADC_DATE_T;
typedef enum {
    LEFT = 0,
    RIGHT = !LEFT
} POSITION_E;
typedef enum {
	IDLE = 0,
	READY,
	COMPLETE
} FIXTURE_E;
typedef enum {
	OK = 0,
	NOK,
	NC
} PRODUCT_E;
typedef struct {
	PRODUCT_E product[8];
} PRODUCT_T;
typedef enum {
	INFORM_NO = 0,
	INFORM_ID,
	INFORM_CONF,
	INFORM_BARC,
	INFROM_FUNC,
} INFORM_E;
typedef enum {
	ERROR_NO = 0,
	ERROR_NUM,
	ERROR_FUNC,
	ERROR_ITAC,
	ERROR_BACHU,
	ERROR_ECUNG,
	ERROR_POSITION,
} ERROR_E;
typedef struct {
	uart_bus_t *bus;
	int data_len;
	time_t dead_time;
} autoliv_t;
POSITION_E *Position_Get(void);
ERROR_E *Error_Get(void);
FIXTURE_STATUS_T *Fixture_Status_Return(void);
void RSU_Init(void);
int RSU_Uart_Get(char *rx_data);
void RSU_Uart_Send(char *tx_data);
void RSU_Uart_SendChar(char tx_data);
int RSU_Can_Read(can_msg_t *msg);
MSG_T RSU_Can_Explain(can_msg_t *can_msg);
void RSU_Read_Power(ADC_DATE_T *adc_array);
EXIST_T RSU_Read_Status(FIXTURE_STATUS_T *fixture_status);
int RSU_mdelay(int ms);
//void RSU_CounterA_Add();
//void RSU_CounterB_Add();
void RSU_RelayA_Set();
void RSU_RelayB_Set();
void RSU_RelayA_Reset();
void RSU_RelayB_Reset();
int RSU_TargetA_On();
int RSU_TargetB_On();
void RSU_check_action();
void RSU_Start_Action();
void RSU_Beep_On();
void RSU_Beep_Off();
void RSU_Power_On();
void RSU_Power_Off();
void RSU_Read_Power(ADC_DATE_T *adc_array);
void RSU_Result(PRODUCT_T product);
typedef INFORM_E (*FUNCTION_T)(void);
#endif