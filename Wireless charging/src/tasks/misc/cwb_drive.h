#ifndef CWB_DRIVE_H
#define CWB_DRIVE_H


#define COUNT 60 //
#define SPV_FALL 25 //ms
#define ISG_RISE 300 //ms
//SPV_OUT
//         |------------------|          |-------------------|
//_________|                  |__________|                   |
//V0 T0(5s)     V1 T1(55s)
//Vol < V1 < Voh
//Is 
//Eff = vi * ii / vo * io * 100%
//
//ISG     500ms
//       |————|
//_______|    |_______________
enum {
	CHANNEL_A1,
        CHANNEL_A2,
	CHANNEL_B1,
        CHANNEL_B2,
	CHANNEL_C1,
        CHANNEL_C2,
        CHANNEL_D1,
        CHANNEL_D2,
};


typedef enum {
    START = 0,                                //0
    END,                                      //1
    POLL,   //ask                             //2
    CFG,    //ask V0 V1 T0 T1 Vol Voh Is Eff  //3
    READ,   //ask                             //4
    HAND,   //target                          //5
    FAULT,  //reset                          //6
    HAND1,                                    //7
   
} CWB_MODE_E;

typedef enum {
    START1 = 0,                                
    END1,
    POLL1,   //ask 
    CFG1,
    READ1,   //ask                             
    HAND01,   //target                          
    FAULT1,  //reset                         
    HAND11,                                    
} CWB_MODE_E1;

typedef enum {
    START2 = 0,                                
    END2,
    POLL2,   //ask   
    CFG2,
    READ2,   //ask                             
    HAND02,   //target                          
    FAULT2,  //reset                         
    HAND12,                                    
} CWB_MODE_E2;

typedef enum {
    START3 = 0,                                
    END3,
    POLL3,   //ask  
    CFG3,
    READ3,   //ask                             
    HAND03,   //target                          
    FAULT3,  //reset                         
    HAND13,                                    
} CWB_MODE_E3;

typedef enum {
    START4 = 0,                                
    END4,
    POLL4,   //ask 
    CFG4,
    READ4,   //ask                             
    HAND04,   //target                          
    FAULT4,  //reset                         
    HAND14,                                    
} CWB_MODE_E4;

typedef enum {
    IDLE1 = 0,
    BUSY1,
    PASS1,
    FAIL1
} CWB_STATUS_E1;

typedef enum {
    IDLE2 = 0,
    BUSY2,
    PASS2,
    FAIL2
} CWB_STATUS_E2;

typedef enum {
    IDLE3 = 0,
    BUSY3,
    PASS3,
    FAIL3
} CWB_STATUS_E3;

typedef enum {
    IDLE4 = 0,
    BUSY4,
    PASS4,
    FAIL4
} CWB_STATUS_E4;

typedef enum {
    VCC_INL = 1,
    VCC_INH
} power;

typedef enum {
    down1 = 1,
    down2
} hub_down;

typedef enum {
    current2_1 = 0,
    current1_5, 
    current1_0,
    current0_5,
    current0_0
} current_set;

typedef enum {
   INUA=0,//输入电压
   INUB  ,
   INUC  ,
   INUD  ,
   INIA  ,//输入电流
   INIB  ,
   INIC  ,
   INID  ,
   OUTUA ,//输出电压
   OUTUB ,
   OUTUC ,
   OUTUD ,
   OUTIA ,//输出电流
   OUTIB ,
   OUTIC ,
   OUTID ,
} Different_ADC_selection;

typedef enum {
    a= 0,
    b, 
    c,
    d,
} power_set;


typedef enum {
    ERROR_NO1 = 0,
    ERROR_CFG1,  //配置错误                                                                                    
    ERROR_CAN1,
    ERROR_SHORT1, //输入电压过低
    ERROR_VOL_IN_HIGH1,//输入电压过高
    ERROR_CUR_IN1,  //输入电流 过低                
    ERROR_VOL_OUT_LOW1,//输出电压过低  
    ERROR_VOL_OUT_HIGH1, //输出电压过高                            
    ERROR_CUR_OUT1,//输出电流过低
    ERROR_CUR_OUT_HIGH1,//输出电流过高
    ERROR_CUR_IN_HIGH1,//输入电流过高
} ERROR_E1;


typedef enum {
    ERROR_NO2= 0,
    ERROR_CFG2,  //配置错误                                                                                    
    ERROR_CAN2,
    ERROR_SHORT2, //DUT short 
    ERROR_VOL_IN_HIGH2,  
    ERROR_CUR_IN2,  //cur in over  
    ERROR_VOL_OUT_LOW2,
    ERROR_VOL_OUT_HIGH2,                              
    ERROR_CUR_OUT2,// 
    ERROR_CUR_OUT_HIGH2,
    ERROR_CUR_IN_HIGH2,
} ERROR_E2;


typedef enum {
    ERROR_NO3 = 0,
    ERROR_CFG3,  //配置错误                                                                                    
    ERROR_CAN3,
    ERROR_SHORT3, //DUT short 
    ERROR_VOL_IN_HIGH3,  
    ERROR_CUR_IN3,  //cur in over  
    ERROR_VOL_OUT_LOW3,
    ERROR_VOL_OUT_HIGH3,                                
    ERROR_CUR_OUT3,// 
    ERROR_CUR_OUT_HIGH3,
    ERROR_CUR_IN_HIGH3,
} ERROR_E3;

typedef enum {
    ERROR_NO4 = 0,
    ERROR_CFG4,  //配置错误                                                                                    
    ERROR_CAN4,
    ERROR_SHORT4, //DUT short 
    ERROR_VOL_IN_HIGH4,  
    ERROR_CUR_IN4,  //cur in over  
    ERROR_VOL_OUT_LOW4,
    ERROR_VOL_OUT_HIGH4,                               
    ERROR_CUR_OUT4,// 
    ERROR_CUR_OUT_HIGH4,
    ERROR_CUR_IN_HIGH4,
} ERROR_E4;


typedef struct {
    float Volinset;      //产品供电选择5v或者12v
    float fCurOutSet;    //负载老化的电流
    float fCurOutSet1;   //负载老化的电流
    float fTimesSet;     //负载时间（单位：分钟）
    float downselect;    //一个还是两个下行口
    float downVolLimitH; //下行口输出电压上限
    float downVolLimitL; //下行口输出电压下限
    float fCountVolLimit;// 允许输出电压错误次数
    float downSetVol; //下行口输出电压下限
    float productselect;
    uint32_t hwCheckSum;   //recv checksum
    
    float fCountGet11;
    float fCountGet12;
    float fCountGet13;
    float fCountGet14; 
    
    
    float fCountGet21;
    float fCountGet22;
    float fCountGet23;
    float fCountGet24;
    
    
    float fCountGet31;
    float fCountGet32;
    float fCountGet33;
    float fCountGet34;
    
    
    float fCountGet41;
    float fCountGet42;
    float fCountGet43;
    float fCountGet44;
    
    
    float fCurInDeltaLimit;//输入电流允许误差值
    float fCurOutDeltaLimit;//输出电流允许误差值    
    float fCountCurLimit;// 允许输出电流误差次数
    
    float fRatioLimit; //88.8
    uint32_t hwCheckSumCalc;  //self calc rx checksum
    
    float fVinGet; //compare with fVol_Limit & fVoh_Limit
    float fCinGet;
    float fVoutGet;
    float fCoutGet; //compare with fCur_Set +- fCur_Delta_Limit
    float fVoutGet1;
    float fCoutGet1; //compare with fCur_Set +- fCur_Delta_Limit
    float fRatioGet; //compare with fRatio_Set
    
    
   

   
    
    
} CWB_MISC_T;

typedef union {
    char BYTE;
    struct ByteHalfByte {
        unsigned BYTEL : 4;
        unsigned BYTEH : 4;
    } BYTE_HALF;
    struct ByteBit {
        unsigned BIT1 : 1;
        unsigned BIT2 : 1;
        unsigned BIT3 : 1;
        unsigned BIT4 : 1;
        unsigned BIT5 : 1;
        unsigned BIT6 : 1;
        unsigned BIT7 : 1;
        unsigned BIT8 : 1;
    } BYTE_BIT;
} ENCODE_STATUS_T;

//! \name finit state machine state
//! @{
typedef enum {
    fsm_rt_err          = -1,    //!< fsm error, error code can be get from other interface
    fsm_rt_cpl          = 0,     //!< fsm complete
    fsm_rt_on_going     = 1,     //!< fsm on-going
} fsm_rt_t;
//! @}

void cwb_can_init(void);
void lx_drive_init(void);
void cwb_en_spv_set(bool bStatus);
void cwb_en_eload_set(bool bStatus);
void cwb_isg_set(bool bStatus);
bool can_filter_set(uint32_t wId);
bool cwb_can_read(can_msg_t *ptMsg);
bool cwb_can_write(can_msg_t *ptMsg);
void product_set_led(led_t tColor, led_status_t tStatus);
uint8_t encode_read(void);
void time3_init(uint32_t wFrq);

void pv_set(uint32_t wDc);
void cwb_scan_init(uint32_t wFrq);
uint32_t cwb_mdelay(uint32_t wMs);
void cwb_adc_init(void);



#endif