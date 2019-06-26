/*
 *	miaofng@2011 initial version
 *	David peng.guo@2011 add content for PDI_SDM10
 *  jiamao.gu update ...
 */
#include "config.h"
#include "ulp_time.h"
#include "drv.h"
#include "pdi.h"
#include "stm32f10x.h"
#include "sys/task.h"
#include "led.h"
#include "stdio.h"

#define Version2 1 
POSITION_E *POSITION;
#ifdef CONFIG_PDI_SDM10
#define LED_red_on			(1<<0)
#define LED_green_on		(1<<1)
#define Beep_on				(1<<2)
#define start_botton		(1<<3)
#define counter_pass		(1<<4)
#define counter_fail		(1<<5)
#define target				(1<<7)
#define swcan_mode0			(1<<10)
#define swcan_mode1			(1<<11)
#define IGN_on				(1<<13)
#define battary_on			(1<<15)
#endif

#ifdef CONFIG_PDI_DM
#define LED_red_on			(1<<0)
#define LED_green_on		(1<<1)
#define target				(1<<2)
#define start_botton		(1<<3)
#define counter_pass		(1<<4)
#define counter_fail		(1<<5)
#define Beep_on				(1<<8)
#define swcan_mode0			(1<<10)
#define swcan_mode1			(1<<11)
#define battary_on			(1<<13)
#define IGN_on				(1<<15)
#endif

#ifdef CONFIG_PDI_BA
#define LED_red_on			(1<<0)
#define LED_green_on		(1<<1)
#define target				(1<<7)
#define start_botton		(1<<3)
#define counter_pass		(1<<4)
#define counter_fail		(1<<5)
#define Beep_on				(1<<8)
#define swcan_mode0			(1<<10)
#define swcan_mode1			(1<<11)
#define battary_on			(1<<13)
#define IGN_on				(1<<15)
#endif

#ifdef CONFIG_PDI_RC
#define LED_red_on			(1<<0)
#define LED_green_on		(1<<1)
#define target				(1<<7)
#define start_botton		(1<<3)
#define counter_pass		(1<<4)
#define counter_fail		(1<<5)
#define JAMA				(1<<2) 
#define Beep_on				(1<<8)
#define swcan_mode0			(1<<10)
#define swcan_mode1			(1<<11)
#define battary_on			(1<<13)
#define IGN_on				(1<<15)
#endif

#ifdef CONFIG_PDI_B515
#define start_botton		(1<<4)
#define counter_fail		(1<<5)
#define target				(1<<2)
#define counter_pass		(1<<3)
#define LED_red_on			(1<<0)
#define LED_green_on		(1<<1)
#define Beep_on				(1<<6)
#define swcan_mode0			(1<<10)
#define swcan_mode1			(1<<11)
#define battary_on			(1<<13)
#define IGN_on				(1<<15)
#endif

#ifdef CONFIG_PDI_J04N
#define start_botton		(1<<3)
#define counter_fail		(1<<4)
#define target				(1<<7)
#define counter_pass		(1<<5)
#define LED_red_on			(1<<1)
#define LED_green_on		(1<<0)
#define JAMA				(1<<2)
#define Beep_on				(1<<8)
#define swcan_mode0			(1<<10)
#define swcan_mode1			(1<<11)
#define battary_on			(1<<13)
#define IGN_on				(1<<15)
#endif

#ifdef CONFIG_PDI_RP
#define start_botton		(1<<3)
#define counter_fail		(1<<4)
#define target				(1<<2)
#define counter_pass		(1<<5)
#define LED_red_on			(1<<0)
#define LED_green_on		(1<<1)
#define Beep_on				(1<<8)
#define swcan_mode0			(1<<10)
#define swcan_mode1			(1<<11)
#define battary_on			(1<<9)
#define IGN_on				(1<<15)
#endif

#ifdef CONFIG_PDI_AP2
#define LED_red_on			(1<<0)
#define LED_green_on		(1<<1)
#define target				(1<<7)
#define start_botton		(1<<3)
#define counter_pass		(1<<4)
#define counter_fail		(1<<5)
#define Beep_on				(1<<8)
#define swcan_mode0			(1<<10)
#define swcan_mode1			(1<<11)
#define battary_on			(1<<13)
#define IGN_on				(1<<15)
#endif

#ifdef CONFIG_PDI_CMF1
#define LED_red_on			(1<<0)
#define LED_green_on		(1<<1)
#define target				(1<<7)
#define start_botton		(1<<3)
#define counter_pass		(1<<4)
#define counter_fail		(1<<5)
#define JAMA				(1<<2)
#define Beep_on				(1<<8)
#define swcan_mode0			(1<<10)
#define swcan_mode1			(1<<11)
#define battary_on			(1<<13)
#define IGN_on				(1<<15)
#endif

#ifdef CONFIG_PDI_BANEW
//PE
#define LED_red1_on			(1<<3)
#define LED_green1_on		(1<<2)
#define LED_red2_on			(1<<5)
#define LED_green2_on		(1<<4)
#define VALVE1              (1<<0)
#define VALVE2              (1<<1)
#define swcan_mode0		    (1<<10)
#define swcan_mode1			(1<<11)
//PB
#define Beep_on				(1<<5)
#ifdef Version2 
#define IGN1_on				(1<<9)        
#define IGN2_on				(1<<8)        
#else 
#define IGN1_on				(1<<15)        
#define IGN2_on				(1<<14)        
#endif 
#define BAT1_on				(1<<7)
#define BAT2_on				(1<<6)
//PD
#ifdef Version2 
#define CAN                 (1<<3)         
#else 
#define CAN                 (1<<4) 
#endif
#define CANx                (1<<4)
#define KLINE               (1<<5)
//PC
#define SPARE1              (1<<3)
#define SPARE2              (1<<2)
#define SPARE3              (1<<1)         
#define SPARE4              (1<<0)  
//PD
#define JAMA1               (1<<14)
#define JAMA2               (1<<15)
//PE
#define target1				(1<<7)
#define target2				(1<<6)
#endif

#if defined(CONFIG_PDI_CCP2) || defined(CONFIG_PDI_B562) || defined(CONFIG_PDI_B12L) ||(CONFIG_PDI_RCLE)
//PE
#define LED_red1_on			(1<<4)
#define LED_green1_on		(1<<5)
#define LED_red2_on			(1<<2)
#define LED_green2_on		(1<<3)
#define VALVE1              (1<<1)
#define VALVE2              (1<<0)

//PB
#define Beep_on				(1<<5)
#ifdef Version2 
#define IGN1_on				(1<<9)        
#define IGN2_on				(1<<8)        
#else 
#define IGN1_on				(1<<15)        
#define IGN2_on				(1<<14)        
#endif 
#define BAT1_on				(1<<7)
#define BAT2_on				(1<<6)
//PD
#ifdef Version2 
#define CAN                 (1<<3)         
#else 
#define CAN                 (1<<4) 
#endif
#define CANx                (1<<4)
#define KLINE               (1<<5)
#define swcan_mode0		    (1<<10)
#define swcan_mode1		    (1<<9)
//PC
#define SPARE1              (1<<3)
#define SPARE2              (1<<2)
#define SPARE3              (1<<1)         
#define SPARE4              (1<<0)  
//PD
#define JAMA1               (1<<15)
#define JAMA2               (1<<14)
//PE
#define target1				(1<<7)
#define target2				(1<<6)
#endif


#if defined(CONFIG_PDI_CHB0410) || defined(CONFIG_PDI_HZH) || defined(CONFIG_PDI_TL_NEW)
//PE
#define LED_yellow2_on			(1<<0)
#define LED_red2_on			(1<<1)
#define LED_green2_on		        (1<<2)
#define LED_yellow1_on			(1<<3)
#define LED_red1_on			(1<<4)
#define LED_green1_on		        (1<<5)
//#define VALVE1              (1<<1)
//#define VALVE2              (1<<0)

//PB
#define Beep_on				(1<<5)
//PC
#define SPARE1              (1<<3)
#define SPARE2              (1<<2)
#define SPARE3              (1<<1)         
#define SPARE4              (1<<0)  
#define BAT1_on				(1<<11)
#define BAT2_on				(1<<12)
//PD
#define IGN1_on				(1<<1)        
#define IGN2_on				(1<<0)  
#define JAMA1               (1<<15)
#define JAMA2               (1<<14)
#define target1				(1<<12)
#define target2				(1<<13)
#define CAN                 (1<<3)         
#define CANx                (1<<4)
#define KLINE               (1<<5)
#define swcan_mode0		    (1<<10)
#define swcan_mode1		    (1<<9)
#endif
//careful negative logic
int pdi_batt_on()
{
	POSITION = Position_Get();
	int bat = ((*POSITION == LEFT) ? BAT1_on : BAT2_on);
	GPIOB->ODR &= ~bat;
	return 0;
}

int pdi_batt_off()
{
	POSITION = Position_Get();
	int bat = ((*POSITION == LEFT) ? BAT1_on : BAT2_on);
	GPIOB->ODR |= bat;
	return 0;
}

int pdi_IGN_on()
{
	POSITION = Position_Get();
	int ign = ((*POSITION == LEFT) ? IGN1_on : IGN2_on);
	GPIOD->ODR &= ~ign;
	if(*POSITION == LEFT) {
		printf("Left power on success ! \r\n");
	}
	else {
		printf("Right power on success ! \r\n");
	}
	return 0;
}

int pdi_IGN_off()
{
	POSITION = Position_Get();
	int ign = ((*POSITION == LEFT) ? IGN1_on : IGN2_on);
	GPIOD->ODR |= ign;
	if(*POSITION == LEFT) {
		printf("\r\n");
		printf("Left power off success ! \r\n");
	}
	else {
		printf("Right power off success ! \r\n");
	}
	return 0;
}

int pdi_drv_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);

#ifdef CONFIG_PDI_SDM10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif

#ifdef CONFIG_PDI_DM
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif

#ifdef CONFIG_PDI_RC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif

#ifdef CONFIG_PDI_B515
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif

#ifdef CONFIG_PDI_J04N
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif

#ifdef CONFIG_PDI_RP
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif
        
#ifdef CONFIG_PDI_AP2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif

#ifdef CONFIG_PDI_CMF1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
        
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif
        
#ifdef CONFIG_PDI_BA
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
        
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif
#ifdef CONFIG_PDI_BANEW
	//OUT counter 1 2 LED1G LED1R LED2G LED2R uln2003
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//OUT SPARE4 3 2 1    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//OUT CAN CAN2 Kline relay    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//OUT Buzz BAT 1 2 IGN 1 2    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//IN Target 1 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//IN Jama 1 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
        
#endif
#if defined(CONFIG_PDI_CCP2) || defined(CONFIG_PDI_B562) || (defined(CONFIG_PDI_B12L)) || defined(CONFIG_PDI_RCLE)
	//OUT counter 1 2 LED1G LED1R LED2G LED2R uln2003
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//OUT SPARE4 3 2 1    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//OUT CAN CAN2 Kline relay    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//OUT Buzz BAT 1 2 IGN 1 2    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//IN Target 1 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//IN Jama 1 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
#endif    
#if defined(CONFIG_PDI_CHB041) || defined(CONFIG_PDI_HZH) || defined(CONFIG_PDI_TL_NEW)
	//OUT SPARE4 3 2 1   PC11,12->  BAT 2  1 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_0);
	GPIO_SetBits(GPIOD, GPIO_Pin_1);
	//OUT PD->0,1 IGN 2  1   PD3->CAN_c   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//OUT Buzz  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//IN Target 1 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//IN Jama 1 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);    
#endif    
	return 0;
}
#ifdef Version2
int beep_on()
{
	GPIOB->ODR |= Beep_on;
	return 0;
}
#else
int beep_on()
{
	GPIOE->ODR |= Beep_on;
	return 0;
}
#endif
#ifdef Version2
int beep_off()
{
	GPIOB->ODR &= ~Beep_on;
	return 0;
}
#else
int beep_off()
{
	GPIOE->ODR &= ~Beep_on;
	return 0;
}
#endif
/*int counter_pass_rise()
{
	GPIOE->ODR |= VALVE1;
	return 0;
}

int counter_pass_down()
{
	GPIOE->ODR &= ~VALVE1;
	return 0;
}

int counter_fail_rise()
{
	GPIOE->ODR |= VALVE2;
	return 0;
}

int counter_fail_down()
{
	GPIOE->ODR &= ~VALVE2;
	return 0;
}
*/
int target1_on()
{
	if((GPIOD->IDR & target1) == 0){
		for(int i = 10000; i > 0; i--);
                if((GPIOD->IDR & target1) == 0)
                    return 0;  //in the right position      
                else
                    return 1;
	}
	return 1;
}

int target2_on()
{        
	if((GPIOD->IDR & target2) == 0){
                for(int i = 10000; i > 0; i--);
                if((GPIOD->IDR & target2) == 0)
                    return 0;  //in the right position      
                else
                    return 1;
	}
	return 1;
}

int pdi_swcan_mode()
{
	GPIOE->ODR |= swcan_mode0;
	GPIOE->ODR |= swcan_mode1;
	return 0;
}
/*
int pdi_lock()
{
	POSITION = Position_Get();
	int valve = ((*POSITION == LEFT) ? VALVE1 : VALVE2);
	GPIOC->ODR |= valve;
	return 0;
}

int pdi_unlock()
{
	POSITION = Position_Get();
	int valve = ((*POSITION == LEFT) ? VALVE1 : VALVE2);
	GPIOC->ODR &= ~valve;
	return 0;
}
*/
int pdi_can()
{
	POSITION = Position_Get();
	//int can_c = ((POSITION == LEFT) ? (~CAN) : CAN);
	if(*POSITION == LEFT) {
		GPIOD->ODR |= CAN;
			printf("Left can !\r\n");
        }
	else {
		GPIOD->ODR &= ~CAN;
			printf("Right can !\r\n");
        }
	return 0;
}

int pdi_kline()
{
	POSITION = Position_Get();
	if(*POSITION == LEFT) {
		GPIOD->ODR |= KLINE;
		printf("Left Kline !\r\n");
	}
	else {
		GPIOD->ODR &= ~KLINE;
		printf("Right Kline !\r\n");
	}
	return 0;
}

#ifdef CONFIG_PDI_RC
int JAMA_on()
{
	if((GPIOE->IDR & JAMA) == 0)		//如果有JAMA返回为0，否则返回为1
		return 0;
	else
		return 1;
}
#endif

#ifdef CONFIG_PDI_J04N
int JAMA_on()
{
	if((GPIOD->IDR & JAMA) == 0)		//如果有JAMA返回为0，否则返回为1
		return 0;
	else
		return 1;

}
#endif

#ifdef CONFIG_PDI_CMF1
int JAMA_on()
{
	if((GPIOE->IDR & JAMA) == 0)		//如果有JAMA返回为0，否则返回为1
		return 0;
	else
		return 1;
}
#endif

#if (defined CONFIG_PDI_CCP2) || (defined CONFIG_PDI_B562)
int JAMA_on()
{
	POSITION = Position_Get();
	if(*POSITION == LEFT) {
		if((GPIOD->IDR & JAMA1) == 0) {		//如果有JAMA返回为0，否则返回为1
			printf("LEFT JAMA ON  \r\n");
			return 0;
		}
		else {
			printf("LEFT JAMA OFF \r\n");
			return 1;
		}
	} else {
		if((GPIOD->IDR & JAMA2) == 0) {		//如果有JAMA返回为0，否则返回为1
			printf("RIGHT JAMA ON \r\n");
			return 0;
		}	
		else {
			printf("RIGHT JAMA OFF \r\n");
			return 1;
		}
	}	
}
#endif
