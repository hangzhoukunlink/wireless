 
 
#include "config.h" 
#include "led.h" 
#include "stm32f10x.h" 
#define ECU_PDI 0 
#define RSU_PDI 1 
 
void led_hwInit(void) 
{ 
        GPIO_InitTypeDef GPIO_InitStructure; 
 
#if (CONFIG_TASK_MOTOR == 1) || (CONFIG_TASK_STEPMOTOR == 1) || (CONFIG_TASK_VVT == 1) || (CONFIG_TASK_SSS == 1) 
        /* Enable GPIOA,GPIOC clock */ 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
 
        /* Configure PC.10 as Output push-pull */ 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOC, &GPIO_InitStructure); 
 
        /* Configure PC.12 as Output push-pull */ 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOC, &GPIO_InitStructure); 
#elif CONFIG_CAN_BPMON == 1 
        //PC4->GREEN, PC5->RED 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOC, &GPIO_InitStructure); 
#elif CONFIG_MISC_VHD == 1 
        //PC6->red, PC7->yellow, PC8->green 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
 
        /* Configure PC.6, PC.7, PC.8 as Output push-pull */ 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOC, &GPIO_InitStructure); 
#elif CONFIG_TASK_PDI == 1 
        //RSU PDI 
        //BY1 -> PE5 BY2 -> PE4 BY3 -> PE3 BY4 -> PE2 
        //BG4 -> PE6 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE); 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 |GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOE, &GPIO_InitStructure); 
        //BG3 -> PD0  BG2 -> PD1  BG1 -> PD2  
        //BR4 -> PD3  BR2 -> PD4  BR2 -> PD5 BR1 -> PD6 
        //AY3 -> PD8  AY2 -> PD9  AY1 -> PD10 
        //AG4 -> PD11 AG3 -> PD12 AG2 -> PD13 AG1 -> PD14 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOD, &GPIO_InitStructure); 
        //AR3 -> PC6 AR2 -> PC7 AR1 -> PC8 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOC, &GPIO_InitStructure); 
        //AY4 -> PB15 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOB, &GPIO_InitStructure); 
#elif (CONFIG_IRT_PROBE == 1) 
        //PB10->red, PB11->green 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOB, &GPIO_InitStructure); 
#elif (CONFIG_MISC_CWB == 1) 
    //PE9->red, PE10->green 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 
 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOE, &GPIO_InitStructure); 
#elif CONFIG_MISC_ICT == 1 
        //PC8 led green 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOC, &GPIO_InitStructure); 
#elif CONFIG_BOARD_HY_SMART == 1 
        //PB1 led green 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOB, &GPIO_InitStructure); 
#elif CONFIG_MISC_MATRIX == 1 
        /*PA0 RLED, PA1 GLED*/ 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOA, &GPIO_InitStructure); 
#elif (CONFIG_OID_HWV2 == 1) || (CONFIG_YBS_MON == 1) || (CONFIG_IRT_MXC5324 == 1) 
        /*PC0 RLED, PC1 GLED*/ 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOC, &GPIO_InitStructure); 
#elif CONFIG_MISC_ALLERGO == 1 
        /*PC3 led red, PE5 led green, on on the same time is yellow*/ 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOC, &GPIO_InitStructure); 
         
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOE, &GPIO_InitStructure); 
#elif CONFIG_MISC_PC == 1 
        /*PC3 led red, PE5 led green, on on the same time is yellow*/ 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOC, &GPIO_InitStructure); 
         
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOE, &GPIO_InitStructure); 
#elif CONFIG_MISC_FFT == 1 
        /*PC3 led red, PE5 led green, on on the same time is yellow*/ 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOB, &GPIO_InitStructure); 
#elif CONFIG_MISC_PROBE == 1 
        /*PB10 -> GREEN PB11 -> RED*/ 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOB, &GPIO_InitStructure); 
#else 
        /*led_r pg8, led_g pg15*/ 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE); 
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_15; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
        GPIO_Init(GPIOG, &GPIO_InitStructure); 
#endif 
} 
 
void led_hwSetStatus(led_t led, led_status_t status) 
{ 
        BitAction ba; 
        switch(status) { 
                case OFF: 
                        ba = Bit_RESET; 
                        break; 
                case ON: 
                        ba = Bit_SET; 
                        break; 
                default: 
                        return; 
        } 
 
        switch(led) { 
 
                case LED_GREEN1: 

                        GPIO_WriteBit(GPIOE, GPIO_Pin_14, ba); 

                        break; 
                case LED_GREEN2: 
 
                        GPIO_WriteBit(GPIOE, GPIO_Pin_12, ba); 

                        break; 
                case LED_GREEN3: 

                        GPIO_WriteBit(GPIOE, GPIO_Pin_10, ba); 

                        break; 
                case LED_GREEN4: 

                        GPIO_WriteBit(GPIOE, GPIO_Pin_8, ba); 
                        
                        break; 
                case LED_RED1: 

                        GPIO_WriteBit(GPIOE, GPIO_Pin_15, ba); 

                        break; 
                case LED_RED2: 

                        GPIO_WriteBit(GPIOE, GPIO_Pin_13, ba); 
 
                        break;     
                case LED_RED3: 

                        GPIO_WriteBit(GPIOE, GPIO_Pin_11, ba); 
 
                        break; 
                case LED_RED4: 

                        GPIO_WriteBit(GPIOE, GPIO_Pin_9, ba); 

                        break; 
                case LED_YELLOW1: 

                        GPIO_WriteBit(GPIOE, GPIO_Pin_15, ba); 
                        GPIO_WriteBit(GPIOE, GPIO_Pin_14, ba); 
                        break; 
                case LED_YELLOW2: 

                        GPIO_WriteBit(GPIOE, GPIO_Pin_13, ba);  
                        GPIO_WriteBit(GPIOE, GPIO_Pin_12, ba); 
                        
                        break; 
                case LED_YELLOW3: 

                        GPIO_WriteBit(GPIOE, GPIO_Pin_11, ba);  
                        GPIO_WriteBit(GPIOE, GPIO_Pin_10, ba); 
                        
                        break; 
                case LED_YELLOW4: 

                        GPIO_WriteBit(GPIOE, GPIO_Pin_9, ba);  
                        GPIO_WriteBit(GPIOE, GPIO_Pin_8, ba); 

                        break; 

                default: 
                        break; 
        } 
}
