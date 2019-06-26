/*
 *	jiamao.gu@2012 initial version
 *      Communication between PDI_Control and AP2,send and receive the message by Uart. 
 *      Two different kinds of message ,one has checksum,and otherwise. 
 *      Timing is significant in debug. Send and receive function should in the same function.
 *      Message received from AP2 exist the echo,use sscanf function to solve.
 *      Send sequence message before require message. 
 */

#include <string.h>
#include "config.h"
#include "sys/task.h"
#include "ulp_time.h"
#include "can.h"
#include "drv.h"
#include "Mbi5025.h"
#include "ls1203.h"
#include "cfg.h"
#include "priv/usdt.h"
#include "ulp/debug.h"
#include "shell/cmd.h"
#include "led.h"
#include "pdi.h"
#include "stdio.h"
#include "AP2_Downloadfile.h"

#define PDI_DEBUG	1

// AP2 communication struct
typedef struct {
	uart_bus_t *bus;
	int data_len;
	time_t dead_time;
} AP2Communication_t;

//Kline struct
typedef struct {
        char Format;
	char Target;
        char Source;
        char Length;
	char *data;
        char Checksum;
} Kline_t;
typedef unsigned char (*AP2_Line_data)[16];//32 char a line mainly use in write a line function
typedef int AP2_Line_address;
// Two kinds of messages are sent to the unit, a sequence or a request. 
//If the message contains no parameter, it's a sequence, otherwise it's a request.
//sequence
static char start_sequence[20]="!EBCD4TEST!";
static char lock_sequence[12]="!ACU4LOCK!";
static char end_sequence[12]="!ACU4END!";
static char errorerase_sequence[20]="!ERASE_ERRORS!";
static char testerase_sequence[12]="!TESTERASED!";
//require
//static char errorcode_request[25]="R:20100F9C";          //error 2053B 082C0 2083F0
//static char softversion_request[25]="R:100082B3";
static char partnum_request[25]="R:01008110";             //TBC
static char errorreset_request[25]="R:022083E1";          //reset the buffer??
static char erroreraseram_request[25]="R:28300F9C";      //erase the error int ram TBC
static char erroreraseeeprom_request[25]="R:2832083F0";   //erase the error int eepram TBC
static char eepromunlock_request[25]="R:0500826A";       // only unlock can modify the eeprom TBC
static char eepromlock_request[25]="R:0500826A";         // once lock can not modify the eeprom TBC
static  Kline_t ap2_start_msg[10];                       // convert to kline message but not need in this project
static int flag=0;
static const ls1203_t pdi_ls1203 = {
		.bus = &uart2,
		.data_len = 11,//TBD
		.dead_time = 20,
};

static const AP2Communication_t pdi_ap2 = {
                .bus = &uart3,
		.data_len = 100,//TBD
		.dead_time = 50,
};

static char AP2_fault_buf[64];			//fault buffer
//get data from ECU
static int AP2_GetFault(char *data,int *num,int *num1);
/**************************************************************************/
/************         Local funcitons                         *************/
/**************************************************************************/

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
/* Description : Add checksum on frame,checksum convert to upper case*/
void AddACU3Checksum (char *wrMsg)
{
 int cpt;
 int varChecksum = 0;
 int checksum;
        //first two byte is length
 	for (cpt = 2; cpt < strlen (wrMsg); cpt++)
		varChecksum += wrMsg [cpt];
	checksum = 0x100 - (varChecksum %0x100);
	sprintf (wrMsg, "%s%02X", wrMsg, checksum & 0xFF);
}
/* Description : AP2 communicate initial*/
void AP2_Init(const AP2Communication_t *chip)
{
	uart_cfg_t cfg = { //UART_CFG_DEF;
		.baud = 4800,
	};
	chip->bus->init(&cfg);
} 
/* Description : convert the message in kline,no need*/
void init_kline_msg(Kline_t *rx_data,char *data)
{
    rx_data->data = data;
    rx_data->Length = strlen(data);
    rx_data->Format = 0x80;
     rx_data->Target = 0x58;
    rx_data->Source = 0xF1;
    //rx_data->Checksum = AddACU3Checksum(data); //not all the message need checksum
}
/* Description : Send senquece and require message,return the length*/
int ACU4KlineCommunication(const AP2Communication_t *chip,Kline_t *cmd_data,char *rx_data,int *length)
{
        time_t overtime;
        int  num_data=0;
        if((cmd_data->data[0] == 'R')||(cmd_data->data[0] == 'W')){ //require message need checksum read and write function
            AddACU3Checksum(cmd_data->data);
            for(int i = 0;i< strlen(cmd_data->data);i++){
                chip->bus->putchar((int)cmd_data->data[i]);
                        num_data++;
                        *length = num_data;
            }
        }
        else {                                                    //sequence message need no checksum
            for(int i = 0;i < cmd_data->Length;i++){
	        chip->bus->putchar((int)cmd_data->data[i]);
                num_data++;
                *length = num_data;
            }
        }
        pdi_mdelay(100);//this value is between 25 and 100
        RCC_ClocksTypeDef a;
        RCC_GetClocksFreq(&a);
        if (chip->bus->poll()) {
		for (int i = 0; i < chip->data_len; i++) {
			overtime = time_get(chip->dead_time);
			while (chip->bus->poll() == 0) {
				if (time_left(overtime) < 0)
					return 1;
			}
			rx_data[i] = chip->bus->getchar(); 
		}
                return 0;
	}
        return 1;
}

/* Description : whether Senquence message's return is posedge or negetive response*/
int Judge_Communication_Ok(char *rxdata)
{
        char temp[100] = {0};
        char kline[10] = {0};
        //sscanf(rxdata,"%*[^!]!%[^!]!",kline);
        sscanf( rxdata, "%*[^\r]%s", kline);    //filter the char before '>'
        if(kline[0] == '>') return 0;
        else return 1;
}
/* Description : get require message and deal with the ':'*/
int Get_Response(char *rxdata,char *kline)
{
        char temp[100] = {0};
        //sscanf(rxdata, "%*[^4]4%[^0]",temp);
        //sscanf(temp, "%*[^!]!%[^0]",kline);
        //sscanf(rxdata,"%*[^!]!%[^!]!",kline);
        sscanf( rxdata, "%*[^:]:%*[^:]:%s!", kline);//filter the char before ':'
}
int AP2_ClrDTC(void)
{
    char Kline[100];
    memset(Kline,0,100);
    int num;
    char data[100];
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    init_kline_msg(&ap2_start_msg[1],errorerase_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,&num);
   /* if(!Judge_Communication_Ok(Kline)) 
          printf("posedge response");
    else 
          printf("negedge response");*/
    
    pdi_mdelay(40);
    memset(Kline,0,100);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,&num);
    if(Kline[num] == 0x0d) 
      return 0;
    else 
      return 1;
}
/*Description : get the fault and num from ram*/
static int AP2_GetFault(char *data,int *num,int *num1)
{
    char Kline[100];
    memset(Kline,0,100);
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("posedge response \n");
    else 
      printf("negedge response \n");
    pdi_mdelay(50);
    memset(Kline,0,100);
    char errorcode_request[25]="R:20100F9C";    //error code ERR_astErrorBuffer
    init_kline_msg(&ap2_start_msg[1],errorcode_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
     for (int i = 0; i < 100; i++) {
                      printf("%x ",Kline[i]);
                }
    for(int j=0;j<strlen(Kline);j++){
          data[j] = Kline[j+(2*(*num)-3)];
          //printf("%02x, %02x\n",data[j],data[j+1]);
    }
    printf("##START##EC-");
    (*num1)=0;
    for(int i=0;i<strlen(data);i+=4){
          if((data[i] == 0x30) && (data[i+1] == 0x30))
            break;
          (*num1)++;
          printf("%02x,%02x,%02x,%02x\n",data[i],data[i+1],data[i+2],data[i+3]);          
    }
    printf("num of fault is: %d\n", *num1);
    printf("##END##\n");
}
/*Description : get the fault and num from eeprom*/
static int AP2_GetFault1(char *data,int *num,int *num1)
{
    char Kline[100];
    memset(Kline,0,100);
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("posedge response \n");
    else 
      printf("negedge response \n");
    pdi_mdelay(50);
    memset(Kline,0,100);
    char errorcode_request[25]="R:200083F0";    //error code EEP_ADR_ERR_ERROR
    init_kline_msg(&ap2_start_msg[1],errorcode_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
     for (int i = 0; i < 100; i++) {
                      printf("%x ",Kline[i]);
                }
    for(int j=0;j<strlen(Kline);j++){
          data[j] = Kline[j+(2*(*num)-3)];
          //printf("%02x, %02x\n",data[j],data[j+1]);
    }
    printf("##START##EC-");
    (*num1)=0;
    for(int i=0;i<strlen(data);i+=4){
          if((data[i] == 0x30) && (data[i+1] == 0x30))
            break;
          (*num1)++;
          printf("%02x,%02x,%02x,%02x\n",data[i],data[i+1],data[i+2],data[i+3]);          
    }
    printf("num of fault is: %d\n", *num1);
    printf("##END##\n");
}
/*Description : write aa to crash inhibited Need in unlock mode,but in fact not,do not know why*/
static int AP2_Write_crashinhibited(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100);
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("posedge response \n");
    else 
      printf("negedge response \n");
    pdi_mdelay(50);                  //timing between sequence and request 
    memset(Kline,0,100);
    char crashinhibited_request[25]="W:010083A2AA";
    init_kline_msg(&ap2_start_msg[1],crashinhibited_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
     for (int i = 0; i < 100; i++) {
                      printf("%x ",Kline[i]);
                }
     if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
        printf("\nposedge response write success\n");
     else 
        printf("\nnegedge response write fail\n");
        pdi_mdelay(20);          //wait for data write success
}
/*Description : write a line ASCII to 0x8000 judge whether work well
  Frame W:nnaaaaaadd..ddcc
  nn:	Number of characters to be written. 2 ASCII characters.
  aaaaaa:	Start address. 6 ASCII characters.
  dd..dd:	Consecutive data bytes. ASCII characters.
  cc:	Checksum.
*/
static int AP2_Write_Oneline(char *data,int *num,const char array[])
{
    char Kline[100];
    memset(Kline,0,100);
    //int *tabaddress = (int*)(&apn1scpd);//little-endian
    int *tabaddress = (int*)(array);//little-endian
    AP2_Line_data tabdata ;
    //tabdata = (AP2_Line_data)(&apn1scpd[4]);//get first data address
    tabdata = (AP2_Line_data)(&array[4]);//get first data address
    char Firstline_request[100];
    int blocLength = 16;
    int blocLine = 128;
    for (int cpt = 0; cpt < blocLine; cpt++) {
        memset(Firstline_request,0,100);
        sprintf (Firstline_request, "W:%02X%06X", blocLength,(*tabaddress+0x10*cpt));    //+0x10*cpt 
        for(int i=0;i < blocLength;i++){
            sprintf (Firstline_request,"%s%02X",Firstline_request,(*tabdata)[i]);
        }
        init_kline_msg(&ap2_start_msg[1],Firstline_request);
        ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
        if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
             printf("\nposedge response write one line success %x",(*tabaddress+0x10*cpt));
        else {
             printf("\nnegedge response write one line fail %x",(*tabaddress+0x10*cpt));
             pdi_fail_action();
             flag=1;
             break;
        }
        pdi_mdelay(20);          //wait for data write success
        tabdata++;
        //printf("%02x",(*tabdata)[cpt]);
    }
    //tabdata = (AP2_Line_data)(&apn1scpd[4]);
     for (int i = 0; i < 100; i++) {
           printf("%x ",Kline[i]);
     }
}
/*Description write a data to reset the count*/
static int AP2_Write_resetcount(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100);
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("posedge response \n");
    else 
      printf("negedge response \n");
    pdi_mdelay(50);                  //timing between sequence and request 
    memset(Kline,0,100);
    char resetcount_request[25]="W:010083DC00"; //write 00 to 0x0083DC
    init_kline_msg(&ap2_start_msg[1],resetcount_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
     for (int i = 0; i < 100; i++) {
                      printf("%x ",Kline[i]);
                }
     if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
        printf("\nposedge response reset count success\n");
     else 
        printf("\nnegedge response reset count fail\n");
        pdi_mdelay(20);          //wait for data write success
}
/*Description write lifetime 00*/
static int AP2_Write_lifetime(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100);
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("posedge response \n");
    else 
      printf("negedge response \n");
    pdi_mdelay(50);                  //timing between sequence and request 
    memset(Kline,0,100);
    char lifetime_request[25]="W:040083DD00000000"; //write 00 to 0x0083DC
    init_kline_msg(&ap2_start_msg[1],lifetime_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
     for (int i = 0; i < 100; i++) {
                      printf("%x ",Kline[i]);
                }
     if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
        printf("\nposedge response life time success\n");
     else 
        printf("\nnegedge response life timet fail\n");
        pdi_mdelay(20);          //wait for data write success
}
/*Description write lifetime 00*/
static int AP2_Write_wramp(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100);
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("posedge response \n");
    else 
      printf("negedge response \n");
    pdi_mdelay(50);                  //timing between sequence and request 
    memset(Kline,0,100);
    char wramp_request[25]="W:040083E300000000"; //write 00 to 0x0083DC
    init_kline_msg(&ap2_start_msg[1],wramp_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
     for (int i = 0; i < 100; i++) {
                      printf("%x ",Kline[i]);
                }
     if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
        printf("\nposedge response wramp success\n");
     else 
        printf("\nnegedge response wramp fail\n");
        pdi_mdelay(20);          //wait for data write success
}
/*Description Read one byte just test the correction
*/
static int Read_Onebyte(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100);
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    char onebyte_request[25]="R:01008001";
    init_kline_msg(&ap2_start_msg[0],onebyte_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    pdi_mdelay(50);
    for(int j=0;j<strlen(Kline);j++){
          data[j]= Kline[j+(2*(*num)-3)];
    }
    for(int i=0;i<strlen(data);i+=4){
          if(data[i] == 0x0d)
            break;
          printf("%02x,%02x,%02x,%02x\n",data[i],data[i+1],data[i+2],data[i+3]); 
    }
    pdi_mdelay(20);          //wait for data write success
}
/*Description : Read the traceability*/
static int AP2_Readtraceability(char *data,int *num,char *data_out)
{
    char Kline[100];
    memset(Kline,0,100);
    char traceability_request[25]="R:1400830B";
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    init_kline_msg(&ap2_start_msg[1],traceability_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    pdi_mdelay(50);
    for(int j=0;j<strlen(Kline);j++){
          data[j]= Kline[j+(2*(*num)-3)];
         // printf("PPPPPPP");
         // printf("%02x\n",data[j]);
    }
   for(int i=0;i<36;i++){
          //if(data[i] == 0x0d)
            data_out[i]=data[i];
            //break;
          //printf("%02x,%02x,%02x,%02x\n",data_out[i],data_out[i+1],data_out[i+2],data_out[i+3]); 
    }
    pdi_mdelay(20);          //wait for data write success  
}
/*Description : Write the traceability*/
static int AP2_Writetraceability(char *data,int *num,char data_in[])
{
    char Kline[100];
    memset(Kline,0,100);
    //char traceability_request[25]="W:0200830B3336";
    char traceability_request[100];
    int blocLength = 36;
    int a = 18;
    memset(traceability_request,0,100);
    sprintf (traceability_request, "W:%02X%06X", a,0x00830B);    //+0x10*cpt 
    for(int i=1;i < blocLength;i=i+2){
        sprintf (traceability_request,"%s%X",traceability_request,data_in[i]);
    }
    init_kline_msg(&ap2_start_msg[1],traceability_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
       printf("\nposedge response write traceability success");
    else {
       printf("\nnegedge response write traceability fail");
    }
    pdi_mdelay(20);          //wait for data write success
    for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
    }
}
/*Description : Write the flag only two address*/
static int AP2_Writeflag(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100); 
    char flag1_request[25]="W:010082EB4A";
    char flag2_request[25]="W:010082EA53";
    char flag3_request[25]="W:010082E945";
    init_kline_msg(&ap2_start_msg[0],flag1_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
       printf("\nposedge response write flag1 success");
    else {
       printf("\nnegedge response write flag1 fail");
    }
    pdi_mdelay(20);          //wait for data write success
    for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
    }
    init_kline_msg(&ap2_start_msg[1],flag2_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
       printf("\nposedge response write flag2 success");
    else {
       printf("\nnegedge response write flag2 fail");
    }
    pdi_mdelay(20);          //wait for data write success
    for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
    }
    init_kline_msg(&ap2_start_msg[2],flag3_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[2],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
       printf("\nposedge response write flag3 success");
    else {
       printf("\nnegedge response write flag3 fail");
    }
    pdi_mdelay(20);          //wait for data write success
    for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
    }
}
/*Description : shutdown just for unlock mode and download the file*/
static int AP2_ShutDown(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100);
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
   /* if(!Judge_Communication_Ok(Kline)) 
          printf("posedge response");
    else 
          printf("negedge response");*/
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("posedge response start success\n");
    else 
      printf("negedge response start fail\n");
    pdi_mdelay(50);
    memset(Kline,0,100);
    char shutdown_request[25]="T:03000";
    init_kline_msg(&ap2_start_msg[1],shutdown_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("posedge response shutdown success\n");
    else 
      printf("negedge response shutdown fail\n");
     for (int i = 0; i < 100; i++) {
                      printf("%x ",Kline[i]);
                }
    for(int j=0;j<strlen(Kline);j++){
          data[j] = Kline[j+(2*(*num)-3)];
          //printf("%02x, %02x\n",data[j],data[j+1]);
    }
    for(int i=0;i<strlen(data);i+=4){
         // if((data[i] == 0x30) && (data[i+1] == 0x30))
          //  break;
          printf("%02x,%02x,%02x,%02x\n",data[i],data[i+1],data[i+2],data[i+3]);          
    }
    printf("\n------\n");
   /* pdi_mdelay(550);
    memset(Kline,0,100);
    char unlockeeprom[40] = "!ACU4UNLOCKFLGDAAMCBLVFLHDM!";
    init_kline_msg(&ap2_start_msg[2],unlockeeprom);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[2],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("posedge response unlockeeprom success\n");
    else 
      printf("negedge response unlockeeprom fail\n");
     for (int i = 0; i < 100; i++) {
      printf("%x ",Kline[i]);
      }*/
    pdi_mdelay(20);          //wait for data write success
    /*no need to end*/
    //init_kline_msg(&ap2_start_msg[2],end_sequence);
    //ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[2],Kline,num);
    printf("Shutdownend\n");
}
/*Description : not shutdown*/
static int AP2_NotShutDown(char *data,int *num)
{
    char Kline[100];
    /*memset(Kline,0,100);
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
        printf("posedge response \n");
    else 
        printf("negedge response \n");
    pdi_mdelay(50);*/
    memset(Kline,0,100);
    char shutdown_request[25]="T:05000";
    init_kline_msg(&ap2_start_msg[1],shutdown_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("posedge response \n");
    else 
      printf("negedge response \n");
     for (int i = 0; i < 100; i++) {
                      printf("%x ",Kline[i]);
                }
    pdi_mdelay(20);          //wait for data write success
    /*no need to end*/
    //init_kline_msg(&ap2_start_msg[2],end_sequence);
    //ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[2],Kline,num);
    printf("return back\n");
}
/**Description:Unlock eeprom*/
static int AP2_Unlockeeprom(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100);
     char unlockeeprom[40] = "!ACU4UNLOCKFLGDAAMCBLVFLHDM!";
    /*init_kline_msg(&ap2_start_msg[0],start_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("posedge response \n");
    else 
      printf("negedge response \n");
    for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
    }
    memset(Kline,0,100); 
    printf("\n++++++++\n");
    pdi_mdelay(100);*/
    init_kline_msg(&ap2_start_msg[1],unlockeeprom);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("\nposedge response \n");
    else 
      printf("\nnegedge response \n");
     for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
     }
     pdi_mdelay(20);          //wait for data write success
     printf("Unlockeeprom end\n");
}
/**Description:lock eeprom*/
static int AP2_Lockeeprom(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100);
     char lockeeprom[20] = "!ACU4LOCK!";
    /*init_kline_msg(&ap2_start_msg[0],start_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("posedge response \n");
    else 
      printf("negedge response \n");
    for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
    }
    memset(Kline,0,100); 
    printf("\n++++++++\n");
    pdi_mdelay(50);*/
    init_kline_msg(&ap2_start_msg[1],lockeeprom);
      ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
      printf("\nposedge response lock success\n");
    else 
      printf("\nnegedge response lock fail\n");
     for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
     }
     pdi_mdelay(20);          //wait for data write success
     printf("\nlockeeprom end\n");
}
/**Description:Factory Unlock*/
static int AP2_UnlockFactory(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100);
    char unlocfactory[40] = "R:05008304";
    char unlockeeprom[40] = "!ACU4UNLOCKFLGDAAMCBLVFLHDM!";
    init_kline_msg(&ap2_start_msg[0],unlocfactory);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
    }
    if(Kline[30]=='0'){
      init_kline_msg(&ap2_start_msg[2],unlockeeprom);
      ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[2],Kline,num);
      if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
        printf("posedge response unlockeeprom success\n");
      else 
       printf("negedge response unlockeeprom fail\n");
      pdi_mdelay(20);          //wait for data write success
    }
    printf("\n****UnlockFactory****\n");
}
/**Description:Factory Unlock*/
static int AP2_lockFactory(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100);
    char unlocfactory[40] = "R:05008304";
    char unlockeeprom[40] = "!ACU4UNLOCKFLGDAAMCBLVFLHDM!";
    init_kline_msg(&ap2_start_msg[0],unlocfactory);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
    }
    /*if(Kline[30]=='0'){
      init_kline_msg(&ap2_start_msg[2],unlockeeprom);
      ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[2],Kline,num);
      if((Kline[*num] == 0x0d)&&(Kline[(*num+1)] == 0x0a)&&(Kline[(*num+2)] == '>')) 
        printf("posedge response unlockeeprom success\n");
      else 
       printf("negedge response unlockeeprom fail\n");
      pdi_mdelay(20);          //wait for data write success
    }*/
    printf("\n****lockFactory****\n");
}
/*Descrition:crash inhibited return hex,but should return ASCII*/
static int AP2_crashinhibited(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100);
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    char crashinhibited_request[25]="R:01008383";
    init_kline_msg(&ap2_start_msg[1],crashinhibited_request);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    pdi_mdelay(50);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
    }
    printf("***********\n");
    for(int j=0;j<strlen(Kline);j++){
          data[j]= Kline[j+(2*(*num)-3)];
    }
    for(int i=0;i<strlen(data);i+=4){
          if(data[i] == 0x0d)
            break;
          printf("%02x,%02x,%02x,%02x\n",data[i],data[i+1],data[i+2],data[i+3]); 
    }
    printf("\n");
    init_kline_msg(&ap2_start_msg[2],end_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[2],Kline,num);
    pdi_mdelay(20);          //wait for data write success
    printf("Crash inhibited end \n");
}
/*Descrition:Read spuib line ,teturn 06 or 0F*/
static int AP2_Readsquibline(char *data,int *num,char *data_out)
{
    char Kline[100];
    memset(Kline,0,100);
    //init_kline_msg(&ap2_start_msg[0],start_sequence);
    char squibline_request[25]="R:010083A0";
    init_kline_msg(&ap2_start_msg[1],squibline_request);
    //ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    //pdi_mdelay(50);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
    }
    printf("***********\n");
    for(int j=0;j<strlen(Kline);j++){
          data[j]= Kline[j+(2*(*num)-3)];
    }
    data_out[0]=data[0];//first two byte 06->2loop 0F->4loop
    data_out[1]=data[1];
    printf("\n");
    pdi_mdelay(20);          //wait for data write success
    printf("Squib line read end \n");
}
/*Descrition:Read RSU Config ,teturn 00 or 03*/
static int AP2_Readrsuconfig(char *data,int *num,char *data_out)
{
    char Kline[100];
    memset(Kline,0,100);
    //init_kline_msg(&ap2_start_msg[0],start_sequence);
    char rsuconfig_request[25]="R:0100820A";
    init_kline_msg(&ap2_start_msg[1],rsuconfig_request);
    //ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    //pdi_mdelay(50);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
    }
    printf("***********\n");
    for(int j=0;j<strlen(Kline);j++){
          data[j]= Kline[j+(2*(*num)-3)];
    }
    data_out[0]=data[0];//first two byte 00->2loop 03->4loop
    data_out[1]=data[1];
    printf("\n");
    pdi_mdelay(20);          //wait for data write success
    printf("Rsu config read end \n");
}
/*Descrition:Read senseing axiel ,teturn 05*/
static int AP2_Readacaconfig(char *data,int *num,char *data_out)
{
    char Kline[100];
    memset(Kline,0,100);
    //init_kline_msg(&ap2_start_msg[0],start_sequence);
    char acaconfig_request[25]="R:01008257";
    init_kline_msg(&ap2_start_msg[1],acaconfig_request);
    //ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    //pdi_mdelay(50);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    for (int i = 0; i < 100; i++) {
         printf("%x ",Kline[i]);
    }
    printf("***********\n");
    for(int j=0;j<strlen(Kline);j++){
          data[j]= Kline[j+(2*(*num)-3)];
    }
    data_out[0]=data[0];//only return 05
    data_out[1]=data[1];
    printf("\n");
    pdi_mdelay(20);          //wait for data write success
    printf("ACA config read end \n");
}
/*Read Soft information*/
static int Read_softinformation(char *data,int *num)
{
    char Kline[100];
    memset(Kline,0,100);
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    char softinformation_request[25]="I:01";
    init_kline_msg(&ap2_start_msg[1],softinformation_request);;
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    /*for(int j=0;j<strlen(Kline);j++){
          data[j]= Kline[j+(2*(*num)-3)];
    }*/
    for(int i=0;i<12;i++){
      data[i]=Kline[i+(2*(*num)-2)];
      printf("%02X",data[i]);
    }
    pdi_mdelay(20);          //wait for data write success
}
/*Read Soft version*/
static int Read_softverison(char *data,int *num)
{
char Kline[100];
    memset(Kline,0,100);
    init_kline_msg(&ap2_start_msg[0],start_sequence);
    char softversion_request[25]="R:100082B3";
    init_kline_msg(&ap2_start_msg[1],softversion_request);
    char softversion1_request[25]="R:020082BD";
    init_kline_msg(&ap2_start_msg[2],softversion1_request);
    init_kline_msg(&ap2_start_msg[3],end_sequence);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[0],Kline,num);
    pdi_mdelay(50);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[1],Kline,num);
    /*for(int j=0;j<strlen(Kline);j++){
          data[j]= Kline[j+(2*(*num)-3)];
    }*/
    for(int i=0;i<20;i++){
      data[i]=Kline[i+(2*(*num)-3)];
    }
    /*for(int i=0;i<strlen(data);i+=4){
          if(data[i] == 0x0d)
            break;
          printf("%02x,%02x,%02x,%02x\n",data[i],data[i+1],data[i+2],data[i+3]); 
    }*/
    memset(Kline,0,100);
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[2],Kline,num);
    for(int i=0;i<4;i++){
      data[(20+i)]= Kline[i+(2*(*num)-3)];
    }
    for(int i=0;i<24;i=i+4){
      printf("%02X,%02X,%02X,%02X\n",data[i],data[i+1],data[i+2],data[i+3]);
    }
    pdi_mdelay(20);          //wait for data write success
    ACU4KlineCommunication(&pdi_ap2,&ap2_start_msg[3],Kline,num);
}
static int ap2_check_softverion(const struct pdi_cfg_s *sr)
{
        char Kline[100];
        memset(Kline,0,100);
        int j;
        Read_softverison(Kline,&j);
	const struct pdi_rule_s* pdi_cfg_rule;
	int i;
	for(i = 0; i < sr->nr_of_rules; i ++) {
		pdi_cfg_rule = pdi_rule_get(sr, i);
		if (&pdi_cfg_rule == NULL) {
			printf("##START##EC-no soft version rule...##END##\n");
			return 1;
		}
		switch(pdi_cfg_rule->type) {
		case PDI_RULE_PART:
			printf("##START##EC-Checking soft version...##END##\n");
			if(pdi_verify(pdi_cfg_rule, Kline) == 0)
			    continue;
		        else 
                            return 1;
                        break;
		case PDI_RULE_UNDEF:
			return 1;
		}	
	}

	return 0;
}
static int ap2_check_softinformation(const struct pdi_cfg_s *sr)
{
        char Kline[100];
        memset(Kline,0,100);
        int j;
        Read_softinformation(Kline,&j);
	const struct pdi_rule_s* pdi_cfg_rule;
	int i;
	for(i = 0; i < sr->nr_of_rules; i ++) {
		pdi_cfg_rule = pdi_rule_get(sr, i);
		if (&pdi_cfg_rule == NULL) {
			printf("##START##EC-no soft version rule...##END##\n");
			return 1;
		}
		switch(pdi_cfg_rule->type) {
		case PDI_RULE_PART:
			printf("##START##EC-Checking soft version...##END##\n");
			if(pdi_verify(pdi_cfg_rule, Kline) == 0)
			    continue;
		        else 
                            return 1;
                        break;
		case PDI_RULE_UNDEF:
			return 1;
		}	
	}

	return 0;
}
static int ap2_error_check(char *fault, int *num, const struct pdi_cfg_s *sr)
{
	int i, j, flag;
	char rp_fault_temp[4];
	const struct pdi_rule_s* pdi_cfg_rule;
	for(i = 0; i < *num; i ++) {
		flag = 0;
		rp_fault_temp[0] = fault[i*4];
		rp_fault_temp[1] = fault[(i*4) + 1];
                rp_fault_temp[2] = fault[(i*4) + 2];
		rp_fault_temp[3] = fault[(i*4) + 3];
#if PDI_DEBUG
		printf("rp_fault_temp:%x\n", rp_fault_temp[0]);//chang
		printf("rp_fault_temp:%x\n", rp_fault_temp[1]);//chang
		printf("%d\n",i);//chang
#endif
		for(j = 0; j < sr -> nr_of_rules; j ++) {
			pdi_cfg_rule = pdi_rule_get(sr, j);
			if(&pdi_cfg_rule == NULL) return 1;
			switch(pdi_cfg_rule -> type) {
				case PDI_RULE_ERROR:
					if(pdi_verify(pdi_cfg_rule, rp_fault_temp) == 0) {
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
int main(void)
{
        const struct pdi_cfg_s* pdi_cfg_file;
	char bcode[11], temp[2],bcode_1[3];
       /* char PN[36]={'3','6','3','2','3','3','3','0','3','6',
                      '3','1','3','8','3','0','3','0','3','0',
                      '3','0','3','0','3','1','3','4','3','2',
                       '3','1','3','6','3','6'};*/
        char PN[36];        //serial number
        char value[2];      //put squib line rsu config and sense axiel
        int rate;
        char Kline[100];
        memset(Kline,0,100);
	int num=0;
        int num1=0;
	/*init all*/
        ulp_init();
	pdi_drv_Init();	//led,beep
	pdi_relay_init();//scanner
	ls1203_Init(&pdi_ls1203);
        //pdi_IGN_on();
        AP2_Init(&pdi_ap2);       
        /*init action*/
	pre_check_action();
        //just for test
        /*char a[2] = {0x00,0x80};
        int *b=(int*)&a;
        printf("%x",*b);*/
        while(1)
        {
        	memset(bcode, 0, sizeof(bcode));
		while(ls1203_Read(&pdi_ls1203, bcode)) {
			ulp_update();
			if(target_on())
                        start_botton_on();
			else start_botton_off();
		}
                pdi_IGN_on();
                pdi_mdelay(1000);
		/*action start checking*/
		start_action();
		/*barcode processing*/
		bcode[11] = '\0';
		printf("##START##SB-");
		printf(bcode,"\0");
		printf("##END##\n");
		//bcode[5] = '\0';
                bcode_1[0]=bcode[4];
                bcode_1[1]=bcode[5];
                bcode_1[2]='\0';
		printf("##START##STATUS-5##END##\n");
		/*check ECU ready*/
		if(!target_on()) {
			pdi_noton_action();
			printf("##START##EC-Target is not on the right position##END##\n");
			continue;
		}

		/*get cfg file */
		pdi_cfg_file = pdi_cfg_get(bcode_1);

		/*whether or not the config file exist*/
		if(pdi_cfg_file == NULL) {
			pdi_fail_action();
			printf("##START##EC-No This Config File##END##\n");
			continue;
		}

		/*if ECU on,set relays*/
		pdi_set_relays(pdi_cfg_file);
                /*file download*/
                if((bcode[4]=='A')&&(bcode[5]=='2')){
                  pdi_IGN_on();
                  printf("PW951328");
                  memset(Kline,0,100);
                  AP2_ShutDown(Kline,&num);
                  memset(Kline,0,100);
                  AP2_UnlockFactory(Kline,&num);
                  memset(Kline,0,100);
                  memset(PN,0,36);
                  AP2_Readtraceability(Kline,&num,PN);
                  memset(Kline,0,100);
                  Read_softinformation(Kline,&num);
                  if(Kline[11] != '4') {
                    pdi_fail_action();
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  AP2_Write_Oneline(Kline,&num,PW951328);
                  memset(Kline,0,100);
                  AP2_Writetraceability(Kline,&num,PN);
                  memset(Kline,0,100);
                  AP2_Writeflag(Kline,&num);
                  memset(Kline,0,100);
                  memset(value,0,2);
                  AP2_Readsquibline(Kline,&num,value);
                  if(value[1]!='6'){ //6 -> 2loops F -> 4loops
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  memset(value,0,2);
                  AP2_Readrsuconfig(Kline,&num,value);
                  if(value[1]!='0'){ //0 -> 2loops 3 -> 4loops
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  memset(value,0,2);
                  AP2_Readacaconfig(Kline,&num,value);
                  if(value[1]!='5'){ // 5 -> 2 or 4 loops
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  AP2_Write_resetcount(Kline,&num);
                  memset(Kline,0,100);
                  AP2_Write_lifetime(Kline,&num);
                  memset(Kline,0,100);
                  AP2_Write_wramp(Kline,&num);
                  memset(Kline,0,100);
                  AP2_Write_crashinhibited(Kline,&num);
                  pdi_IGN_off();
                  pdi_mdelay(1000);
                  pdi_IGN_on();
                  pdi_mdelay(1000);
                  memset(Kline,0,100);
                  AP2_lockFactory(Kline,&num);
                  memset(Kline,0,100);
                  AP2_Lockeeprom(Kline,&num);//lock cmd
                  memset(Kline,0,100);
                  AP2_NotShutDown(Kline,&num); 
                  //pdi_IGN_off();
                  //memset(Kline,0,100);
                  //Read_Onebyte(Kline,&num);      	 
                }
                if((bcode[4]=='A')&&(bcode[5]=='4')){
                  pdi_IGN_on();
                  printf("PW951329");
                  AP2_ShutDown(Kline,&num); //include unlock cmd
                  memset(Kline,0,100);
                  AP2_UnlockFactory(Kline,&num);
                  memset(Kline,0,100);
                  memset(PN,0,36);
                  AP2_Readtraceability(Kline,&num,PN);
                  memset(Kline,0,100);
                  Read_softinformation(Kline,&num);
                  if(Kline[11] != '4') {
                    pdi_fail_action();
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  AP2_Write_Oneline(Kline,&num,PW951329);
                  memset(Kline,0,100);
                  AP2_Writetraceability(Kline,&num,PN);
                  memset(Kline,0,100);
                  AP2_Writeflag(Kline,&num);
                  memset(Kline,0,100);
                  memset(value,0,2);
                  AP2_Readsquibline(Kline,&num,value);
                  if(value[1]!='F'){ //6 -> 2loops F -> 4loops
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  memset(value,0,2);
                  AP2_Readrsuconfig(Kline,&num,value);
                  if(value[1]!='0'){ //0 -> 2loops 3 -> 4loops
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  memset(value,0,2);
                  AP2_Readacaconfig(Kline,&num,value);
                  if(value[1]!='5'){ // 5 -> 2 or 4 loops
                    goto ERR_x;
                  }
                  AP2_Write_resetcount(Kline,&num);
                  memset(Kline,0,100);
                  AP2_Write_lifetime(Kline,&num);
                  memset(Kline,0,100);
                  AP2_Write_wramp(Kline,&num);
                  memset(Kline,0,100);
                  memset(Kline,0,100);
                  AP2_Write_crashinhibited(Kline,&num);
                  pdi_IGN_off();
                  pdi_mdelay(1000);
                  pdi_IGN_on();
                  pdi_mdelay(1000);
                  memset(Kline,0,100);
                  AP2_Lockeeprom(Kline,&num);//lock cmd
                  memset(Kline,0,100);
                  AP2_NotShutDown(Kline,&num);
                  //memset(Kline,0,100);
                  //Read_Onebyte(Kline,&num);      	
                }
                if((bcode[4]=='N')&&(bcode[5]=='R')){
                  pdi_IGN_on();
                  printf("PW951295");
                  AP2_ShutDown(Kline,&num); //include unlock cmd
                  memset(Kline,0,100);
                  AP2_UnlockFactory(Kline,&num);
                  memset(PN,0,36);
                  memset(Kline,0,100);
                  AP2_Readtraceability(Kline,&num,PN);
                  memset(Kline,0,100);
                  Read_softinformation(Kline,&num);
                  if(Kline[11] != '4') {
                    pdi_fail_action();
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  AP2_Write_Oneline(Kline,&num,PW951295);
                  memset(Kline,0,100);
                  AP2_Writetraceability(Kline,&num,PN);
                  AP2_Writeflag(Kline,&num);
                  memset(Kline,0,100);
                  memset(value,0,2);
                  AP2_Readsquibline(Kline,&num,value);
                  if(value[1]!='6'){ //6 -> 2loops F -> 4loops
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  memset(value,0,2);
                  AP2_Readrsuconfig(Kline,&num,value);
                  if(value[1]!='0'){ //0 -> 2loops 3 -> 4loops
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  memset(value,0,2);
                  AP2_Readacaconfig(Kline,&num,value);
                  if(value[1]!='5'){ // 5 -> 2 or 4 loops
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  AP2_Write_resetcount(Kline,&num);
                  memset(Kline,0,100);
                  AP2_Write_lifetime(Kline,&num);
                  memset(Kline,0,100);
                  AP2_Write_wramp(Kline,&num);
                  memset(Kline,0,100);
                  memset(Kline,0,100);
                  AP2_Write_crashinhibited(Kline,&num);
                  pdi_IGN_off();
                  pdi_mdelay(1000);
                  pdi_IGN_on();
                  pdi_mdelay(1000);
                  memset(Kline,0,100);
                  AP2_Lockeeprom(Kline,&num);//lock cmd
                  memset(Kline,0,100);
                  AP2_NotShutDown(Kline,&num);
                  //pdi_IGN_off();
                  //memset(Kline,0,100);
                  //Read_Onebyte(Kline,&num);      	
                }
                if((bcode[4]=='N')&&(bcode[5]=='S')){
                  pdi_IGN_on();
                  printf("PW951803");
                  memset(Kline,0,100);
                  AP2_Readtraceability(Kline,&num,PN);
                  memset(Kline,0,100);
                  AP2_Writetraceability(Kline,&num,PN);
                  memset(Kline,0,100);
                  AP2_Readtraceability(Kline,&num,PN);
                  pdi_IGN_off();
                }
                if((bcode[4]=='N')&&(bcode[5]=='T')){
                  pdi_IGN_on();
                  printf("PW951296");
                  AP2_ShutDown(Kline,&num); //include unlock cmd
                  memset(Kline,0,100);
                  AP2_UnlockFactory(Kline,&num);
                  memset(PN,0,36);
                  memset(Kline,0,100);
                  AP2_Readtraceability(Kline,&num,PN);
                  memset(Kline,0,100);
                  Read_softinformation(Kline,&num);
                  if(Kline[11] != '4') {
                    pdi_fail_action();
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  AP2_Write_Oneline(Kline,&num,PW951296);
                  memset(Kline,0,100);
                  AP2_Writetraceability(Kline,&num,PN);
                  AP2_Writeflag(Kline,&num);
                  memset(Kline,0,100);
                  memset(value,0,2);
                  AP2_Readsquibline(Kline,&num,value);
                  if(value[1]!='F'){ //6 -> 2loops F -> 4loops
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  memset(value,0,2);
                  AP2_Readrsuconfig(Kline,&num,value);
                  if(value[1]!='0'){ //0 -> 2loops 3 -> 4loops
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  memset(value,0,2);
                  AP2_Readacaconfig(Kline,&num,value);
                  if(value[1]!='5'){ // 5 -> 2 or 4 loops
                    goto ERR_x;
                  }
                  memset(Kline,0,100);
                  AP2_Write_resetcount(Kline,&num);
                  memset(Kline,0,100);
                  AP2_Write_lifetime(Kline,&num);
                  memset(Kline,0,100);
                  AP2_Write_wramp(Kline,&num);
                  memset(Kline,0,100);
                  memset(Kline,0,100);
                  AP2_Write_crashinhibited(Kline,&num);
                  memset(Kline,0,100);
                  /*if(ap2_check_softinformation(pdi_cfg_file)){
                  //printf("1");//wrong!
                        pdi_fail_action();
	                printf("##START##EC-Soft information wrong...##END##\n");
			continue;
                }*/
                  //Read_softinformation(Kline,&num);
                  pdi_IGN_off();
                  pdi_mdelay(1000);
                  pdi_IGN_on();
                  pdi_mdelay(1000);
                  AP2_Lockeeprom(Kline,&num);//lock cmd
                  memset(Kline,0,100);
                  AP2_NotShutDown(Kline,&num); 
                  //pdi_IGN_off();
                  //memset(Kline,0,100);
                  //Read_Onebyte(Kline,&num);      	
                }
                pdi_IGN_off();
                /*delay 7s*/
		for (rate = 17; rate <= 60; rate ++) {
			pdi_mdelay(89);
		} 

		/*ECU power on*/
		pdi_IGN_on();
                /*if ECU on,set relays*/
		pdi_set_relays(pdi_cfg_file);
		/*delay 1s*/
		for (rate = 5; rate <= 17; rate ++) {
		pdi_mdelay(89);
		printf("##START##STATUS-");
		sprintf(temp, "%d", rate);
		printf("%s", temp);
		printf("##END##\n");
		}
		/*delay 7s*/
		for (rate = 17; rate <= 96; rate ++) {
			pdi_mdelay(89);
			printf("##START##STATUS-");
			sprintf(temp, "%d", rate);
			printf("%s", temp);
			printf("##END##\n");
		} 
                //Read_softverison(Kline,&num);
                if(ap2_check_softverion(pdi_cfg_file)){
                  //printf("1");//wrong!
                        pdi_fail_action();
	                printf("##START##EC-Soft Version wrong...##END##\n");
			continue;
                }
                printf("##START##EC-      Checking Soft Version Done...##END##\n");
                memset(Kline,0,100);
                num=0;
                AP2_GetFault1(Kline,&num,&num1);
                if(num1) goto ERR_x;
                num=0;
                AP2_GetFault(Kline,&num,&num1);
                //ap2_error_check(Kline,&num,pdi_cfg_file);
                if(num1){
                        if(ap2_error_check(Kline,&num1,pdi_cfg_file)){
                              pdi_fail_action();
                         }
                        else{
                              printf("##START##EC-");
                              printf("##START##EC-These faults are acceptable!");
                              printf("##END##\n");
                              pdi_pass_action();
                        }
                }
                else if(num1 == 0){
                   printf("##START##EC-");
                   printf("error:0xA5  lost!");
                   printf("##END##\n");
                   pdi_fail_action();
                   //printf("##START##STATUS-100##END##\n");
                   //goto ERR_x;
                } 
                ERR_x:
                     //pdi_fail_action();
                   printf("##START##STATUS-100##END##\n");
          }
}
#if 1
static int cmd_ap2_func(int argc, char *argv[])
{
	int num_fault, i;
        int num_fault1;
	const char *usage = {
		"ap2 , usage:\n"
		"ap2 fault		read the error code\n"
		"ap2 batt on/off		battary on or off\n"
		"ap2 start		start the diagnostic seesion\n"
	};

	if (argc < 2) {
		printf("%s", usage);
		return 0;
	}

	if(argc == 2) {
		if(argv[1][0] == 'f') {
			pdi_IGN_on();
			pdi_mdelay(100);
			//pdi_startsession();
			if (AP2_GetFault(AP2_fault_buf, &num_fault,&num_fault1))
				printf("##ERROR##\n");
			else {
				printf("##OK##\n");
				printf("num of fault is: %d\n", num_fault);
				for (i = 0; i < num_fault; i += 4)
					printf("0x%02x, 0x%02x, 0x%02x, 0x%02x\n", AP2_fault_buf[i], AP2_fault_buf[i+1], AP2_fault_buf[i+2], AP2_fault_buf[i+3]);
			}
			pdi_IGN_off();
                        
		}
	}

	// power on/off
	if(argc == 3) {
		if(argv[1][0] == 'b') {
			if(argv[2][1] == 'n')
				pdi_IGN_on();
			if(argv[2][1] == 'f')
				pdi_IGN_off();
		}
	}

	return 0;
}
                               
#endif 

