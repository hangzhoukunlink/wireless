/*
 *	peng.guo@2012 initial version
 * 	jiamao.gu@2013 modify mbi5025 pin and add some pdi funtion for test
 */

#include <string.h>
#include "config.h"
#include "sys/task.h"
#include "ulp_time.h"
#include "ulp/sys.h"
#include "can.h"
#include "drv.h"
#include "mbi5025.h"
#include "cfg.h"
#include "priv/usdt.h"
#include "ulp/debug.h"
#include "shell/cmd.h"
#include "led.h"
#include "pdi.h"

POSITION_E *POSITION1;
static const mbi5025_t pdi_mbi5025 = {
		.bus = &spi1,
		.idx = SPI_CS_DUMMY,
		.load_pin = SPI_CS_PC4,
		.oe_pin = SPI_CS_PC5,
};
void pdi_relay_init()
{
	mbi5025_Init(&pdi_mbi5025);     //SPI bus,shift register
	mbi5025_EnableOE(&pdi_mbi5025); //Enable the OutPin 
}
// delay n ms
int pdi_mdelay(int ms)
{
	int left;
	time_t deadline = time_get(ms);
	do {
		left = time_left(deadline);
		if(left >= 10) { //system update period is expected less than 10ms
			sys_update();
		}
	} while(left > 0);
	return 0;
}
#if 0
void pdi_set_relays(const struct pdi_cfg_s *sr)
{
	POSITION1 = Position_Get();     //Get the current position
	char *o = (char *)&(sr -> relay_ex);
	char *p = (char *)&(sr->relay);
	if( *POSITION1 == LEFT){
		mbi5025_WriteByte(&pdi_mbi5025, *(o+3));
		mbi5025_WriteByte(&pdi_mbi5025, *(o+2));
		mbi5025_WriteByte(&pdi_mbi5025, *(o+1));
		mbi5025_WriteByte(&pdi_mbi5025, *(o+0));
		mbi5025_WriteByte(&pdi_mbi5025, *(p+3));
		mbi5025_WriteByte(&pdi_mbi5025, *(p+2));
		mbi5025_WriteByte(&pdi_mbi5025, *(p+1));
		mbi5025_WriteByte(&pdi_mbi5025, *(p+0));
	}
	else{
		mbi5025_WriteByte(&pdi_mbi5025, ~*(o+3));
		mbi5025_WriteByte(&pdi_mbi5025, ~*(o+2));
		mbi5025_WriteByte(&pdi_mbi5025, ~*(o+1));
		mbi5025_WriteByte(&pdi_mbi5025, ~*(o+0));
		mbi5025_WriteByte(&pdi_mbi5025, ~*(p+3));
		mbi5025_WriteByte(&pdi_mbi5025, ~*(p+2));
		mbi5025_WriteByte(&pdi_mbi5025, ~*(p+1));
		mbi5025_WriteByte(&pdi_mbi5025, ~*(p+0));
          }
	spi_cs_set(pdi_mbi5025.load_pin, 1);
	spi_cs_set(pdi_mbi5025.load_pin, 0);
}
#endif

void pdi_set_relays(const config_t sr)
{
	//char relay_temp[8];
        char relay_temp[12];
	memset(relay_temp,'0',sizeof(relay_temp));
	POSITION1 = Position_Get();     //Get the current position
	//for(int i = 0; i < 16; i = i + 2) {
        for(int i = 0 ; i < 24; i = i + 2) {
		relay_temp[(i + 1) / 2]=(sr.relay[i] & 0x0f ) | ((sr.relay[i + 1] & 0x0f)  << 4 );
		printf("0x%x\t",relay_temp[(i + 1) / 2]);
	}
	if( *POSITION1 == LEFT){
		//for(int i = 0; i < 8; i++) {
                for(int i = 0; i < 12; i++) {
			//mbi5025_WriteByte(&pdi_mbi5025, relay_temp[7 - i] & 0xff);
                        mbi5025_WriteByte(&pdi_mbi5025, relay_temp[11 - i] & 0xff);
		}	
	}
	else{
		//for(int i = 0; i < 8; i++) {
                for(int i = 0; i < 12; i++) {
			//mbi5025_WriteByte(&pdi_mbi5025, (~relay_temp[7 - i]) & 0xff);
                        mbi5025_WriteByte(&pdi_mbi5025, (~relay_temp[11 - i]) & 0xff);
		}
	}
	spi_cs_set(pdi_mbi5025.load_pin, 1);
	pdi_mdelay(100);
	spi_cs_set(pdi_mbi5025.load_pin, 0);
}
#if 1
// whether the error code is allowed
// To be improved 
// if the error code is 3 bytes, must repair this funciton
int pdi_error_check(char *fault, int *num, const config_t sr)
{
	int i, j;
        //int pdi_fault_temp[32];
        int pdi_fault_temp[48];
	memset(pdi_fault_temp,'\0',48);
	int faultable_temp[5];
	memset(faultable_temp,'\0',5);
	int length1 = *num;
	int length2 = strlen(sr.errorable) / 2;
	// rebuilt array
	for(i = 0; i < *num; i ++) {  
		pdi_fault_temp[i] = ((int)fault[i * 2]) << 8 | ((int)fault[i * 2 + 1]);  // two char merged in a short 
        }
	for(j = 0; j < strlen(sr.errorable); j += 2) {
        //for(j = 0; j < strlen(sr.errorable); j += 4) {
		//faultable_temp[(j + 1) / 3] = ((int)sr.errorable[j]) << 16 | ((int)sr.errorable[j + 1]) << 8 | ((int)sr.errorable[j + 2]); // two char merged in a short
		faultable_temp[(j + 1) / 2] = ((int)sr.errorable[j]) << 8 | ((int)sr.errorable[j + 1]); // two char merged in a short	
        }
	// compare two arrays
	for(i = 0; i < length1; i++) {
		for(j = 0; j < length2; j++) {
			if(pdi_fault_temp[i] == faultable_temp[j]) { // compare one error with all the allowable codes
				break;                                  // if equal, out the circle 
			}											// add which error code can be allowed in future 
		}
		if(j == (length2 - 1)) {								// if one error code is not equal with all the allowable codes
			return 1;                                   // return 
		}
	}
	return 0;
}
#endif
void pdi_fail_action()
{
	pdi_mdelay(500);
	POSITION1 = Position_Get();
	if( *POSITION1 == LEFT){
		led_off(LED_GREEN1);
		led_off(LED_RED1);
                led_off(LED_YELLOW1);
		led_on(LED_RED1);
	}
	else{
		led_off(LED_GREEN2);
		led_off(LED_RED2);
                led_off(LED_YELLOW2);
		led_on(LED_RED2);
	}
	beep_on();
	pdi_mdelay(1000);
	beep_off();
}

void pdi_pass_action()
{
	pdi_mdelay(500);
	POSITION1 = Position_Get();
	if( *POSITION1 == LEFT){
		led_off(LED_GREEN1);
		led_off(LED_RED1);
                led_off(LED_YELLOW1);
		led_on(LED_GREEN1);
	}
	else{
		led_off(LED_GREEN2);
		led_off(LED_RED2);
                led_off(LED_YELLOW2);
		led_on(LED_GREEN2);
	}
	beep_on();
	pdi_mdelay(20);
	printf("##START##EC-Test Result : No Error...##END##\n");
	pdi_mdelay(250);
	beep_off();
	pdi_mdelay(130);
	beep_on();
	pdi_mdelay(250);
	beep_off();
}

void pdi_noton_action()
{
	POSITION1 = Position_Get();
	if( *POSITION1 == LEFT){
		for(int i = 0; i < 4; i++) {
			beep_on();
			led_on(LED_YELLOW1);
			pdi_mdelay(200);
			beep_off();
			led_off(LED_YELLOW1);
			pdi_mdelay(100);
	}
		for(int i = 0; i < 4; i++) {
			led_on(LED_YELLOW1);
			pdi_mdelay(200);
			led_off(LED_YELLOW1);
			pdi_mdelay(100);
		}
	}
	else{
		for(int i = 0; i < 4; i++) {
			beep_on();
			led_on(LED_YELLOW2);
			pdi_mdelay(200);
			beep_off();
			led_off(LED_YELLOW2);
			pdi_mdelay(100);
		}
		for(int i = 0; i < 4; i++) {
			led_on(LED_YELLOW2);
			pdi_mdelay(200);
			led_on(LED_YELLOW2);
			pdi_mdelay(100);
		}
	}
}

/*******************************************
***** fast blink the yellow led five time***
***** beep on / off************************* 
********************************************/

void pre_check_action()
{
	led_on(LED_GREEN1);
        led_on(LED_GREEN2);
        led_on(LED_RED1);
        led_on(LED_RED2);
        led_on(LED_YELLOW1);
        led_on(LED_YELLOW2);
	//led_on(ALL_LED);
	beep_on();
	pdi_mdelay(200);
	led_off(LED_GREEN1);
        led_off(LED_GREEN2);
        led_off(LED_RED1);
        led_off(LED_RED2);
        led_off(LED_YELLOW1);
        led_off(LED_YELLOW2);
	beep_off();
	pdi_mdelay(100);
        
	for(int i = 0; i < 5; i++){
          led_on(LED_GREEN1);
          led_on(LED_GREEN2);
          led_on(LED_RED1);
          led_on(LED_RED2);
          led_on(LED_YELLOW1);
          led_on(LED_YELLOW2);
          //led_on(LED_YELLOW2);
          pdi_mdelay(200);
          led_off(LED_GREEN1);
          led_off(LED_GREEN2);
          led_off(LED_RED1);
          led_off(LED_RED2);
          led_off(LED_YELLOW1);
          led_off(LED_YELLOW2);
          pdi_mdelay(100);
        }
}
/*void counter_fail_add()
{
	counter_fail_rise();
	for(int i = 500000; i > 0; i--);
	counter_fail_down();
	for(int i = 500000; i > 0; i--);
}

void counter_pass_add()
{
	counter_pass_rise();
	for(int i = 500000; i > 0; i--);
	counter_pass_down();
	for(int i = 500000; i > 0; i--);
}*/

/*******************************
** Blink the yellow when test***
*******************************/
void start_action()
{	
	POSITION1 = Position_Get();
	if(*POSITION1 == LEFT){
		led_off(LED_GREEN1);
		led_off(LED_RED1);
                led_off(LED_YELLOW1);
		led_Update_Immediate();
		led_flash(LED_YELLOW1);
	}
	else{
		led_off(LED_GREEN2);
		led_off(LED_RED2);
                led_off(LED_YELLOW2);
		led_Update_Immediate();
		led_flash(LED_YELLOW2);
	}
}
/**/
void config_init(config_t file)
{
	for(int i = 0; i < length_bcode_part; i++) {
		file.bcode_part[i] = 0;
	}
	file.bcode_part[length_bcode_part] = '\0';
	for(int i = 0; i < length_relay; i++) {
		file.relay[i] = 0;
	}
	file.relay[length_relay] = '\0';
	file.length_softversion = 0;
	for(int i = 0; i < max_length_softversion; i++) {
		file.softversion[i] = 0;
	}       
	file.softversion[max_length_softversion] = '\0';
	file.IMU = 0;
	for(int i = 0; i < length_errorable; i++) {
		file.errorable[i] = '\0';
	}
}
/**/
void autoliv_init(const autoliv_t *chip)
{
	uart_cfg_t cfg = {
		.baud = 9600,
	};
	chip->bus->init(&cfg);
}

/**/
void autoliv_send(const autoliv_t *chip, char *buffer)
{
	for(int i = 0;i < strlen(buffer); i++) {          //counter to until '\n'
		chip->bus->putchar(buffer[i]);
	}
}
/**/
void autoliv_send_char(const autoliv_t *chip, char a)
{
	chip->bus->putchar(a);
}
/**/
int autoliv_get(const autoliv_t *chip,char *rx_data)
{
	time_t overtime;
	if (chip->bus->poll()) {
		for (int i = 0; i < chip->data_len; i++) {
			//according to overtime
			overtime = time_get(chip->dead_time);
			while (chip->bus->poll() == 0) {
				if (time_left(overtime) < 0)
					return 0;
			}
			rx_data[i] = chip->bus->getchar();
		}
		return 0;
	}
	return 1;
}
#ifdef CONFIG_PDI_CCP2
//transplant from Autoliv
void CalculateKey (unsigned char accessLevel, unsigned char challenge[8],unsigned char secKeyOut[3])
{
	unsigned char idx, i;
	unsigned char mask, x;
	unsigned char kSecB2[2] = {0x00, 0x80};
	unsigned char kSecC2[2] = {0x00, 0x10}; 
	unsigned char kSecC1[2] = {0x00, 0x90}; 
	unsigned char kSecC0[2] = {0x00, 0x28};
	unsigned char secKey[3] = {0xA9, 0x41, 0xC5};
	// Select the five fixed bytes
	switch(accessLevel) {
		case 0x01:
			challenge[3] = 0x52;
			challenge[4] = 0x6F;  
			challenge[5] = 0x77;  
			challenge[6] = 0x61;  
			challenge[7] = 0x6E; 
			break;		
		case 0x03:
			challenge[3] = 0x5A;
			challenge[4] = 0x89;  
			challenge[5] = 0xE4;  
			challenge[6] = 0x41;  
			challenge[7] = 0x72; 
			break;			
		case 0x61:
			challenge[3] = 0x4D;
			challenge[4] = 0x5A;  
			challenge[5] = 0x04;  
			challenge[6] = 0x68;  
			challenge[7] = 0x38; 
			break;	
	}	
	// Key calculation
	mask = 0x01;
	x=0;	
	for(i=0; i<64;i++) {
		if((mask & challenge[x])!=0) { // 最低位不为0
			idx = 0x01;
		}
		else {
			idx = 0x00;
		}		
		idx ^= (secKey[0] & 0x01);		
		if(mask == 0x80) {
			mask = 0x01;
			x++;
		}
		else {
			mask <<= 1;
		}		
		secKey[0]>>=1;		
		if((secKey[1] & 0x01) != 0) {
			secKey[0] |= 0x80;
		}		
		secKey[1]>>=1;
		if((secKey[2] & 0x01) != 0) {
			secKey[1] |= 0x80;
		}		
		secKey[2]= ((secKey[2]>>=1) | kSecB2[idx]);			
		secKey[0] = (secKey[0] ^ kSecC0[idx]);
		secKey[1] = (secKey[1] ^ kSecC1[idx]); 
		secKey[2] = (secKey[2] ^ kSecC2[idx]); 
	}	
	secKeyOut[0] = (secKey[0] >> 4);
	secKeyOut[0] |= (secKey[1] << 4); 	
	secKeyOut[1] = (secKey[2] >> 4);
	secKeyOut[1] |= (secKey[1] & 0xF0); 	
	secKeyOut[2] = (secKey[2] & 0x0F); 
	secKeyOut[2] |= (secKey[0] << 4);	
}
#endif
static int cmd_pdi_func(int argc, char *argv[])
{
	const char *usage = {
		"pdi , usage:\n"
		"pdi set led x y  x:1 or 2; y:green red or yellow\n"
		"pdi reset led x y x:1 or 2; y:green red or yellow\n"
		"pdi set beep on                     test the beep\n"
		"pdi set can on             \n"
		"pdi set kline on             \n"
		"pdi set pass counter       pass counter add\n"
		"pdi set fail counter       fail counter add\n"
		"pdi lock valve x           lock the valve\n"
		"pdi unlock valve x         unlock the valve\n"
		"pdi set relay plus data    set the relay plus\n"
		"pdi set relay minus data   set the relay minus\n"
		"pdi battery on/off		power on/off\n"
		"pdi set local relay x\r\n"
	};

	if (argc < 2) {
		printf("%s", usage);
		return 0;
	}
	int index = 1;
	led_t result = NR_OF_LED;
	if(argc == 5){
		if(!strcmp(argv[1],"set")){
			sscanf(argv[3],"%d",&index);
			result = (strcmp(argv[4],"green") == 0)?LED_GREEN: \
				((strcmp(argv[4],"red") == 0)?LED_RED:LED_YELLOW);
			led_on((led_t)(result + 3 * index)); 
			printf("OK\n");
		}
		if(!strcmp(argv[1],"reset")){
			sscanf(argv[3],"%d",&index);
			result = (strcmp(argv[4],"green") == 0)?LED_GREEN: \
				((strcmp(argv[4],"red") == 0)?LED_RED:LED_YELLOW);
			led_off((led_t)(result + 3 * index));
			printf("OK\n");			
		}
		int temp = 0;
		if(!strcmp(argv[3],"plus")){
			sscanf(argv[4],"%x",&temp);
			mbi5025_WriteByte(&pdi_mbi5025, temp);
			spi_cs_set(pdi_mbi5025.load_pin, 1);
			pdi_mdelay(10);
			spi_cs_set(pdi_mbi5025.load_pin, 0);
			printf("OK\n");
		}
		if(!strcmp(argv[3],"minus")){
			sscanf(argv[4],"%x",&temp);
			mbi5025_WriteByte(&pdi_mbi5025, ~temp);
			spi_cs_set(pdi_mbi5025.load_pin, 1);
			pdi_mdelay(10);
			spi_cs_set(pdi_mbi5025.load_pin, 0);
			printf("OK\n");
		}
	}
	if(argc == 4){
		if(!strcmp(argv[2],"beep")){
			beep_on();
			pdi_mdelay(200);
			beep_off();
			pdi_mdelay(200);
			beep_on();
			pdi_mdelay(200);
			beep_off();
			printf("OK\n");
		}
		if(!strcmp(argv[2],"can")){
			pdi_can();
			printf("OK\n");
		}
		if(!strcmp(argv[2],"kline")){
			pdi_kline();
			printf("OK\n");
		}
		if(!strcmp(argv[1],"lock")){
			if(!strcmp(argv[3],"1")){
				GPIOC->ODR |= (1<<0);
				printf("OK\n");
			}
			if(!strcmp(argv[3],"2")){
				GPIOC->ODR |= (1<<1);
				printf("OK\n");
			}
		}
		if(!strcmp(argv[1],"unlock")){
			if(!strcmp(argv[3],"1")){
				GPIOC->ODR &= !(1<<0);
				printf("OK\n");
			}
			if(!strcmp(argv[3],"2")){
				GPIOC->ODR &= !(1<<1);
				printf("OK\n");
			}
		}
		if(!strcmp(argv[3],"counter")) {
			if(!strcmp(argv[2],"pass")) {
				//counter_pass_add();
				printf("OK\n");
			}
			if(!strcmp(argv[2],"fail")) {
				//counter_fail_add();
				printf("OK\n");
			}
		}
	}
	if(argc == 5) {
		if(!strcmp(argv[2],"relay")) {
			//KD35-57K30D 11
			char relay_620724500W[8] = {0xfd,0xa0,0x92,0x49,0x42,0x0,0x12,0x0};
			//KD45-57K30B 12
			char relay_620726500W[8] = {0x3d,0x82,0x92,0x49,0x42,0x8,0x10,0x0};
			//KD47-57K30 13
			char relay_620726600N[8] = {0x3d,0x82,0x92,0x49,0x42,0x8,0x0,0x0};
			//KD15-57K30 14
			char relay_629022100A[8] = {0x3d,0x82,0x92,0x49,0x42,0x8,0x10,0x0};
			//KD32-57K30 15
			char relay_629022200A[8] = {0x3d,0x82,0x92,0x49,0x42,0x8,0x0,0x0};
			//GJR9-57K30B 21
			char relay_624551100G[8] = {0xfd,0xa0,0x92,0x49,0x42,0x0,0x92,0x0};
			//GHP9-57K30A 22
			char relay_624551700G[8] = {0x3d,0x82,0x90,0x49,0x42,0x8,0x10,0x0};
			//BHN9-57K30A 41
			char relay_626095600F[8] = {0xfd,0xa0,0x92,0x49,0x42,0x0,0x12,0x0};
			//B45A-57K30A 42
			char relay_626116800E[8] = {0x3d,0x82,0x90,0x49,0x42,0x0,0x10,0x0};
			//KD47-57K30A 43
			char relay_626117000E[8] = {0x3d,0x82,0x90,0x49,0x42,0x8,0x0,0x0};
			//B46D-57K30A 44
			char relay_626117300E[8] = {0x1d,0x82,0x90,0x48,0x0,0x0,0x10,0x0};
			//RC7 50
			//char relay_rc7[12] = {0x10,0x24,0x0,0x90,0xa,0x96,0x0,0xd9,0xff,0x0,0x0,0xff};
			char relay_rc7[12] = {0xff,0x0,0x0,0xff,0xd9,0x0,0x96,0xa,0x90,0x0,0x24,0x10};
			//B515
			char relay_0cee[16] = {0xdf,0xfb,0x0,0x0,0x20,0x04,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
			char relay_0bee[16] = {0xdf,0xf9,0x0,0x0,0x20,0x06,0xff,0xff,0xf3,0xff,0xff,0xff,0xff,0xff,0xf8,0xff};
			//char relay_test[12] ={0x7f,0xbb,0xba,0x7b,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
			char relay_test[12]={0XdF,0XFF,0XFF,0XFF,0X5F,0X2F,0X7F,0X49,0X92,0XA4,0X92,0XFF};
			char relay_chb041_8[12] = {0xe7,0x3,0x7,0x1,0x2f,0x0,0x20,0x20,0x1,0x10,0x1,0x1};
			char relay_hzh_jx[12] = {0xff,0xff,0x7f,0x1f,0x1f,0x11,0xd,0x0,0x0,0x0,0x0,0x0};
			if(!strcmp(argv[3],"left")) {
				int temp = 0;
				sscanf(argv[4],"%d",&temp);
				if(temp == 11) {
					printf("11 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_620724500W[7 - i] & 0xff);
					}
				} else if(12 == temp) {
					printf("12 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_620726500W[7 - i] & 0xff);
					}
				} else if(13 == temp) {
					printf("13 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_620726600N[7 - i] & 0xff);
					}
				} else if(14 == temp) {
					printf("14 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_629022100A[7 - i] & 0xff);
					}
				} else if(15 == temp) {
					printf("15 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_629022200A[7 - i] & 0xff);
					}
				} else if(21 == temp) {
					printf("21 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_624551100G[7 - i] & 0xff);
					}
				} else if(22 == temp) {
					printf("22 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_624551700G[7 - i] & 0xff);
					}
				} else if(41 == temp) {
					printf("41 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_626095600F[7 - i] & 0xff);
					}
				} else if(42 == temp) {
					printf("42 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_626116800E[7 - i] & 0xff);
					}
				} else if(43 == temp) {
					printf("43 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_626117000E[7 - i] & 0xff);
					}
				} else if(44 == temp) {
					printf("44 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_626117300E[7 - i] & 0xff);
					}
				} else if(50 == temp) {
					printf("50 OK !\r\n");
					for(int i = 0; i < 12; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_rc7[11 - i] & 0xff);
					}
				} else if(51 == temp) {
					printf("51 OK !\r\n");
					for(int i = 0; i < 16; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_0cee[15 - i] & 0xff);
					}
				} else if(52 == temp) {
					printf("52 OK !\r\n");
					for(int i = 0; i < 16; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_0bee[15 - i] & 0xff);
					}
				} else if(60 == temp){
					printf("60 OK !\r\n");
					for(int i = 0; i < 12; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_test[11 - i] & 0xff);
					}
				} else if(61 == temp) {
					printf("61 OK!\n");
					for(int i = 0; i < 12; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_chb041_8[11 - i] & 0xff);
					}
				} else if(70 == temp) {
					printf("70 OK!\n");
					for(int i = 0; i < 12; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, relay_hzh_jx[11 - i] & 0xff);
					}
				}
				spi_cs_set(pdi_mbi5025.load_pin, 1);
				pdi_mdelay(15);
				spi_cs_set(pdi_mbi5025.load_pin, 0);
			} else {
				int temp = 0;
				sscanf(argv[4],"%d",&temp);
				if(temp == 11) {
					printf("11 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_620724500W[7 - i]) & 0xff);
					}
				} else if(12 == temp) {
					printf("12 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_620726500W[7 - i]) & 0xff);
					}
				} else if(13 == temp) {
					printf("13 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_620726600N[7 - i]) & 0xff);
					}
				} else if(14 == temp) {
					printf("14 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_629022100A[7 - i]) & 0xff);
					}
				} else if(15 == temp) {
					printf("15 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_629022200A[7 - i]) & 0xff);
					}
				} else if(21 == temp) {
					printf("21 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_624551100G[7 - i]) & 0xff);
					}
				} else if(22 == temp) {
					printf("22 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_624551700G[7 - i]) & 0xff);
					}
				} else if(41 == temp) {
					printf("41 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_626095600F[7 - i]) & 0xff);
					}
				} else if(42 == temp) {
					printf("42 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_626116800E[7 - i]) & 0xff);
					}
				} else if(43 == temp) {
					printf("43 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_626117000E[7 - i]) & 0xff);
					}
				} else if(44 == temp) {
					printf("44 OK !\r\n");
					for(int i = 0; i < 8; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_626117300E[7 - i]) & 0xff);
					}
				} else if(50 == temp) {
					printf("50 OK !\r\n");
					for(int i = 0; i < 12; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_rc7[11 - i]) & 0xff);
					}
				} else if(51 == temp) {
					printf("51 OK !\r\n");
					for(int i = 0; i < 16; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_0cee[15 - i]) & 0xff);
					}
				} else if(52 == temp) {
					printf("52 OK !\r\n");
					for(int i = 0; i < 16; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_0bee[15 - i]) & 0xff);
					}
				} else if(60 == temp){
					printf("60 OK !\r\n");
					for(int i = 0; i < 12; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_test[11 - i] )& 0xff);
					}
				} else if(61 == temp) {
					printf("61 OK!\n");
					for(int i = 0; i < 12; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_chb041_8[11 - i]) & 0xff);
					}
				} else if(70 == temp) {
					printf("70 OK!\n");
					for(int i = 0; i < 12; i++) {
						mbi5025_WriteByte(&pdi_mbi5025, (~relay_hzh_jx[11 - i]) & 0xff);
					}
				}
				spi_cs_set(pdi_mbi5025.load_pin, 1);
				pdi_mdelay(15);
				spi_cs_set(pdi_mbi5025.load_pin, 0);
			}	
		}	
	}
	/*power on/off*/
	if(argc == 3) {
		if(argv[1][0] == 'b') {
			if(argv[2][1] == 'n')
				pdi_IGN_on();
			if(argv[2][1] == 'f')
				pdi_IGN_off();
		}
	}	
        int i = 0;
	int temp = 0;
	if(!strcmp(argv[1],"set") && (!strcmp(argv[2],"mbi5025"))) {
		for (i = 0; i < (argc - 3); i++) {
				sscanf(argv[3+i], "%x", &temp);
				mbi5025_WriteByte(&pdi_mbi5025, temp);
			}
			spi_cs_set(pdi_mbi5025.load_pin, 1);
			pdi_mdelay(15);
			spi_cs_set(pdi_mbi5025.load_pin, 0);
			printf("%d Bytes Write Successful!\n", argc-3);
	}
        

	return 0;
}
const cmd_t cmd_pdi = {"pdi", cmd_pdi_func, "pdi cmd i/f"};
DECLARE_SHELL_CMD(cmd_pdi)