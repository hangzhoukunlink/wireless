/* led.h
 * 	miaofng@2009 initial version
 */
#ifndef __LED_H_
#define __LED_H_

#ifdef CONFIG_LED_UPDATE_MS
#define LED_FLASH_PERIOD	CONFIG_LED_UPDATE_MS
#else
#define LED_FLASH_PERIOD	1000 /*unit ms*/
#endif

/*for led ecode use*/
#define LED_ERROR_PERIOD	200 /*unit: mS*/
#define LED_ERROR_IDLE		10

typedef enum {
		LED_GREEN = 0,
		LED_RED,
		LED_YELLOW,
		LED_GREEN1,
		LED_RED1,
		LED_YELLOW1,
		LED_GREEN2,
		LED_RED2,
		LED_YELLOW2,
		LED_GREEN3,
		LED_RED3,
		LED_YELLOW3,
		LED_GREEN4,
		LED_RED4,
		LED_YELLOW4,
		LED_GREEN5,
		LED_RED5,
		LED_YELLOW5,
		LED_GREEN6,
		LED_RED6,
		LED_YELLOW6,
		LED_GREEN7,
		LED_RED7,
		LED_YELLOW7,
		LED_GREEN8,
		LED_RED8,
		LED_YELLOW8,
		NR_OF_LED
} led_t;

typedef enum {
	OFF = 0,
	ON
} led_status_t;

typedef enum {
	OFF1 = 0,
        OFF2,
	ON1,
        ON2
} status_t;
void led_Init(void);
void led_Update(void);
void led_Update_Immediate(void);
void led_on(led_t led);
void led_off(led_t led);
void led_inv(led_t led);
void led_flash(led_t led);
void led_error(int ecode);

/*hw led driver routines*/
void led_hwInit(void);
void led_hwSetStatus(led_t led, led_status_t status);
#endif /*__LED_H_*/
