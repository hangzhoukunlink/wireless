#ifndef KEY_H
#define KEY_H
#include "stm32f10x.h"

#ifndef __STM32F10x_H
typedef enum {
	false = 0,
	true = !false
} bool;
#else
#define false FALSE
#define true TRUE
#endif

void key_init();
bool is_key1_down();
bool is_key1_up();
bool is_key2_down();
bool is_key2_up(); 
unsigned char key_read(void);

#define IS_KEY1_DOWN() is_key1_down()
#define IS_KEY1_UP() is_key1_up()
#define IS_KEY2_DOWN() is_key2_down()
#define IS_KEY2_UP is_key2_up()
#define N_key    0             //ÎÞ¼ü
#define S_key    1             //µ¥¼ü
#define D_key    2             //Ë«¼ü
#define L_key    3             //³¤¼ü

#define key_state_0        0
#define key_state_1        1
#define key_state_2        2
#define key_state_3        3

typedef enum {
	NONE = 0,
	KEY1DOWN,
	KEY1UP,
	KEY2DOWN,
	KEY2UP,
} keycode_t;
typedef struct {
	//var
	//function
	void (*key_init)();
	bool (*is_key1_down)();
	bool (*is_key1_up)();
	bool (*is_key2_down)();
	bool (*is_key2_up)();
} key_t;

extern const key_t KEY;
#endif