/*
 *jiamao.gu @2013 first version
 *jiamao.gu @2014 update
*/
#ifndef __CH446_H_
#define __CH446_H_

typedef struct {
	void (*init)(void);
	void (*write)(int xy, int value);
	void (*set)(int y, int x, int on);
	void (*reset)(void);
} ch446_func_t;

typedef enum {
	NO_EXIST = 0,
	YES_EXIST = ! NO_EXIST
} Channel_Exist_E;

void ch446_Write(int y, int x, int on);
void ch446_Switch(int xy, int on);
void ch446_Init(void);

extern const ch446_func_t ch446;

#endif
