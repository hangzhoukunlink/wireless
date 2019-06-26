#ifndef __LPT_AD5347_H
#define __LPT_AD5347_H

#include "stm32f10x.h"

typedef struct {
	void (*init)(int choice);
	void (*write)(unsigned int mv, unsigned int channel);
	int (*read)(unsigned int channel);
	void (*clear)(void);
} ad5347_t;

extern const ad5347_t ad5347;
#endif
