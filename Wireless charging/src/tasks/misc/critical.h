#ifndef CRITICAL_H
#define CRITICAL_H

#include "stm32f10x.h"

typedef struct {
    bool bLocked;
} critical_sector_t;
typedef critical_sector_t mutex_t;
#define ENTER_CRITICAL_SECTOR(__CRITICAL) enter_critical_sector(__CRITICAL) //true:enter critical sector 
#define LEAVE_CRITICAL_SECTOR(__CRITICAL) leave_critical_sector(__CRITICAL)
#define INIT_CRITICAL_SECTOR(__CRITICAL) init_critical_sector(__CRITICAL)

void init_critical_sector(critical_sector_t *ptCritical);
bool enter_critical_sector(critical_sector_t *ptCritical);
void leave_critical_sector(critical_sector_t *ptCritical);
#endif