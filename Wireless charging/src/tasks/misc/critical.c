#include "critical.h"
#include <stdio.h>

void init_critical_sector(critical_sector_t *ptCritical)
{
    if(NULL == ptCritical) {
        return ;
    }
    ptCritical -> bLocked = false;  //unlock
}

bool enter_critical_sector(critical_sector_t *ptCritical)
{
    if(NULL == ptCritical) {
        return false;
    }
    if(ptCritical -> bLocked) { //entered
        return false;
    }
    ptCritical -> bLocked = true;   //entering
    return true;
}

void leave_critical_sector(critical_sector_t *ptCritical)
{
    if(NULL == ptCritical) {
        return ;
    }
    ptCritical -> bLocked = false;
}