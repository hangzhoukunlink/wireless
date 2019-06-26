#ifndef EVENT_H
#define EVENT_H

#include "stm32f10x.h"

typedef struct {
        bool bAutoReset;
        bool bIsSet;
    } event_t;

#define SET_EVENT(__EVENT) set_event(__EVENT)
#define WAIT_EVENT(__EVENT) wait_event(__EVENT)
#define RESET_EVENT(__EVENT) reset_event(__EVENT)
#define INIT_EVENT(__EVENT, __INIT_VALUE, __MANUAL) init_event(__EVENT, __INIT_VALUE, __MANUAL)

void set_event(event_t *ptEvent);
void reset_event(event_t *ptEvent);
bool wait_event(event_t *ptEvent);
void init_event(event_t *ptEvent, bool bValue, bool bManual);

#endif