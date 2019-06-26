#include "event.h"
#include "stm32f10x.h"
#include <stdio.h>
void set_event(event_t *ptEvent)
{
    if(NULL == ptEvent) {
        return ;
    }
    ptEvent -> bIsSet = true;
}

void reset_event(event_t *ptEvent)
{
    if(NULL == ptEvent) {
        return ;
    }
    ptEvent -> bIsSet = false;
}
//return true : set
//bManual:true 鎵嬪姩 false 鑷姩
//bAutoReset true 鑷姩 false 鎵嬪姩
bool wait_event(event_t *ptEvent)
{
    if(NULL == ptEvent) {
        return false;
    }
    if(false != ptEvent -> bIsSet) {
        if(false != ptEvent -> bAutoReset) { //auto reset
            reset_event(ptEvent);
        }
        return true;
    }
    return false;
}

//bManual:true 手动 false 自动
//bAutoReset true 自动 false 手动
void init_event(event_t *ptEvent, bool bValue, bool bManual)
{
    if(NULL == ptEvent) {
        return ;
    }
    ptEvent -> bAutoReset = (bool)!bManual;
    ptEvent -> bIsSet = bValue;
}