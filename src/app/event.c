//{{{ include
#include <stdlib.h>
#include "event.h"
#include "workdata.h"
#include "user_conf.h"
//}}}

//{{{ define
event_t event[EVENT_MAX];
//}}}

//{{{ task
void event_task(void)
{
        os[EVENT_TASK].Attrib &= ~OSREQUSTED;
        for(int i = 0; i < EVENT_MAX; i++){
                if(event[i].fun != NULL){
                        if(event[i].time)
                                event[i].time--;
                        if(event[i].time == 0) {
                                if(event[i].repeat)
                                        event[i].repeat--;
                                else{
                                        event[i].fun = NULL;
                                        return;
                                }
                                if(event[i].fun(event[i].param) == 0){
                                        event[i].fun = NULL;
                                        return;
                                }
                                event[i].time = event[i].PERIOD;
                        }
                }
        }
        return;
}
//}}}

//{{{ add
int32_t event_add(uint32_t period, uint32_t repeat, void *param, event_fun_t fun)
{
        uint32_t cnt = period / (1000 / OS_TICK_PER_SECOND);
        for(int i = 0; i < EVENT_MAX; i++){
                if(event[i].fun == NULL){
                        event[i].PERIOD = cnt;
                        event[i].time = cnt;
                        event[i].repeat = repeat;
                        event[i].param = param;
                        event[i].fun = fun;
                        return 0;
                }
        }
        return -1;
}
//}}}

//{{{ init
void event_init(void) 
{
        for(int i = 0; i < EVENT_MAX; i++){
                event[i].PERIOD = 0;
                event[i].time = 0;
                event[i].repeat = 0;
                event[i].param = NULL;
                event[i].fun = NULL;
        }
}
//}}}
