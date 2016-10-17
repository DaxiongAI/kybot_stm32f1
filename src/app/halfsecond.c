//{{{ include
#include "stm32f10x.h"
#include "workdata.h"
#include "led.h"
#include "log.h"
#include "file_id.h"

#define this_file_id 	file_id_halfsecond
//}}}

//{{{ define
uint8_t turn = 0;
//}}}

//{{{ task
void half_second_task (void)
{ 
        os[HALF_SECOND_TASK].Attrib &= ~OSREQUSTED; 
        turn = ~turn;
        //log_info("half second");
        if(turn)//运行指示灯，闪烁  
                run_on();
        else
                run_off();

        return;        
}
//}}}
