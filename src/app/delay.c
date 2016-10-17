//{{{ include
#include "delay.h"
#include "tim.h"
#include "stm32f10x.h"
#include "user_conf.h"
//}}}

//{{{ define
static __IO uint32_t TimingDelay;
//}}}

//{{{ init
void system_tick_init(void)	 
{

        if (SysTick_Config(SystemCoreClock / OS_TICK_PER_SECOND))
        { 
                /* Capture error */ 
                while (1);
        }
        // NVIC_SetPriority(SysTick_IRQn, 0x0);                       //SysTick中断优先级设置

}								    
//}}}

//{{{ delay
void delay_ms(u16 nms)
{ 
        TimingDelay = nms;///(1000/OS_TICK_PER_SECOND);
        start_delay_tim();
        while(TimingDelay != 0);
        stop_delay_tim();
}
void delay_us(u32 nus)
{ 
        TimingDelay = nus;

        while(TimingDelay != 0);
}
void TimingDelay_Decrement(void)
{
        if (TimingDelay != 0x00)
        { 
                TimingDelay--;
        }
}
//}}}
