//{{{ include
#include "stm32f10x.h"
#include "buzzer.h"
#include "workdata.h"
//}}}

//{{{ declare
volatile u16 buzzer_toggle_time = 0;//100ms
static volatile u16 AUTO_LOAD_TIME = 0;
static u8 turn_on = 0;
static u8 is_buzzer_on = 0;
volatile u16 buzzer_toggle_cnt = 0;

static void timer_check(void);
//}}}

//{{{ on, off
void buzzer_on(void)
{
        //TIM_SetCompare3(BUZZER_USE_TIM,BUZZER_PULSE);
        GPIO_SetBits(BUZZER_GPIO, BUZZER_PIN);
}
void buzzer_off(void)
{
        //TIM_SetCompare3(BUZZER_USE_TIM,0);
        GPIO_ResetBits(BUZZER_GPIO, BUZZER_PIN);
}
//}}}

//{{{ task
static void timer_check(void)
{
        turn_on = 1 - turn_on;

        if(turn_on){
                buzzer_on();
        }
        else{
                buzzer_off();
        }

        if(buzzer_toggle_cnt==0){
                turn_on = 0;
                buzzer_off();
                is_buzzer_on = 0;
        }
        else{
                if(turn_on)
                        buzzer_toggle_cnt--; 

                buzzer_toggle_time = AUTO_LOAD_TIME;
        }
}
void buzzer_task(void)
{
        os[BUZZER_TASK].Attrib &= ~OSREQUSTED; 

        timer_check();

}
//}}}

//{{{ init
void buzzer_init(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_InitStructure.GPIO_Pin   = BUZZER_PIN;
        GPIO_Init(BUZZER_GPIO, &GPIO_InitStructure);

        GPIO_ResetBits(BUZZER_GPIO, BUZZER_PIN);
}
//}}}

//{{{ toggle
/**
cnt: 蜂鸣器响的次数
period:蜂鸣器响的间隔，单位ms
最多能响65.53s
*/
void buzzer_toggle(u16 cnt, u16 period)
{
        if ( (cnt == 0) || (period == 0) || is_buzzer_on )
                return;
        buzzer_toggle_time = period/(1000/OS_TICK_PER_SECOND);
        if (buzzer_toggle_time == 0) {
                return;
        }
        // 马上开启蜂鸣器，这里不开就需要等period的时间蜂鸣器才会响
        buzzer_on();
        is_buzzer_on = 1;
        turn_on = 1;
        AUTO_LOAD_TIME = buzzer_toggle_time ;
        buzzer_toggle_cnt = cnt - 1;//前面加上buzzer_on就要-1
}
//}}}
