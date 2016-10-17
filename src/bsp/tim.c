//{{{include
#include "tim.h"
#include "buzzer.h"
#include "user_conf.h"
//}}}

//{{{ init
/**************************实现函数********************************************
 *函数原型:		
 *功　　能:1ms中断一次,计数器为1000		
 *******************************************************************************/
void delay_tim_init(u16 period_num)
{
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        //TIM_OCInitTypeDef  TIM_OCInitStructure;
        //基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出

        TIM_DeInit(DELAY_TIM);

        TIM_TimeBaseStructure.TIM_Period=period_num;//装载值
        //prescaler is 1200,that is 72000000/72/500=2000Hz;
        TIM_TimeBaseStructure.TIM_Prescaler = 72-1;//分频系数
        //set clock division 
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
        //count up
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

        TIM_TimeBaseInit(DELAY_TIM,&TIM_TimeBaseStructure);
        /**********************pwm for buzzer****************************************************
        //PWM channels configuration (All identical!)
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 0;//当计数器计数小于这个值(CCR1)时，输出有效电平
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //当计数器计数值小于CCR1时，高电平为有效电平

        TIM_OC3Init(DELAY_TIM,&TIM_OCInitStructure);//PB8

        TIM_OC3PreloadConfig(DELAY_TIM, TIM_OCPreload_Enable);

        //TIM_SetCompare3(DELAY_TIM,500);

        TIM_ARRPreloadConfig(DELAY_TIM, ENABLE);	
         ***************************************************************************************/
        //clear the TIM2 overflow interrupt flag
        TIM_ClearFlag(DELAY_TIM,TIM_FLAG_Update);
        //TIM2 overflow interrupt enable
        TIM_ITConfig(DELAY_TIM,TIM_IT_Update,ENABLE);
        //enable TIM2
        //TIM_Cmd(DELAY_TIM,ENABLE);
}
//}}}

//{{{ start, stop
void start_delay_tim(void)
{
        TIM_SetCounter(DELAY_TIM, 0);
        TIM_Cmd(DELAY_TIM,ENABLE);
}
void stop_delay_tim(void)
{
        TIM_Cmd(DELAY_TIM,DISABLE);
}
//}}}
