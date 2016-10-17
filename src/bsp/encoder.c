//{{{ include
#include "encoder.h"
#include "motor.h"
#include "workdata.h"
#include "controller.h"
#include "stm32f10x.h"
//}}}

//{{{ declare
encoder_t encoder;
encoder_t prev_tick;
//}}}

//{{{ init
void encoder_init(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;

        //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_12;//使用TIM1_ETR和TIM2_CH1_ETR引脚 
        //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;//PB3 PB4
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//TIM2_ETR
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         
        GPIO_Init(GPIOA,&GPIO_InitStructure);                               

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//TIM1_ETR
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         
        GPIO_Init(GPIOA,&GPIO_InitStructure);   

        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

        //定时器1初始化
        TIM_DeInit(LEFT_ENC_TIM);

        TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
        TIM_TimeBaseStructure.TIM_Prescaler = 0x00;
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(LEFT_ENC_TIM, &TIM_TimeBaseStructure); // Time base configuration 

        //TIM_TIxExternalClockConfig(TIM2,TIM_TIxExternalCLK1Source_TI2,TIM_ICPolarity_Rising,0);
        TIM_ETRClockMode2Config(LEFT_ENC_TIM, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
        TIM_SetCounter(LEFT_ENC_TIM, 0);
        TIM_ClearFlag(LEFT_ENC_TIM,TIM_FLAG_Update);
        //TIM2 overflow interrupt enable
        TIM_ITConfig(LEFT_ENC_TIM,TIM_IT_Update,ENABLE);
        TIM_Cmd(LEFT_ENC_TIM,ENABLE);  //TIM_GetCounter(TIM1); 

        //定时器2初始化
        TIM_DeInit(RIGHT_ENC_TIM);

        TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
        TIM_TimeBaseStructure.TIM_Prescaler = 0x00;
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(RIGHT_ENC_TIM, &TIM_TimeBaseStructure); // Time base configuration 

        TIM_TimeBaseInit(RIGHT_ENC_TIM, &TIM_TimeBaseStructure); // Time base configuration 
        TIM_ETRClockMode2Config(RIGHT_ENC_TIM, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
        TIM_SetCounter(RIGHT_ENC_TIM, 0);
        TIM_ClearFlag(RIGHT_ENC_TIM,TIM_FLAG_Update);
        //TIM2 overflow interrupt enable
        TIM_ITConfig(RIGHT_ENC_TIM,TIM_IT_Update,ENABLE);
        TIM_Cmd(RIGHT_ENC_TIM,ENABLE);

        /*		
                        EXTI_InitTypeDef EXTI_InitStructure;

                        EXTI_ClearITPendingBit(EXTI_Line3);
                        EXTI_ClearITPendingBit(EXTI_Line15);
        //EXTI_ClearITPendingBit(EXTI_Line4);

        GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
        //GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);



        //EXTI_InitStructure.EXTI_Line= EXTI_Line3 | EXTI_Line4;   
        EXTI_InitStructure.EXTI_Line= EXTI_Line3 | EXTI_Line15;   
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);
        //EXTI_GenerateSWInterrupt(EXTI_Line3);//软件触发一次中断
        //EXTI_GenerateSWInterrupt(EXTI_Line15);//软件触发一次中断
        */
}
//}}}

//{{{ reset
void encoder_reset(void)
{
        encoder.left    = 0;
        encoder.right   = 0;
        
        prev_tick.left  = 0;
        prev_tick.right = 0;

        LEFT_ENC_TIM->CNT  = 0;
        RIGHT_ENC_TIM->CNT = 0;
}
//}}}

double encoder_get_theta(void)
{
        static uint16_t prev_left_tick = 0;
        static uint16_t prev_right_tick = 0;

        uint16_t curr_left_tick  = LEFT_ENC_TIM->CNT;
        uint16_t curr_right_tick = RIGHT_ENC_TIM->CNT;

        int16_t delta_left  = curr_left_tick  - prev_left_tick;
        int16_t delta_right = curr_right_tick - prev_right_tick;

	prev_left_tick = curr_left_tick;
	prev_right_tick = curr_right_tick;

	double tick_to_rad = (double)3.1415926 * 2 / wheel_ticks;
	double dright = tick_to_rad * delta_right;
	double dleft = tick_to_rad * delta_left;

	return (double)wheel_diameter / 2.0 * (dright - dleft) / wheel_bias;
}

