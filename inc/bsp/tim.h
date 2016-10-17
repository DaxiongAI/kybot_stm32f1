#ifndef _BSP_TIM_H_
#define _BSP_TIM_H_
#include "stm32f10x.h"

//��ʱ��1us����һ��,IM4_PERIOD x 1us = 1ms = 1000HZ
#define DELAY_TIM_PERIOD 				1000

extern u32 TIM4_IRQCNT;

void delay_tim_init(u16 period_num);//���ھ�ȷ��ʱ
void start_delay_tim(void);
void stop_delay_tim(void);

#endif
