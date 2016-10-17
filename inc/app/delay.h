#ifndef __DELAY_H__
#define __DELAY_H__			   

#include "stm32f10x.h"

void system_tick_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void TimingDelay_Decrement(void);

#endif





























