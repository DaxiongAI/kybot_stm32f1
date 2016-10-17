#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "tim.h"

#define BUZZER_GPIO 					GPIOB
#define BUZZER_PIN 					GPIO_Pin_12
//#define BUZZER_USE_TIM 				TIM4
#define DEFAULT_ALARM_TIME 				10000
//占空比，单位(%)
#define BUZZER_DUTY_CYCLE  				50
#define BUZZER_PULSE 					(BUZZER_DUTY_CYCLE / 100.0 * TIM4_PERIOD)

extern volatile u16 buzzer_toggle_time;

void buzzer_task(void);
void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_toggle(u16 cnt, u16 period);

#endif
