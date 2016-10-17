#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f10x.h"

// 36KHZ
#define PWM_PERIOD   						(1000-1)  //PWM周期1000 x 1us = 1ms = 1000hz
#define PWM_PRESCALE 						(2-1)  //1us(1MHz)计数一次

#define LEFT_CTR_GPIO           GPIOA
#define LEFT_CTR1_PIN           GPIO_Pin_6 // TIM3_CH1
#define LEFT_CTR2_PIN           GPIO_Pin_7 // TIM3_CH2

#define RIGHT_CTR_GPIO         	GPIOB
#define RIGHT_CTR1_PIN          GPIO_Pin_0 // TIM3_CH3
#define RIGHT_CTR2_PIN          GPIO_Pin_1 // TIM3_CH4

#define MOTOR_FAULT_GPIO        GPIOA
#define MOTOR_FAULT_PIN         GPIO_Pin_11

typedef struct {
		int8_t l_pwm;
		int8_t r_pwm;
}motor_t;
// 紧急制动使用
extern uint32_t emergency;
extern motor_t motor;

int8_t get_left_pwm(void);
int8_t get_right_pwm(void);
void emergency_stop(void);
void motor_init(void);
void set_pwm(int8_t lpwm, int8_t rpwm);
void set_left_pwm(int8_t pwm);
void set_right_pwm(int8_t pwm);

#endif

