#ifndef __AT_MOTOR_CMD_H__
#define __AT_MOTOR_CMD_H__

#include "stm32f10x.h"

void at_motor_sw_protection_set(uint8_t id, uint8_t *pPara);
void at_motor_lpwm_query(uint8_t id);
void at_motor_lpwm_set(uint8_t id, uint8_t *pPara);
void at_motor_rpwm_query(uint8_t id);
void at_motor_rpwm_set(uint8_t id, uint8_t *pPara);
void at_motor_pwm_query(uint8_t id);
void at_motor_pwm_set(uint8_t id, uint8_t *pPara);
void at_motor_set(uint8_t id, uint8_t *pPara);
void at_vr_set(uint8_t id, uint8_t *pPara);
void at_robot_wz_set(uint8_t id, uint8_t *pPara);
void at_robot_vx_set(uint8_t id, uint8_t *pPara);

#endif
