#ifndef __AT_IMU_CMD_H__
#define __AT_IMU_CMD_H__

#include "stm32f10x.h"

void at_imu_heading_query(uint8_t id);
void at_imu_heading_factor_query(uint8_t id);
void at_imu_heading_factor_set(uint8_t id, uint8_t *pPara);
void at_motor_static_imu_active_set(uint8_t id, uint8_t *pPara);
#endif
