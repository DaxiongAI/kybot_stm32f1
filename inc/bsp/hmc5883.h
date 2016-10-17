#ifndef __HMC5883_H__
#define __HMC5883_H__

#include "stm32f10x.h"
#include "imu.h"

extern uint8_t flag_mag_offset_ok;
extern int16_imu_t mag_offset;//零漂
extern int16_imu_t mag_latest;

void hmc5883_init(void);
void hmc5883_read(void);

#endif 
