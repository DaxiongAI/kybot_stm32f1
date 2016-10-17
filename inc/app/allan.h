#ifndef __ALLAN__
#define __ALLAN__

#include "stm32f10x.h"
#include "imu.h"

#define IMU_NORMAL_MODE
//#define IMU_CALCU_VARI

#ifndef IMU_NORMAL_MODE
        #define CALCU_ALLAN_VARI

        #ifndef CALCU_ALLAN_VARI
                #define CALCU_CALIBRATED_PARAM
        #endif
#endif

void output_acc_gyr(int16_imu_t *acc, int16_imu_t *gyr);
void output_gyr(int16_imu_t *gyr);
#endif
