#ifndef __IMU_H__
#define __IMU_H__

#include "stm32f10x.h"

#define i2c_write   i2cWrite
#define i2c_write_byte   i2cWriteByte
#define i2c_read    i2cRead
#define i2c_read_byte   i2cReadByte

//#define IMU_USE_MPU6050
// 10s的数据
//#define MAX_FILTER_CNT 1000
// 15s的数据
#define MAX_FILTER_CNT  2000
// 静置3s等待稳定
#define STATIC_TIME      3
//#define GYR_SENSITIVITY  875/100000
// 大小详见kobuki_driver/include/kobuki_driver/packets/three_axis_gyro.hpp MAX_DATA_SIZE
#define FEEDBACK_BUFFER_NUM 		8
// gyr raw data feedback max count，必须小于FEEDBACK_BUFFER_NUM
#define GYR_RAW_MAX_FEEDBACK_CNT	3
//数据更新(读取)频率100HZ
#define GYR_READ_FREQ 		100
// 放大倍数
// 100000，滤掉120以下的
//#define FACTOR 100

// 10000，滤掉12以下的
//#define FACTOR 1000

// 1000，滤掉1以下
//#define FACTOR 10000

// 100，不过滤
//#define FACTOR 100000

// 10，不过滤
//#define FACTOR 1000000

// 1
//#define FACTOR 10000000

typedef struct int16_imu{
	int16_t x;
	int16_t y;
	int16_t z;
} int16_imu_t;

typedef struct float_imu{
	float x;
	float y;
	float z;
} float_imu_t;

//typedef struct quaternion{
        //float w;
        //float x;
        //float y;
        //float z;
//} quaternion_t;

extern double heading;
extern double fheading;
extern double heading_factor;
extern uint8_t enable_imu_when_motor_static;

void imu_init(void);
void imu_analyze_task(void);
void imu_update_with_acc_gyr(const int16_imu_t *filted_acc, const int16_imu_t *filted_gyr);
void imu_update_with_acc_gyr_mag(const int16_imu_t *filted_acc, const int16_imu_t *filted_gyr, const int16_imu_t *filted_mag);
int32_t log_heading_callback(void *param);

#endif
