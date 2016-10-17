#ifndef __L3G4200_H_
#define __L3G4200_H_

#include "stm32f10x.h"
#include "imu.h"

// 读写函数会左移地址用下面的宏地址
// l3g4200第4引脚SA0接低电平
#define L3G4200_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
// l3g4200第4引脚SA0接高电平
#define L3G4200_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
// 读写函数不会左移地址用下面的宏地址
// l3g4200第4引脚SA0接gnd
//#define L3G4200_ADDRESS_AD0_LOW    0xD0 // address pin high (GND)
// l3g4200第4引脚SA0接vcc
//#define L3G4200_ADDRESS_AD0_HIGH    0xD2 // address pin high (VCC)
//定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
#define	L3G4200_ADDR   L3G4200_ADDRESS_AD0_HIGH

//**********L3G4200D内部寄存器地址*********
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define REFERENCE 0x25
#define OUT_TEMP 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_TSH_XH 0x32
#define INT1_TSH_XL 0x33
#define INT1_TSH_YH 0x34
#define INT1_TSH_YL 0x35
#define INT1_TSH_ZH 0x36
#define INT1_TSH_ZL 0x37
#define INT1_DURATION 0x38

extern uint8_t flag_gyr_offset_ok;
extern uint8_t gyr_buf_cnt;
extern int16_imu_t gyr_latest;//最新一次读取值
extern int16_imu_t gyr_buf[FEEDBACK_BUFFER_NUM];
//extern uint8_t temperature;
extern int16_imu_t gyr_raw_offset;//零漂
extern float_imu_t gyr_vel_offset;//rad/s

void l3g4200_init(void);
void l3g4200_read(void);

#endif
