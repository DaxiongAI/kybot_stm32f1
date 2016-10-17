#ifndef __ADXL345_H__
#define __ADXL345_H__

#include "stm32f10x.h"
#include "imu.h"

// 读写函数会左移地址用下面的宏地址
// adxl345第12引脚SD0接低电平
#define ADXL345_ADDRESS_AD0_LOW     0x53 // alt address pin low (GND)
// adxl345第12引脚SD0接高电平
#define ADXL345_ADDRESS_AD0_HIGH    0x1D // alt address pin high (VCC)

// 读写函数不会左移地址用下面的宏地址
// adxl345第4引脚SA0接gnd
//#define ADXL345_ADDRESS_AD0_LOW    0xA6 // address pin high (GND)
// adxl345第4引脚SA0接vcc
//#define ADXL345_ADDRESS_AD0_HIGH    0x3A // address pin high (VCC)
//定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
#define	ADXL345_ADDR   ADXL345_ADDRESS_AD0_LOW

//**********adxl345内部寄存器地址*********
#define DEVICE_ID               0X00    //器件ID,0XE5
#define THRESH_TAP              0X1D    //敲击阀值
#define OFSX                    0X1E
#define OFSY                    0X1F
#define OFSZ                    0X20
#define DUR                     0X21
#define LATENT                  0X22
#define WINDOW                  0X23 
#define THRESH_ACK              0X24
#define THRESH_INACT            0X25 
#define TIME_INACT              0X26
#define ACT_INACT_CTL           0X27     
#define THRESH_FF               0X28    
#define TIME_FF                 0X29 
#define TAP_AXES                0X2A  
#define ACT_TAP_STATUS          0X2B 
#define BW_RATE                 0X2C 
#define POWER_CTL               0X2D 

#define INT_ENABLE              0X2E
#define INT_MAP                 0X2F
#define INT_SOURCE              0X30
#define DATA_FORMAT             0X31
#define DATA_X0                 0X32
#define DATA_X1                 0X33
#define DATA_Y0                 0X34
#define DATA_Y1                 0X35
#define DATA_Z0                 0X36
#define DATA_Z1                 0X37
#define FIFO_CTL                0X38
#define FIFO_STATUS             0X39

extern uint8_t flag_acc_offset_ok;
extern int16_imu_t acc_offset;//零漂
extern int16_imu_t acc_latest;//最新一次读取值

void adxl345_init(void);
void adxl345_read(void);

#endif
