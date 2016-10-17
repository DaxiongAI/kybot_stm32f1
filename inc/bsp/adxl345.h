#ifndef __ADXL345_H__
#define __ADXL345_H__

#include "stm32f10x.h"
#include "imu.h"

// ��д���������Ƶ�ַ������ĺ��ַ
// adxl345��12����SD0�ӵ͵�ƽ
#define ADXL345_ADDRESS_AD0_LOW     0x53 // alt address pin low (GND)
// adxl345��12����SD0�Ӹߵ�ƽ
#define ADXL345_ADDRESS_AD0_HIGH    0x1D // alt address pin high (VCC)

// ��д�����������Ƶ�ַ������ĺ��ַ
// adxl345��4����SA0��gnd
//#define ADXL345_ADDRESS_AD0_LOW    0xA6 // address pin high (GND)
// adxl345��4����SA0��vcc
//#define ADXL345_ADDRESS_AD0_HIGH    0x3A // address pin high (VCC)
//����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�
#define	ADXL345_ADDR   ADXL345_ADDRESS_AD0_LOW

//**********adxl345�ڲ��Ĵ�����ַ*********
#define DEVICE_ID               0X00    //����ID,0XE5
#define THRESH_TAP              0X1D    //�û���ֵ
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
extern int16_imu_t acc_offset;//��Ư
extern int16_imu_t acc_latest;//����һ�ζ�ȡֵ

void adxl345_init(void);
void adxl345_read(void);

#endif
