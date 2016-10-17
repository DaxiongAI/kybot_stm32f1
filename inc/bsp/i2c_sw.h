#ifndef __I2C_H__
#define __I2C_H__

#include "stm32f10x.h"
int32_t I2C_Start(void);
int32_t I2C_WaitAck(void);
uint8_t I2C_ReceiveByte(void);
unsigned char i2cRead(unsigned char SlaveAddress,unsigned char REG_Address);
void i2c_init(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
void I2C_SendByte(uint8_t byte);
int32_t i2cReadBuffer(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
int32_t i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t * buf, uint8_t len);
int32_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t data) ;

#endif 
