// 网友“茶不思”提供
//#include "STM32_I2C.h"
#include "stm32f10x.h"

#define false 0
#define true  1

#define I2C_GPIO 			GPIOB
#define I2C_GPIO_PERIPH  	RCC_APB2Periph_GPIOB
#define SCL_PIN 			GPIO_Pin_6
#define SDA_PIN 			GPIO_Pin_7

#define SCL_H         (GPIOB->BSRR = SCL_PIN) /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L         (GPIOB->BRR  = SCL_PIN) /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H         (GPIOB->BSRR = SDA_PIN) /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L         (GPIOB->BRR  = SDA_PIN) /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read      (GPIOB->IDR  & SCL_PIN) /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      (GPIOB->IDR  & SDA_PIN) /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */

static void I2C_delay(void)
{
    volatile int i = 10;
    while (i)
        i--;
}

int32_t I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if (!SDA_read)
        return false;
    SDA_L;
    I2C_delay();
    if (SDA_read)
        return false;
    SDA_L;
    I2C_delay();
    return true;
}

void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}

void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

int32_t I2C_WaitAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    if (SDA_read) {
        SCL_L;
        return false;
    }
    SCL_L;
    return true;
}

void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) {
        SCL_L;
        I2C_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}

uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if (SDA_read) {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}

void i2c_init(void)
{
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
    //RCC_APB2PeriphClockCmd(I2C_GPIO_PERIPH,ENABLE);
		// 初始化引脚。
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = SCL_PIN | SDA_PIN; // B6 => CLK , B7 => DAT
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //模拟I2C配置成Out_OD模式
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(I2C_GPIO,SCL_PIN | SDA_PIN);
    // 初始化I2C。
    /*
	I2C_InitTypeDef I2C_InitStructure;
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_ClockSpeed    = 400000;
    I2C_InitStructure.I2C_Mode          = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle     = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1,&I2C_InitStructure);
	*/
}

int32_t i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t * buf, uint8_t len )
{
    int i;
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (i = 0; i < len; i++) {
        I2C_SendByte(buf[i]);
        if (!I2C_WaitAck()) {
            I2C_Stop();
            return false;
        }
    }
    I2C_Stop();
    return true;
}

int32_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(data);
    I2C_WaitAck();
    I2C_Stop();
    return true;
}

int32_t i2cReadBuffer(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr | I2C_Direction_Receiver);
    I2C_WaitAck();
    while (len) {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        buf++;
        len--;
    }
    I2C_Stop();
    return true;
}

//单字节读取*****************************************
unsigned char i2cRead(unsigned char SlaveAddress,unsigned char REG_Address)
{   unsigned char REG_data;     	
	if(!I2C_Start())return false;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return false;}
    I2C_SendByte((u8) REG_Address);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

	REG_data= I2C_ReceiveByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
	return REG_data;

}
