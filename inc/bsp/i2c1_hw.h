//hÎÄ¼þ
#ifndef __STM32_HWIIC_h__
#define __STM32_HWIIC_h__
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* I2C SPE mask */
#define CR1_PE_Set              ((uint16_t)0x0001)
#define CR1_PE_Reset            ((uint16_t)0xFFFE)

/* I2C START mask */
#define CR1_START_Set           ((uint16_t)0x0100)
#define CR1_START_Reset         ((uint16_t)0xFEFF)

#define CR1_POS_Set           ((uint16_t)0x0800)
#define CR1_POS_Reset         ((uint16_t)0xF7FF)

/* I2C STOP mask */
#define CR1_STOP_Set            ((uint16_t)0x0200)
#define CR1_STOP_Reset          ((uint16_t)0xFDFF)

/* I2C ACK mask */
#define CR1_ACK_Set             ((uint16_t)0x0400)
#define CR1_ACK_Reset           ((uint16_t)0xFBFF)

/* I2C ENARP mask */
#define CR1_ENARP_Set           ((uint16_t)0x0010)
#define CR1_ENARP_Reset         ((uint16_t)0xFFEF)

/* I2C NOSTRETCH mask */
#define CR1_NOSTRETCH_Set       ((uint16_t)0x0080)
#define CR1_NOSTRETCH_Reset     ((uint16_t)0xFF7F)

/* I2C registers Masks */
#define CR1_CLEAR_Mask          ((uint16_t)0xFBF5)

/* I2C DMAEN mask */
#define CR2_DMAEN_Set           ((uint16_t)0x0800)
#define CR2_DMAEN_Reset         ((uint16_t)0xF7FF)

/* I2C LAST mask */
#define CR2_LAST_Set            ((uint16_t)0x1000)
#define CR2_LAST_Reset          ((uint16_t)0xEFFF)

/* I2C FREQ mask */
#define CR2_FREQ_Reset          ((uint16_t)0xFFC0)

/* I2C ADD0 mask */
#define OAR1_ADD0_Set           ((uint16_t)0x0001)
#define OAR1_ADD0_Reset         ((uint16_t)0xFFFE)

/* I2C ENDUAL mask */
#define OAR2_ENDUAL_Set         ((uint16_t)0x0001)
#define OAR2_ENDUAL_Reset       ((uint16_t)0xFFFE)

/* I2C ADD2 mask */
#define OAR2_ADD2_Reset         ((uint16_t)0xFF01)

/* I2C F/S mask */
#define CCR_FS_Set              ((uint16_t)0x8000)

/* I2C CCR mask */
#define CCR_CCR_Set             ((uint16_t)0x0FFF)

/* I2C FLAG mask */
#define FLAG_Mask               ((uint32_t)0x00FFFFFF)

/* I2C Interrupt Enable mask */
#define ITEN_Mask               ((uint32_t)0x07000000)


#define I2C_IT_BUF                      ((uint16_t)0x0400)
#define I2C_IT_EVT                      ((uint16_t)0x0200)
#define I2C_IT_ERR                      ((uint16_t)0x0100)


#define I2C_DIRECTION_TX 0
#define I2C_DIRECTION_RX 1

#define OwnAddress1 0x28
#define OwnAddress2 0x30


#define I2C1_DMA_CHANNEL_TX           DMA1_Channel6
#define I2C1_DMA_CHANNEL_RX           DMA1_Channel7

#define I2C2_DMA_CHANNEL_TX           DMA1_Channel4
#define I2C2_DMA_CHANNEL_RX           DMA1_Channel5

#define I2C1_DR_Address              0x40005410
#define I2C2_DR_Address              0x40005810

// Delay is approx 0.2us per loop @64Mhz
#define I2CDEV_LOOPS_PER_US  10
#define I2CDEV_LOOPS_PER_MS  (1000 * I2CDEV_LOOPS_PER_US)

#define I2CDEV_NO_MEM_ADDR  0xFF

#define I2CDEV_I2C1_PIN_SDA GPIO_Pin_7
#define I2CDEV_I2C1_PIN_SCL GPIO_Pin_6

#define GPIO_WAIT_LOW(gpio, pin, timeoutcycles)\
  {\
    int i = timeoutcycles;\
    while(GPIO_ReadInputDataBit(gpio, pin) == Bit_RESET && i--);\
  }

#define GPIO_WAIT_HIGH(gpio, pin, timeoutcycles) \
  {\
    int i = timeoutcycles;\
    while(GPIO_ReadInputDataBit(gpio, pin) == Bit_SET && i--);\
  }

typedef enum
{
  INTERRUPT,
  DMA
} I2C_ProgrammingModel;

typedef enum
{
	FALSE = 0, 
	TRUE = !FALSE
} FalseStatus;

//typedef unsigned char       bool;

/* Exported types ------------------------------------------------------------*/
typedef enum i2c_result
{
  NO_ERR  = 0,  
  TIMEOUT = 1,
  BUS_BUSY = 2,
  SEND_START_ERR = 3,
  ADDR_MATCH_ERR = 4,
  ADDR_HEADER_MATCH_ERR = 5,
  DATA_TIMEOUT = 6,
  WAIT_COMM = 7,
  STOP_TIMEOUT = 8

}I2C_Result;

typedef enum i2c_state
{
  COMM_DONE  = 0,  // done successfully
  COMM_PRE = 1,
  COMM_IN_PROCESS = 2,
  CHECK_IN_PROCESS = 3,
  COMM_EXIT = 4 // exit since failure
    
}I2C_STATE;

extern I2C_STATE i2c_comm_state;
extern vu8 MasterReceptionComplete;
extern vu8 MasterTransitionComplete; 
extern vu8 WriteComplete;

void  I2C1_Comm_Init(u32 I2C_Speed, u16 I2C_Addr);


uint8_t I2C_Master_BufferRead(uint8_t* pBuffer, uint32_t NumByteToRead, I2C_ProgrammingModel Mode, uint8_t SlaveAddress, uint32_t timeoutMs);
uint8_t I2C_Master_BufferWrite(uint8_t* pBuffer, uint32_t NumByteToWrite, I2C_ProgrammingModel Mode, uint8_t SlaveAddress, uint32_t timeoutMs);
void I2C_Slave_BufferReadWrite(I2C_ProgrammingModel Mode);


uint8_t i2cdevReadByte(uint8_t devAddress, uint8_t memAddress, uint8_t *data);
uint8_t i2cdevRead(uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data);


uint8_t i2cdevWriteByte(uint8_t devAddress, uint8_t memAddress, uint8_t data);
uint8_t i2cdevWrite(uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data);
uint8_t i2cdevWriteBit(uint8_t devAddress, uint8_t memAddress, uint8_t bit, uint8_t bit_data);
uint8_t i2cdevWriteBits(uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t bit_data);


void i2cInterruptHandlerI2c1(void);
void i2cErrorInterruptHandlerI2c1(void);

#define i2cWrite(a,b,c,d) i2cdevWrite(a,b,c,d)
#define i2cWriteByte(a,b,c) i2cdevWriteByte(a,b,c)

#define i2cWriteBit(a,b,c,d) i2cdevWriteBit(a,b,c,d)
#define i2cWriteBits(a,b,c,d,e) i2cdevWriteBits(a,b,c,d,e)

#define i2cRead(a,b,c,d) i2cdevRead(a,b,c,d)
#define i2cReadByte(a,b,c) i2cdevReadByte(a,b,c)

#endif // __iic_h__

