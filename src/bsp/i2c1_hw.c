#include "i2c1_hw.h"
#include "stm32f10x_i2c.h"
/******* specified by user ************/
//#define I2C_REMAP
//#define SLAVE_10BIT_ADDRESS

/******* specify by user ************/

#define I2C_TIMEOUT 5
#define I2CDEV_CLK_TS (1000000 / 100000)

#define Transmitter             0x00
#define Receiver                0x01

#define FALSE 0
#define TRUE 1

__IO uint8_t Address;
__IO uint32_t I2CDirection = I2C_DIRECTION_TX;
__IO uint32_t NumbOfBytes1;
/* Buffer of data to be received by I2C1 */
uint8_t* Buffer_Rx1;
/* Buffer of data to be transmitted by I2C1 */
uint8_t* Buffer_Tx1;

__IO uint8_t Tx_Idx1 = 0, Rx_Idx1 = 0;

static void i2cdevRuffLoopDelay(uint32_t us)
{
        volatile uint32_t delay;

        for(delay = I2CDEV_LOOPS_PER_US * us; delay > 0; delay--);
}

static void i2cdevResetBusI2c1(void)
{
        /* Make sure the bus is free by clocking it until any slaves release the bus. */
        GPIO_InitTypeDef  GPIO_InitStructure;
        /* Reset the I2C block */
        I2C_DeInit(I2C1);

        /* I2C1 clock enable */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        /* I2C1 SDA configuration */
        GPIO_InitStructure.GPIO_Pin = I2CDEV_I2C1_PIN_SDA;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
        /* I2C1 SCL configuration */
        GPIO_InitStructure.GPIO_Pin = I2CDEV_I2C1_PIN_SCL;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SDA);
        /* Check SDA line to determine if slave is asserting bus and clock out if so */
        while(GPIO_ReadInputDataBit(GPIOB, I2CDEV_I2C1_PIN_SDA) == Bit_RESET)
        {
                /* Set clock high */
                GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
                /* Wait for any clock stretching to finish. */
                GPIO_WAIT_LOW(GPIOB, I2CDEV_I2C1_PIN_SCL, 10 * I2CDEV_LOOPS_PER_MS);
                i2cdevRuffLoopDelay(I2CDEV_CLK_TS);

                /* Generate a clock cycle */
                GPIO_ResetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
                i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
                GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
                i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
        }

        /* Generate a start then stop condition */
        GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
        i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
        GPIO_ResetBits(GPIOB, I2CDEV_I2C1_PIN_SDA);
        i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
        GPIO_ResetBits(GPIOB, I2CDEV_I2C1_PIN_SDA);
        i2cdevRuffLoopDelay(I2CDEV_CLK_TS);

        /* Set data and clock high and wait for any clock stretching to finish. */
        GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SDA);
        GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
        GPIO_WAIT_LOW(GPIOB, I2CDEV_I2C1_PIN_SCL, 10 * I2CDEV_LOOPS_PER_MS);
        /* Wait for data to be high */
        GPIO_WAIT_HIGH(GPIOB, I2CDEV_I2C1_PIN_SDA, 10 * I2CDEV_LOOPS_PER_MS);

}

void  I2C1_Comm_Init(u32 I2C_Speed, u16 I2C_Addr)
{
        /******* GPIO configuration and clock enable *********/
        GPIO_InitTypeDef  GPIO_InitStructure; 
        I2C_InitTypeDef  I2C_InitStructure;

        i2cdevResetBusI2c1();

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_AHBPeriph_FSMC, DISABLE);

#ifdef I2C_REMAP
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
#else
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
#endif
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        /*********** I2C periphral configuration **********/
        I2C_InitStructure.I2C_Mode = I2C_Mode_I2C; // fixed
        I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;  // fixed
        I2C_InitStructure.I2C_OwnAddress1 = I2C_Addr;  // user parameter
        I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; // fixed
#ifdef SLAVE_10BIT_ADDRESS  
        I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_10bit;  // user define
#else
        I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
#endif
        I2C_InitStructure.I2C_ClockSpeed = I2C_Speed; // user parameter

        I2C_Init(I2C1, &I2C_InitStructure);
        I2C_Cmd(I2C1, ENABLE);

        //	I2C1->CR2 = (I2C1->CR2 & 0xff00)|0x24;

#define I2C_BUSY 0x20
        if (I2C1->SR2 & I2C_BUSY)
        {
                /* Reset the I2C block */
                I2C_SoftwareResetCmd(I2C1, ENABLE);
                I2C_SoftwareResetCmd(I2C1, DISABLE);
        }
}



/**
 * @brief  Reads buffer of bytes  from the slave.
 * @param pBuffer: Buffer of bytes to be read from the slave.
 * @param NumByteToRead: Number of bytes to be read by the Master.
 * @param Mode: Polling or DMA or Interrupt having the highest priority in the application.
 * @param SlaveAddress: The address of the slave to be addressed by the Master.
 * @retval : None.
 */
uint8_t I2C_Master_BufferRead(uint8_t* pBuffer, uint32_t NumByteToRead, I2C_ProgrammingModel Mode, uint8_t SlaveAddress, uint32_t timeoutMs)

{
        // __IO uint32_t temp = 0;
        __IO uint32_t Timeout = 0;

        /* Enable I2C errors interrupts (used in all modes: Polling, DMA and Interrupts */
        I2C1->CR2 |= I2C_IT_ERR;

        /* I2Cx Master Reception using Interrupts with highest priority in an application */
        // else
        {
                /* Enable EVT IT*/
                I2C1->CR2 |= I2C_IT_EVT;
                /* Enable BUF IT */
                I2C1->CR2 |= I2C_IT_BUF;
                /* Set the I2C direction to reception */
                I2CDirection = I2C_DIRECTION_RX;
                Buffer_Rx1 = pBuffer;
                SlaveAddress |= OAR1_ADD0_Set;
                Address = SlaveAddress;
                NumbOfBytes1 = NumByteToRead;
                /* Send START condition */
                I2C1->CR1 |= CR1_START_Set;
                Timeout = timeoutMs * I2CDEV_LOOPS_PER_MS;
                /* Wait until the START condition is generated on the bus: START bit is cleared by hardware */
                while ((I2C1->CR1 & 0x100) == 0x100 && Timeout)
                {
                        Timeout--;
                }
                /* Wait until BUSY flag is reset (until a STOP is generated) */
                while ((I2C1->SR2 & 0x0002) == 0x0002 && Timeout)
                {
                        Timeout--;
                }
                /* Enable Acknowledgement to be ready for another reception */
                I2C1->CR1 |= CR1_ACK_Set;

                if (Timeout == 0)
                        return 1;
        }

        return 0;
        //  temp++; //To avoid GCC warning!
}

/**
 * @brief  Writes buffer of bytes.
 * @param pBuffer: Buffer of bytes to be sent to the slave.
 * @param NumByteToWrite: Number of bytes to be sent by the Master.
 * @param Mode: Polling or DMA or Interrupt having the highest priority in the application.
 * @param SlaveAddress: The address of the slave to be addressed by the Master.
 * @retval : None.
 */
uint8_t I2C_Master_BufferWrite(uint8_t* pBuffer, uint32_t NumByteToWrite, I2C_ProgrammingModel Mode, uint8_t SlaveAddress, uint32_t timeoutMs)
{

        // __IO uint32_t temp = 0;
        __IO uint32_t Timeout = 0;

        /* Enable Error IT (used in all modes: DMA, Polling and Interrupts */
        I2C1->CR2 |= I2C_IT_ERR;
        /* I2Cx Master Transmission using Interrupt with highest priority in the application */
        // else
        {
                /* Enable EVT IT*/
                I2C1->CR2 |= I2C_IT_EVT;
                /* Enable BUF IT */
                I2C1->CR2 |= I2C_IT_BUF;
                /* Set the I2C direction to Transmission */
                I2CDirection = I2C_DIRECTION_TX;
                Buffer_Tx1 = pBuffer;
                SlaveAddress &= OAR1_ADD0_Reset;
                Address = SlaveAddress;
                NumbOfBytes1 = NumByteToWrite;
                /* Send START condition */
                I2C1->CR1 |= CR1_START_Set;
                Timeout = timeoutMs * I2CDEV_LOOPS_PER_MS;
                /* Wait until the START condition is generated on the bus: the START bit is cleared by hardware */
                while ((I2C1->CR1 & 0x100) == 0x100 && Timeout)
                {
                        Timeout--;
                }
                /* Wait until BUSY flag is reset: a STOP has been generated on the bus signaling the end
                   of transmission */
                while ((I2C1->SR2 & 0x0002) == 0x0002 && Timeout)
                {
                        Timeout--;
                }

                if (Timeout == 0)
                        return 1;
        }
        return 0;

        //  temp++; //To avoid GCC warning!
}

uint8_t i2cdevReadByte(uint8_t devAddress, uint8_t memAddress, uint8_t *data)
{
        return i2cdevRead(devAddress, memAddress, 1, data);
}

uint8_t i2cdevRead(uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data)
{
        uint8_t status = 0;

        if (memAddress != I2CDEV_NO_MEM_ADDR)
        {
                status = I2C_Master_BufferWrite(&memAddress,  1, INTERRUPT, devAddress << 1, I2C_TIMEOUT);
        }
        if (!status)
        {
                //TODO: Fix DMA transfer if more then 3 bytes
                status = I2C_Master_BufferRead((uint8_t*)data,  len, INTERRUPT, devAddress << 1, I2C_TIMEOUT);
        }

        return status;
}

uint8_t i2cdevWriteByte(uint8_t devAddress, uint8_t memAddress, uint8_t data)
{
        return i2cdevWrite(devAddress, memAddress, 1, &data);
}

uint8_t i2cdevWrite(uint8_t devAddress, uint8_t memAddress, uint16_t len, uint8_t *data)
{
        uint8_t status;
        static uint8_t buffer[17];
        int i;

        if (memAddress != I2CDEV_NO_MEM_ADDR)
        {
                // Sorry ...
                if (len > 16) len = 16;

                if(len == 0) return 1;

                buffer[0] = memAddress;
                for(i = 0; i < len ; i++)
                        buffer[i + 1] = data[i];

                status = I2C_Master_BufferWrite(buffer,  len + 1, INTERRUPT, devAddress << 1, I2C_TIMEOUT);
        }
        else
        {
                status = I2C_Master_BufferWrite(data,  len, INTERRUPT, devAddress << 1, I2C_TIMEOUT);
        }

        return status;
}

uint8_t i2cdevWriteBit(uint8_t devAddress, uint8_t memAddress, uint8_t bit, uint8_t bit_data) 
{
	uint8_t register_data, status;
	status = i2cdevRead(devAddress, memAddress, 1, &register_data);
	register_data = (bit_data != 0) ? (register_data | (1 << bit)) : (register_data & ~(1 << bit));
	status = i2cdevWrite(devAddress, memAddress, 1, &register_data);
        return status;
}
uint8_t i2cdevWriteBits(uint8_t devAddress, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t bit_data)
{
        uint8_t register_data, mask, status;
        status = i2cdevRead(devAddress, memAddress, 1, &register_data);
        mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        bit_data <<= (8 - length);
	bit_data >>= (7 - bitStart);
	register_data &= mask;
	register_data |= bit_data;
	status = i2cdevWrite(devAddress, memAddress, 1, &register_data);
        return status;
}
/**
 * @brief  This function handles I2C1 Event interrupt request.
 * @param  None
 * @retval : None
 */
void i2cInterruptHandlerI2c1(void)
{

        __IO uint32_t SR1Register = 0;
        __IO uint32_t SR2Register = 0;

        /* Read the I2C1 SR1 and SR2 status registers */
        SR1Register = I2C1->SR1;
        SR2Register = I2C1->SR2;

        /* If SB = 1, I2C1 master sent a START on the bus: EV5) */
        if ((SR1Register & 0x0001) == 0x0001)
        {
                /* Send the slave address for transmssion or for reception (according to the configured value
                   in the write master write routine */
                I2C1->DR = Address;
                SR1Register = 0;
                SR2Register = 0;
        }
        /* If I2C1 is Master (MSL flag = 1) */

        if ((SR2Register & 0x0001) == 0x0001)
        {
                /* If ADDR = 1, EV6 */
                if ((SR1Register & 0x0002) == 0x0002)
                {
                        /* Write the first data in case the Master is Transmitter */
                        if (I2CDirection == I2C_DIRECTION_TX)
                        {
                                /* Initialize the Transmit counter */
                                Tx_Idx1 = 0;
                                /* Write the first data in the data register */
                                I2C1->DR = Buffer_Tx1[Tx_Idx1++];
                                /* Decrement the number of bytes to be written */
                                NumbOfBytes1--;
                                /* If no further data to be sent, disable the I2C BUF IT
                                   in order to not have a TxE  interrupt */
                                if (NumbOfBytes1 == 0)
                                {
                                        I2C1->CR2 &= (uint16_t) ~I2C_IT_BUF;
                                }
                        }
                        /* Master Receiver */
                        else
                        {
                                /* Initialize Receive counter */
                                Rx_Idx1 = 0;
                                /* At this stage, ADDR is cleared because both SR1 and SR2 were read.*/
                                /* EV6_1: used for single byte reception. The ACK disable and the STOP
                                   Programming should be done just after ADDR is cleared. */
                                if (NumbOfBytes1 == 1)
                                {
                                        /* Clear ACK */
                                        I2C1->CR1 &= CR1_ACK_Reset;
                                        /* Program the STOP */
                                        I2C1->CR1 |= CR1_STOP_Set;
                                }
                        }
                        SR1Register = 0;
                        SR2Register = 0;
                }
                /* Master transmits the remaing data: from data2 until the last one.  */
                /* If TXE is set */
                if ((SR1Register & 0x0084) == 0x0080)
                {
                        /* If there is still data to write */
                        if (NumbOfBytes1 != 0)
                        {
                                /* Write the data in DR register */
                                I2C1->DR = Buffer_Tx1[Tx_Idx1++];
                                /* Decrment the number of data to be written */
                                NumbOfBytes1--;
                                /* If  no data remains to write, disable the BUF IT in order
                                   to not have again a TxE interrupt. */
                                if (NumbOfBytes1 == 0)
                                {
                                        /* Disable the BUF IT */
                                        I2C1->CR2 &= (uint16_t) ~I2C_IT_BUF;
                                }
                        }
                        SR1Register = 0;
                        SR2Register = 0;
                }
                /* If BTF and TXE are set (EV8_2), program the STOP */
                if ((SR1Register & 0x0084) == 0x0084)
                {
                        /* Program the STOP */
                        I2C1->CR1 |= CR1_STOP_Set;
                        /* Disable EVT IT In order to not have again a BTF IT */
                        I2C1->CR2 &= (uint16_t) ~I2C_IT_EVT;
                        SR1Register = 0;
                        SR2Register = 0;
                }
                /* If RXNE is set */
                if ((SR1Register & 0x0040) == 0x0040)
                {
                        /* Read the data register */
                        Buffer_Rx1[Rx_Idx1++] = I2C1->DR;
                        /* Decrement the number of bytes to be read */
                        NumbOfBytes1--;
                        /* If it remains only one byte to read, disable ACK and program the STOP (EV7_1) */
                        if (NumbOfBytes1 == 1)
                        {
                                /* Clear ACK */
                                I2C1->CR1 &= CR1_ACK_Reset;
                                /* Program the STOP */
                                I2C1->CR1 |= CR1_STOP_Set;
                        }
                        SR1Register = 0;
                        SR2Register = 0;
                }
        }
}

/**
 * @brief  This function handles I2C1 Error interrupt request.
 * @param  None
 * @retval : None
 */
void i2cErrorInterruptHandlerI2c1(void)
{

        __IO uint32_t SR1Register = 0;

        /* Read the I2C1 status register */
        SR1Register = I2C1->SR1;
        /* If AF = 1 */
        if ((SR1Register & 0x0400) == 0x0400)
        {
                I2C1->SR1 &= 0xFBFF;
                SR1Register = 0;
        }
        /* If ARLO = 1 */
        if ((SR1Register & 0x0200) == 0x0200)
        {
                I2C1->SR1 &= 0xFBFF;
                SR1Register = 0;
        }
        /* If BERR = 1 */
        if ((SR1Register & 0x0100) == 0x0100)
        {
                I2C1->SR1 &= 0xFEFF;
                SR1Register = 0;
        }
        /* If OVR = 1 */
        if ((SR1Register & 0x0800) == 0x0800)
        {
                I2C1->SR1 &= 0xF7FF;
                SR1Register = 0;
        }
}

