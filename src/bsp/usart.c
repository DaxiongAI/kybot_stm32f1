//{{{ include
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "user_conf.h"
#include "buffer.h"
#include "usart.h"
#include "at_port.h"
//}}}

//{{{ declare
volatile u8 uart1_dma_has_txed = 1;//不应该初始化成0，见buffer_reset()
volatile u8 uart2_dma_has_txed = 1;//不应该初始化成0，见buffer_reset()
volatile u8 uart3_dma_has_txed = 1;//不应该初始化成0，见buffer_reset()

u8 uart1_dma_rx_buf[DMA_RX_BUFFER_SIZE];
u8 uart2_dma_rx_buf[DMA_RX_BUFFER_SIZE];
u8 uart3_dma_rx_buf[DMA_RX_BUFFER_SIZE];
//}}}

//{{{ uart1
void uart1_tx_dma_config(void)
{
        DMA_InitTypeDef  UART1_TX_DMA_InitStructure; 
        /* DMA channel6configuration */  
        DMA_DeInit(USART1_TX_DMA_CHANNEL);  
        UART1_TX_DMA_InitStructure.DMA_PeripheralBaseAddr       = (u32)(&USART1->DR);  //
        UART1_TX_DMA_InitStructure.DMA_MemoryBaseAddr           = (u32)ros_send_buffer;  
        UART1_TX_DMA_InitStructure.DMA_DIR                      = DMA_DIR_PeripheralDST;  // DMA_DIR_PeripheralSRC
        UART1_TX_DMA_InitStructure.DMA_BufferSize               = BUFFER_SIZE;// BufferSize  
        UART1_TX_DMA_InitStructure.DMA_PeripheralInc            = DMA_PeripheralInc_Disable;// DMA_PeripheralInc_Enable;
        UART1_TX_DMA_InitStructure.DMA_MemoryInc                = DMA_MemoryInc_Enable;    
        UART1_TX_DMA_InitStructure.DMA_PeripheralDataSize       = DMA_PeripheralDataSize_Byte; 
        UART1_TX_DMA_InitStructure.DMA_MemoryDataSize           = DMA_PeripheralDataSize_Byte; // DMA_MemoryDataSize_Word  
        UART1_TX_DMA_InitStructure.DMA_Mode                     = DMA_Mode_Normal; // DMA_Mode_Normal DMA_Mode_Circular
        UART1_TX_DMA_InitStructure.DMA_Priority                 = DMA_Priority_High;  
        UART1_TX_DMA_InitStructure.DMA_M2M                      = DMA_M2M_Disable;             // DMA_M2M_Enable;      

        DMA_Init(USART1_TX_DMA_CHANNEL, &UART1_TX_DMA_InitStructure);  

        /* Enable DMA Channel4Transfer Complete interrupt */  
        DMA_ITConfig(USART1_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);  	
}
void uart1_rx_dma_config(void)
{
        DMA_InitTypeDef  DMA_InitStructure; 

        DMA_DeInit(USART1_RX_DMA_CHANNEL);  
        DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);  
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart1_dma_rx_buf;  
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  
        DMA_InitStructure.DMA_BufferSize = DMA_RX_BUFFER_SIZE;  
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 

        DMA_Init(USART1_RX_DMA_CHANNEL,&DMA_InitStructure);  

        DMA_Cmd(USART1_RX_DMA_CHANNEL,ENABLE); 
}
void uart1_dma_start_tx(uint16_t size)
{
        //UART1_TX_DMA_InitStructure.DMA_BufferSize               = size;//BufferSize
        //DMA_Init(DMA1_Channel4, &UART1_TX_DMA_InitStructure);
        DMA_SetCurrDataCounter(USART1_TX_DMA_CHANNEL, size);
        uart1_dma_has_txed = 0;
        DMA_Cmd(USART1_TX_DMA_CHANNEL, ENABLE);
}
/* USART1 */
#define UART1_GPIO_TX		GPIO_Pin_9
#define UART1_GPIO_RX		GPIO_Pin_10
#define UART1_GPIO			GPIOA
void uart1_init(u32 bound)
{
        //GPIO端口设置
        GPIO_InitTypeDef GPIO_InitStructure;
        //USART1 初始化设置
        USART_InitTypeDef USART_InitStructure;
        //USART_ClockInitTypeDef USART_ClockInitStruct;		

        /* USART1 Pins configuration ************************************************/
        /* Configure USART Rx/tx PIN */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX;
        GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Pin = UART1_GPIO_TX;
        GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
        //配置USART1时钟
        //USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
        //USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
        //USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
        //USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
        //USART_ClockInit(USART1, &USART_ClockInitStruct);

        USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
        USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
        USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
        USART_Init(USART1, &USART_InitStructure); //初始化串口	/* Enable 8xUSARTs Receive interrupts */

        USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
        USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
        /* Enable the 8xUSARTs */
        USART_Cmd(USART1, ENABLE);//使能串口    
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);//开启中断  
        USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启中断  
        USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//开启中断             

}
//}}}

//{{{ uart2
void uart2_tx_dma_config(void)
{
        DMA_InitTypeDef  UART2_TX_DMA_InitStructure; 
        /* DMA channel6configuration */  
        DMA_DeInit(USART2_TX_DMA_CHANNEL);  
        UART2_TX_DMA_InitStructure.DMA_PeripheralBaseAddr       = (u32)(&USART2->DR);  //
        /*UART2_TX_DMA_InitStructure.DMA_MemoryBaseAddr           = (u32)ros_coframe_buffer.data;  */
        UART2_TX_DMA_InitStructure.DMA_DIR                      = DMA_DIR_PeripheralDST;  // DMA_DIR_PeripheralSRC
        UART2_TX_DMA_InitStructure.DMA_BufferSize               = BUFFER_SIZE;// BufferSize  
        UART2_TX_DMA_InitStructure.DMA_PeripheralInc            = DMA_PeripheralInc_Disable;// DMA_PeripheralInc_Enable;
        UART2_TX_DMA_InitStructure.DMA_MemoryInc                = DMA_MemoryInc_Enable;    
        UART2_TX_DMA_InitStructure.DMA_PeripheralDataSize       = DMA_PeripheralDataSize_Byte; 
        UART2_TX_DMA_InitStructure.DMA_MemoryDataSize           = DMA_PeripheralDataSize_Byte; // DMA_MemoryDataSize_Word  
        UART2_TX_DMA_InitStructure.DMA_Mode                     = DMA_Mode_Normal; // DMA_Mode_Normal DMA_Mode_Circular
        UART2_TX_DMA_InitStructure.DMA_Priority                 = DMA_Priority_High;  
        UART2_TX_DMA_InitStructure.DMA_M2M                      = DMA_M2M_Disable;             // DMA_M2M_Enable;      

        DMA_Init(USART2_TX_DMA_CHANNEL, &UART2_TX_DMA_InitStructure);  

        /* Enable DMA Channel4Transfer Complete interrupt */  
        DMA_ITConfig(USART2_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);  	
}
void uart2_rx_dma_config(void)
{
        DMA_InitTypeDef  DMA_InitStructure; 

        DMA_DeInit(USART2_RX_DMA_CHANNEL);  
        DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);  
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart2_dma_rx_buf;  
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  
        DMA_InitStructure.DMA_BufferSize = DMA_RX_BUFFER_SIZE;  
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 

        DMA_Init(USART2_RX_DMA_CHANNEL,&DMA_InitStructure);  

        DMA_Cmd(USART2_RX_DMA_CHANNEL,ENABLE); 
}
void uart2_dma_start_tx(uint16_t size)
{
        //UART2_TX_DMA_InitStructure.DMA_BufferSize               = size;//BufferSize
        //DMA_Init(DMA1_Channel4, &UART2_TX_DMA_InitStructure);
        DMA_SetCurrDataCounter(USART2_TX_DMA_CHANNEL, size);
        uart2_dma_has_txed = 0;
        DMA_Cmd(USART2_TX_DMA_CHANNEL, ENABLE);
}
// /* USART2 */
#define UART2_GPIO_TX	    GPIO_Pin_2
#define UART2_GPIO_RX	    GPIO_Pin_3
#define UART2_GPIO	    	GPIOA
void uart2_init(u32 bound)
{
        //GPIO端口设置
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;

        /* USART2 Pins configuration ************************************************/
        /* Configure USART Rx/tx PIN */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStructure.GPIO_Pin = UART2_GPIO_RX;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(UART2_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Pin = UART2_GPIO_TX;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(UART2_GPIO, &GPIO_InitStructure);

        //USART2 初始化设置
        USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
        USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
        USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

        USART_Init(USART2, &USART_InitStructure); //初始化串口

        USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
        USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
        /* Enable the 8xUSARTs */
        USART_Cmd(USART2, ENABLE);//使能串口 
        USART_ITConfig(USART2, USART_IT_TC, DISABLE);//开启中断  
        USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//开启中断  
        USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);//开启中断             

        /* Enable 8xUSARTs Receive interrupts */
        /*USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断*/
}
//}}}

//{{{ uart3
void uart3_tx_dma_config(void)
{
        DMA_InitTypeDef  UART3_TX_DMA_InitStructure; 
        /* DMA channel6configuration */  
        DMA_DeInit(USART3_TX_DMA_CHANNEL);  
        UART3_TX_DMA_InitStructure.DMA_PeripheralBaseAddr       = (u32)(&USART3->DR);  //
        UART3_TX_DMA_InitStructure.DMA_MemoryBaseAddr           = (u32)dbg_send_buffer;  
        UART3_TX_DMA_InitStructure.DMA_DIR                      = DMA_DIR_PeripheralDST;  // DMA_DIR_PeripheralSRC
        UART3_TX_DMA_InitStructure.DMA_BufferSize               = BUFFER_SIZE;// BufferSize  
        UART3_TX_DMA_InitStructure.DMA_PeripheralInc            = DMA_PeripheralInc_Disable;// DMA_PeripheralInc_Enable;
        UART3_TX_DMA_InitStructure.DMA_MemoryInc                = DMA_MemoryInc_Enable;    
        UART3_TX_DMA_InitStructure.DMA_PeripheralDataSize       = DMA_PeripheralDataSize_Byte; 
        UART3_TX_DMA_InitStructure.DMA_MemoryDataSize           = DMA_PeripheralDataSize_Byte; // DMA_MemoryDataSize_Word  
        UART3_TX_DMA_InitStructure.DMA_Mode                     = DMA_Mode_Normal; // DMA_Mode_Normal DMA_Mode_Circular
        UART3_TX_DMA_InitStructure.DMA_Priority                 = DMA_Priority_High;  
        UART3_TX_DMA_InitStructure.DMA_M2M                      = DMA_M2M_Disable;             // DMA_M2M_Enable;      

        DMA_Init(USART3_TX_DMA_CHANNEL, &UART3_TX_DMA_InitStructure);  

        /* Enable DMA Channel4Transfer Complete interrupt */  
        DMA_ITConfig(USART3_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);  	
}
void uart3_rx_dma_config(void)
{
        DMA_InitTypeDef  DMA_InitStructure; 

        DMA_DeInit(USART3_RX_DMA_CHANNEL);  
        DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DR);  
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart3_dma_rx_buf;  
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  
        DMA_InitStructure.DMA_BufferSize = DMA_RX_BUFFER_SIZE;  
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 

        DMA_Init(USART3_RX_DMA_CHANNEL,&DMA_InitStructure);  

        DMA_Cmd(USART3_RX_DMA_CHANNEL,ENABLE); 
}
void uart3_dma_start_tx(uint16_t size)
{
        //UART3_TX_DMA_InitStructure.DMA_BufferSize = size;//BufferSize
        //DMA_Init(DMA1_Channel4, &UART3_TX_DMA_InitStructure);
        DMA_SetCurrDataCounter(USART3_TX_DMA_CHANNEL, size);
        uart3_dma_has_txed = 0;
        DMA_Cmd(USART3_TX_DMA_CHANNEL, ENABLE);
}

#define UART3_GPIO_TX		GPIO_Pin_10
#define UART3_GPIO_RX		GPIO_Pin_11
#define UART3_GPIO			GPIOB
void uart3_init(u32 bound)
{
        //GPIO端口设置
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;

        /* USART2 Pins configuration ************************************************/
        /* Configure USART Rx/tx PIN */
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStructure.GPIO_Pin = UART3_GPIO_RX;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Pin = UART3_GPIO_TX;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

        //USART3 初始化设置
        USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
        USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
        USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

        USART_Init(USART3, &USART_InitStructure); //初始化串口

        USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
        USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
        /* Enable the 8xUSARTs */
        USART_Cmd(USART3, ENABLE);//使能串口    
        USART_ITConfig(USART3, USART_IT_TC, DISABLE);//开启中断  
        USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//开启中断  
        USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//开启中断             

}
//}}}

//{{{ bt
#ifdef SET_BT_BAUD
void set_dbg_bt_baudrate(uint32_t baudrate)
{
        char *str;
        uint8_t i=0;
        switch(baudrate){
        case 115200: str = "AT+BAUD8\r\n";break;

        }
	buffer_t tmp_buffer;
	tmp_buffer.pTop = 0;
        while(str[i] != '\0') {
                buffer_push_u8(&tmp_buffer, str[i++]);
        }
        dbg_send_queue_push(tmp_buffer.data, tmp_buffer.pTop);
}
void set_ros_bt_baudrate(uint32_t baudrate)
{
        char *str;
        uint8_t i=0;
        switch(baudrate){
        case 115200: str = "AT+BAUD8\r\n";break;

        }

        ros_coframe_buffer.pTop = 0;
        while(str[i] != '\0') {
                buffer_push_u8(&ros_coframe_buffer, str[i++]);
        }
        while(!uart1_dma_has_txed){//等待上一次发送完成
                // delay_ms(1);
                // log_info("delay");  
        }
        // ros_coframe_buffer 是组帧缓冲buffer
        memcpy(ros_send_buffer, ros_coframe_buffer.data, BUFFER_SIZE);
        uart1_dma_start_tx(buffer_size(&ros_coframe_buffer));
}
#endif
void set_dbg_bt_name(uint8_t name[7])
{
        uint8_t i=0;
        char debug[32] = {'\0'};
        strcat(debug, "AT+NAMEdbg-");
        strcat(debug, (const char*)name);
        strcat(debug, "\r\n");

	buffer_t tmp_buffer;
	tmp_buffer.pTop = 0;
        while(debug[i] != '\0') {
                buffer_push_u8(&tmp_buffer, debug[i++]);
        }
        dbg_send_queue_push(tmp_buffer.data, tmp_buffer.pTop);
}
void set_ros_bt_name(uint8_t name[7])
{
        uint8_t i=0;
        char ros[32] = {'\0'};
        strcat(ros, "AT+NAMEros-");
        strcat(ros, (const char*)name);
        strcat(ros, "\r\n");

        ros_coframe_buffer.pTop = 0;
        while(ros[i] != '\0') {
                buffer_push_u8(&ros_coframe_buffer, ros[i++]);
        }
        while(!uart1_dma_has_txed){//等待上一次发送完成
                // delay_ms(1);
                // log_info("delay");  
        }
        // ros_coframe_buffer 是组帧缓冲buffer
        memcpy(ros_send_buffer, ros_coframe_buffer.data, BUFFER_SIZE);
        uart1_dma_start_tx(buffer_size(&ros_coframe_buffer));
}

//}}}
