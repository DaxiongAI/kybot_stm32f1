#ifndef __USART_H__
#define __USART_H__

#include "stm32f10x.h"
#include "user_conf.h"

#define USART1_TX_DMA_CHANNEL   DMA1_Channel4
#define USART1_RX_DMA_CHANNEL   DMA1_Channel5

#define USART2_TX_DMA_CHANNEL   DMA1_Channel7
#define USART2_RX_DMA_CHANNEL   DMA1_Channel6

#define USART3_TX_DMA_CHANNEL   DMA1_Channel2
#define USART3_RX_DMA_CHANNEL   DMA1_Channel3

void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);

void uart1_dma_start_tx(uint16_t size);
void uart1_rx_dma_config(void);
void uart1_tx_dma_config(void);

void uart2_dma_start_tx(uint16_t size);
void uart2_rx_dma_config(void);
void uart2_tx_dma_config(void);

void uart3_dma_start_tx(uint16_t size);
void uart3_rx_dma_config(void);
void uart3_tx_dma_config(void);

#ifdef SET_BT_BAUD
void set_dbg_bt_baudrate(uint32_t baudrate);
void set_ros_bt_baudrate(uint32_t baudrate);
#endif

void set_dbg_bt_name(uint8_t name[7]);
void set_ros_bt_name(uint8_t name[7]);

extern volatile u8 uart1_dma_has_txed;
extern volatile u8 uart2_dma_has_txed;
extern volatile u8 uart3_dma_has_txed;

extern u8 uart1_dma_rx_buf[DMA_RX_BUFFER_SIZE];
extern u8 uart2_dma_rx_buf[DMA_RX_BUFFER_SIZE];
extern u8 uart3_dma_rx_buf[DMA_RX_BUFFER_SIZE];

#endif
