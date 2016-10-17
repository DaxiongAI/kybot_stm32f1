#ifndef __BUFFER_H__
#define __BUFFER_H__

#include "stm32f10x.h"

#define HEADER0 0xaa
#define HEADER1 0x55

#define BUFFER_SIZE  256
#define DBG_SEND_QUEUE_SIZE  30

typedef struct {
	
	uint8_t data[BUFFER_SIZE];
	uint8_t pTop;
	
} buffer_t;//栈数据结构

typedef struct {

  uint16_t pHead;
  uint16_t pTail;
  buffer_t buffer[DBG_SEND_QUEUE_SIZE];//循环队列缓冲区

} dbg_send_buffer_t;

extern buffer_t ros_coframe_buffer;
extern uint8_t ros_send_buffer[BUFFER_SIZE];
extern uint8_t dbg_send_buffer[BUFFER_SIZE];

uint8_t buffer_size(buffer_t *buffer);

void ros_coframe_buffer_reset(void);
void ros_coframe_buffer_send(void);

void dbg_buffer_send(void);
void dbg_send_queue_push(uint8_t *src, int size);

void buffer_push_u8(buffer_t *buffer,uint8_t b);
void buffer_push_u16(buffer_t *buffer,uint16_t b);
void buffer_push_u24(buffer_t *buffer,uint32_t b);
void buffer_push_u32(buffer_t *buffer,uint32_t b);
void buffer_copy(buffer_t *buffer, uint8_t * src, int size);
extern uint8_t is_pushing;

#endif
