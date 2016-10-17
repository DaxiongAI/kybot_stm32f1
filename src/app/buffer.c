//{{{ include
#include <string.h>
#include "usart.h"
#include "buffer.h"
#include "workdata.h"
#include "delay.h"
//}}}

//{{{ define
buffer_t ros_coframe_buffer;
uint8_t ros_send_buffer[BUFFER_SIZE];
uint8_t dbg_send_buffer[BUFFER_SIZE];
uint8_t is_pushing = 0;

dbg_send_buffer_t dbg_send_queue;
//}}}

//{{{ push
void buffer_push_u8(buffer_t *buffer, uint8_t b)
{
        buffer->data[buffer->pTop++] = b;
}
void buffer_push_u16(buffer_t *buffer, uint16_t b)
{
        buffer->data[buffer->pTop++] = b&0xff;
        buffer->data[buffer->pTop++] = ((b>>8)&0xff);
}
void buffer_push_u24(buffer_t *buffer, uint32_t b)
{
        buffer->data[buffer->pTop++] = b&0xff;
        buffer->data[buffer->pTop++] = ((b>>8)&0xff);
        buffer->data[buffer->pTop++] = ((b>>16)&0xff);
}
void buffer_push_u32(buffer_t *buffer, uint32_t b)
{
        buffer->data[buffer->pTop++] = b&0xff;
        buffer->data[buffer->pTop++] = ((b>>8)&0xff);
        buffer->data[buffer->pTop++] = ((b>>16)&0xff);
        buffer->data[buffer->pTop++] = ((b>>24)&0xff);
}
//}}}

//{{{ size copy
uint8_t buffer_size(buffer_t *buffer)
{
        return buffer->pTop;
}
void buffer_copy(buffer_t *buffer, uint8_t * src, int size)
{
	memcpy(buffer->data, src, size);
	buffer->pTop = size;
}
//}}}

//{{{ reset send
void ros_coframe_buffer_reset(void) 
{
	buffer_t *buffer = &ros_coframe_buffer;
	
        buffer->pTop = 0;

        buffer_push_u8(buffer,HEADER0);
        buffer_push_u8(buffer,HEADER1);

        buffer_push_u8(buffer,0);
}
void ros_coframe_buffer_send(void)
{
        buffer_t *buffer = &ros_coframe_buffer;

        buffer->data[2] = buffer_size(buffer) - 3;
        uint8_t checksum = 0;
        for (uint32_t i = 2; i < buffer_size(buffer); i++)
                checksum ^= (buffer->data[i]);

        buffer_push_u8(buffer,checksum);
        while(!uart1_dma_has_txed){//等待上一次发送完成
                // delay_ms(1);
                // log_info("delay");  
        }
        memcpy(ros_send_buffer, ros_coframe_buffer.data, BUFFER_SIZE);
        uart1_dma_start_tx(buffer_size(buffer));
}
void dbg_send_queue_push(uint8_t *src, int size)
{
	is_pushing = 1;
	uint16_t pTail_inc = (dbg_send_queue.pTail + 1) % DBG_SEND_QUEUE_SIZE;
	while(dbg_send_queue.pHead == pTail_inc){//如果队列满
		dbg_buffer_send();// 启动发送，不然会导致死锁，因为is_pushing的存在
	}
	buffer_t *buffer = &dbg_send_queue.buffer[dbg_send_queue.pTail];
	/*memset(buffer->data, 0, BUFFER_SIZE);*/
	memcpy(buffer->data, src, size);
	buffer->pTop = size;
	dbg_send_queue.pTail = pTail_inc;

	// 如果没有is_pushing限制DMA中断调用dbg_buffer_send()，运行到dbg_buffer_send()函数的时候DMA中断来了会导致出错
	dbg_buffer_send();
	is_pushing = 0;
}
void dbg_buffer_send(void)
{
	if(!uart3_dma_has_txed)//等待上一次发送完成
		return;
	if(dbg_send_queue.pHead != dbg_send_queue.pTail) { //如果队列不空

		buffer_t *buffer = &dbg_send_queue.buffer[dbg_send_queue.pHead];
		uint16_t size = buffer_size(buffer);
		memcpy(dbg_send_buffer, buffer->data, size);
		uart3_dma_start_tx(size);
		uint16_t pHead_inc = (dbg_send_queue.pHead + 1) % DBG_SEND_QUEUE_SIZE;
		// dbg_buffer_send()被dbg_send_queue_push()调用时，这里被中断了会导致单个字节会重复发送两次，并且会导致队列满的情况，实际上队列并没有满
		// 中断出错
		dbg_send_queue.pHead = pHead_inc;
		/*memset(buffer->data, 0, BUFFER_SIZE);*/
		/*buffer->pTop = 0;*/
	}
}
//}}}
