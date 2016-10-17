#ifndef  __DEFRAME_TASK_H__
#define  __DEFRAME_TASK_H__

#include "stm32f10x.h"

extern uint8_t ros_on_comm;

void deframe_task(void);
void analyze_buffer(void);
void deframe(uint8_t *payload,uint8_t size);
void base_control(int16_t speed, int16_t radius);

#endif
