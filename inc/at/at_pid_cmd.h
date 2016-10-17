#ifndef __AT_PID_CMD_H__
#define __AT_PID_CMD_H__

#include "stm32f10x.h"

void at_pid_query(uint8_t id);
void at_pid_set(uint8_t id, uint8_t *pPara);

#endif
