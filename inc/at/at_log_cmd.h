#ifndef __AT_LOG_CMD_H__
#define __AT_LOG_CMD_H__

#include "stm32f10x.h"

void at_log_set(uint8_t id, uint8_t *pPara);
void at_log_query(uint8_t id);
void at_logclr_set(uint8_t id, uint8_t *pPara);

#endif
