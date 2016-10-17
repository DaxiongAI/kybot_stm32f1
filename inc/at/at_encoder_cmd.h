#ifndef __AT_ENCODER_CMD_H__
#define __AT_ENCODER_CMD_H__

#include "stm32f10x.h"

void at_encoder_query(uint8_t id);
void at_runtick_set(uint8_t id, uint8_t *pPara);

#endif
