#ifndef __AT_ENCRYPT_CMD_H__
#define __AT_ENCRYPT_CMD_H__

#include "stm32f10x.h"

void at_encrypt_query(uint8_t id);
void at_encrypt_set(uint8_t id, uint8_t *pPara);

#endif
