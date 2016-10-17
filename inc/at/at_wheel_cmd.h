#ifndef __AT_WHEEL_CMD_H__
#define __AT_WHEEL_CMD_H__

#include "stm32f10x.h"

void at_base_query(uint8_t id);
void at_base_set(uint8_t id, uint8_t *pParam);

void at_bias_query(uint8_t id);
void at_bias_set(uint8_t id, uint8_t *pParam);

void at_radius_query(uint8_t id);
void at_radius_set(uint8_t id, uint8_t *pParam);

void at_ticks_query(uint8_t id);
void at_ticks_set(uint8_t id, uint8_t *pParam);
#endif
