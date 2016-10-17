#ifndef __Encoders_h__
#define __Encoders_h__

#include "stm32f10x.h"

typedef struct {
	volatile uint16_t left;
	volatile uint16_t right;
} encoder_t;

extern encoder_t encoder;

void encoder_init(void);
void encoder_reset(void);
double encoder_get_theta(void);
#endif
