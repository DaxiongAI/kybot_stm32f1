#ifndef __ADC_H
#define	__ADC_H

#include "stm32f10x.h"

// ���/kobuki_driver/src/driver/battery.cpp
// kobuki ��ض��ѹ16.5V
#define KOBUKI_BAT_CAPACITY 	165
// �������ĵ�ض��ѹ����Ӧkobuki��16.5V
#define KYBOT_BAT_CAPACITY	84
// �͵�ѹ7.2V����Ӧkobuki��14.0V
#define KYBOT_BAT_LOW		72
// Σ�յ�ѹ6.8V����Ӧkobuki��13.2V
#define KYBOT_BAT_DANGEROUS	68

extern __IO u16 ADC_ConvertedValue;

void adc1_init(void);
uint8_t get_battery_vol(void);
uint32_t get_battery_value(void);


#endif /* __ADC_H */

