#ifndef __ADC_H
#define	__ADC_H

#include "stm32f10x.h"

// 详见/kobuki_driver/src/driver/battery.cpp
// kobuki 电池额定电压16.5V
#define KOBUKI_BAT_CAPACITY 	165
// 本机器的电池额定电压，对应kobuki的16.5V
#define KYBOT_BAT_CAPACITY	84
// 低电压7.2V，对应kobuki的14.0V
#define KYBOT_BAT_LOW		72
// 危险电压6.8V，对应kobuki的13.2V
#define KYBOT_BAT_DANGEROUS	68

extern __IO u16 ADC_ConvertedValue;

void adc1_init(void);
uint8_t get_battery_vol(void);
uint32_t get_battery_value(void);


#endif /* __ADC_H */

