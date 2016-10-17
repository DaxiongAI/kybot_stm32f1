#ifndef __KEY_H__
#define __KEY_H__
/*
KEY_RST    PC14
*/

#define KEY_RST_GPIO 	  GPIOB
#define KEY_RST_PIN 	  GPIO_Pin_9
#define key_rst_GETVALUE()    GPIO_ReadInputDataBit(KEY_RST_GPIO, KEY_RST_PIN)

void key_init(void);

#endif
