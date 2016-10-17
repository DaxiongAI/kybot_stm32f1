//{{{ include
#include <stm32f10x.h>
#include "led.h"
//}}}

//{{{ define
static const char *m_color_name[] = {
"off",
"red",
"green",
"blue"
};
//}}}

//{{{ tostring
const char *led_color_to_str(int color)
{
        return m_color_name[color];
}
//}}}

//{{{ init
void led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin   = RUN_PIN;
	GPIO_Init(RUN_GPIO, &GPIO_InitStructure);

	run_off();

        GPIO_InitStructure.GPIO_Pin   = LED1_RED_PIN;
        GPIO_Init(LED1_RED_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin   = LED1_GREEN_PIN;
        GPIO_Init(LED1_GREEN_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin   = LED1_BLUE_PIN;
        GPIO_Init(LED1_BLUE_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin   = LED2_RED_PIN;
        GPIO_Init(LED2_RED_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin   = LED2_GREEN_PIN;
        GPIO_Init(LED2_GREEN_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin   = LED2_BLUE_PIN;
        GPIO_Init(LED2_BLUE_GPIO, &GPIO_InitStructure);

        GPIO_SetBits(LED1_RED_GPIO, LED1_RED_PIN);
        GPIO_SetBits(LED1_GREEN_GPIO, LED1_GREEN_PIN);
        GPIO_SetBits(LED1_BLUE_GPIO, LED1_BLUE_PIN);
        GPIO_SetBits(LED2_RED_GPIO, LED2_RED_PIN);
        GPIO_SetBits(LED2_GREEN_GPIO, LED2_GREEN_PIN);
        GPIO_SetBits(LED2_BLUE_GPIO, LED2_BLUE_PIN);
}
//}}}

//{{{ on, off
void run_off(void)
{
	GPIO_SetBits(RUN_GPIO, RUN_PIN);
}

void run_on(void)
{
	GPIO_ResetBits(RUN_GPIO, RUN_PIN);
}
//}}}

//{{{ setter
void led_set_mutex(enum led ledn, enum led_color color)
{
        if (ledn == led1) {
                GPIO_SetBits(LED1_RED_GPIO, LED1_RED_PIN);
                GPIO_SetBits(LED1_GREEN_GPIO, LED1_GREEN_PIN);
                GPIO_SetBits(LED1_BLUE_GPIO, LED1_BLUE_PIN);
        }
        else {
                GPIO_SetBits(LED2_RED_GPIO, LED2_RED_PIN);
                GPIO_SetBits(LED2_GREEN_GPIO, LED2_GREEN_PIN);
                GPIO_SetBits(LED2_BLUE_GPIO, LED2_BLUE_PIN);
        }
        switch(color)
        {
        case off:
                break;
        case red:
                if (ledn == led1)
                       GPIO_ResetBits(LED1_RED_GPIO, LED1_RED_PIN);
                else
                       GPIO_ResetBits(LED2_RED_GPIO, LED2_RED_PIN);
                break;
        case green:
                if (ledn == led1)
                       GPIO_ResetBits(LED1_GREEN_GPIO, LED1_GREEN_PIN);
                else
                       GPIO_ResetBits(LED2_GREEN_GPIO, LED2_GREEN_PIN);
                break;
        case blue:
                if (ledn == led1)
                       GPIO_ResetBits(LED1_BLUE_GPIO, LED1_BLUE_PIN);
                else
                       GPIO_ResetBits(LED2_BLUE_GPIO, LED2_BLUE_PIN);
                break;
        default:break;
        }
}
void led_set(enum led ledn, enum led_color color, uint8_t clear)
{
        if (clear) {
                GPIO_SetBits(LED1_RED_GPIO, LED1_RED_PIN);
                GPIO_SetBits(LED1_GREEN_GPIO, LED1_GREEN_PIN);
                GPIO_SetBits(LED1_BLUE_GPIO, LED1_BLUE_PIN);
                GPIO_SetBits(LED2_RED_GPIO, LED2_RED_PIN);
                GPIO_SetBits(LED2_GREEN_GPIO, LED2_GREEN_PIN);
                GPIO_SetBits(LED2_BLUE_GPIO, LED2_BLUE_PIN);
        }
        switch(color)
        {
        case off:
                break;
        case red:
                if (ledn == led1)
                        GPIO_ResetBits(LED1_RED_GPIO, LED1_RED_PIN);
                else if (ledn == led2)
                        GPIO_ResetBits(LED2_RED_GPIO, LED2_RED_PIN);
                else if (ledn == led12){
                        GPIO_ResetBits(LED1_RED_GPIO, LED1_RED_PIN);
                        GPIO_ResetBits(LED2_RED_GPIO, LED2_RED_PIN);
                }
                break;
        case green:
                if (ledn == led1)
                        GPIO_ResetBits(LED1_GREEN_GPIO, LED1_GREEN_PIN);
                else if (ledn == led2)
                        GPIO_ResetBits(LED2_GREEN_GPIO, LED2_GREEN_PIN);
                else if (ledn == led12){
                        GPIO_ResetBits(LED1_GREEN_GPIO, LED1_GREEN_PIN);
                        GPIO_ResetBits(LED2_GREEN_GPIO, LED2_GREEN_PIN);
                }
                break;
        case blue:
                if (ledn == led1)
                        GPIO_ResetBits(LED1_BLUE_GPIO, LED1_BLUE_PIN);
                else if (ledn == led2)
                        GPIO_ResetBits(LED2_BLUE_GPIO, LED2_BLUE_PIN);
                else if (ledn == led12){
                        GPIO_ResetBits(LED1_BLUE_GPIO, LED1_BLUE_PIN);
                        GPIO_ResetBits(LED2_BLUE_GPIO, LED2_BLUE_PIN);
                }
                break;
        default:break;
        }
}
//}}}
