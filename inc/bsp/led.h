#ifndef __LED_H__
#define __LED_H__

// led define
#define RUN_GPIO                        GPIOB
#define RUN_PIN                         GPIO_Pin_13

#define LED1_RED_GPIO                   GPIOC
#define LED1_RED_PIN                    GPIO_Pin_13

#define LED1_GREEN_GPIO                 GPIOC
#define LED1_GREEN_PIN                  GPIO_Pin_14

#define LED1_BLUE_GPIO                  GPIOC
#define LED1_BLUE_PIN                   GPIO_Pin_15

#define LED2_RED_GPIO                   GPIOA
#define LED2_RED_PIN                    GPIO_Pin_8

#define LED2_GREEN_GPIO                 GPIOB
#define LED2_GREEN_PIN                  GPIO_Pin_15

#define LED2_BLUE_GPIO                  GPIOB
#define LED2_BLUE_PIN                   GPIO_Pin_14

enum led{
        led1  = 0x01,
        led2  = 0x02,
        led12 = 0x03,
};
enum led_color{
        off   = 0x00,
        red   = 0x01,
        green = 0x02,
        blue  = 0x03,
};
void led_init(void);
void run_on(void);
void run_off(void);
void led_set(enum led ledn, enum led_color color, uint8_t clear);
void led_set_mutex(enum led ledn, enum led_color color);
const char *led_color_to_str(int color);

#endif
