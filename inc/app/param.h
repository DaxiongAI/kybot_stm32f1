#ifndef __PARAM_H__
#define __PARAM_H__

#include "usart.h"
#include "led.h"
#include "eeprom.h"
#include "adc.h"

void ee_init(void);
void ee_save_pid(void);
void ee_read_pid(void);
void ee_save_loglv(void);
void ee_read_loglv(void);
void disable_read_out_protection(void);
void ee_save_swprot(void);
void ee_read_swprot(void);
void ee_save_encrypt_key(uint32_t key0, uint32_t key1);
void ee_read_encrypt_key(void);
void param_load(void);
uint8_t is_first_on(void);
void set_not_first_on(void);

void ee_read_wheel(void);
void ee_save_wheel(void);
void ee_save_wheel_bias(void);
void ee_save_wheel_diameter(void);
void ee_save_wheel_ticks(void);

void ee_save_heading_factor(uint32_t factor);
void ee_read_heading_factor(void);
void ee_save_logclr(void);
void ee_read_logclr(void);
//void ee_save_imuact(void);
//void ee_read_imuact(void);

uint8_t is_need_rename_dbg_bt(void);
void clean_rename_dbg_bt_flag(void);
void save_dbg_bt_name(uint8_t name[7]);
void read_dbg_bt_name(uint8_t name[7]);
uint8_t is_need_rename_ros_bt(void);
void clean_rename_ros_bt_flag(void);
void save_ros_bt_name(uint8_t name[7]);
void read_ros_bt_name(uint8_t name[7]);

#endif
