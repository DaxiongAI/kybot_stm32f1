#ifndef __ENCRYPT_H__
#define __ENCRYPT_H__

#include "stm32f10x.h"

extern uint32_t read_encrypt_key0, read_encrypt_key1;
void disable_read_out_protection(void);
void enable_read_out_protection(void);
void check_chip_serial_number(void);
void set_encrypt(uint32_t key0, uint32_t key1);
void read_serial_number0(void);
void read_serial_number1(void);
void read_serial_number2(void);
void gen_encryption_key0(void);
void gen_encryption_key1(void);

#endif
