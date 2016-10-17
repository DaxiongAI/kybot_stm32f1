#ifndef __AT_UTILS_H__
#define __AT_UTILS_H__

#include "stm32f10x.h"

void echo_ok(int the_file_id, const char *func, int line);
void echo_error(int the_file_id, const char *func, int line);
void echo_no_fun(int the_file_id, const char *func, int line);
int number_valid(int the_file_id, const char *func, int line, uint8_t *src);
uint8_t *string_trim(uint8_t *s);

#endif
