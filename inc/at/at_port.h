#ifndef __AT_PORT_H__
#define __AT_PORT_H__
#include "stm32f10x.h"

//{{{ define
#define AT_CMD_LEN_MAX          (64)
#define AT_DATA_LEN_MAX         (512)
//}}}

typedef struct {
        int ix;
        uint8_t data[AT_CMD_LEN_MAX];
} at_buffer_t;

void at_recv(uint8_t rx_char);
void at_process_task(void);

extern at_buffer_t at_cmd_line;

#endif
