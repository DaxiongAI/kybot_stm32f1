#ifndef __SHELL_H__
#define __SHELL_H__

#include "stm32f10x.h"

#define SHELL_HISTORY_LINES     20
#define SHELL_CMD_SIZE          40
#define SHELL_PROMPT            "[kybot@kydea ~$] "

enum input_stat {
	WAIT_NORMAL,
	WAIT_SPEC_KEY,
	WAIT_FUNC_KEY,
};
typedef struct {

	enum input_stat stat;

	uint8_t echo_mode:1;

	uint16_t current_history;
	uint16_t history_count;

	char cmd_history[SHELL_HISTORY_LINES][SHELL_CMD_SIZE];

	char line[SHELL_CMD_SIZE];
	uint8_t line_position;
	uint8_t line_curpos;

} shell_t ;

void shell_task(void);

extern shell_t shell;

#endif
