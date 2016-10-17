#include <string.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "log.h"
#include "buffer.h"
#include "debug.h"
#include "file_id.h"

#define wait_any_key()

// because isprint() cause Segmentation fault with stack address, so write mine.
static inline bool qisprint(int c)
{
	// ' ' - '~'
	return (c >= ' ' && c <= '~');
	/*
	   if (c >= ' ' && c <= '/')
	   ¦       return true;
	   if (c >= '0' && c <= '9')
	   ¦       return true;
	   if (c >= ':' && c <= '@')
	   ¦       return true;
	   if (c >= 'A' && c <= 'Z')
	   ¦       return true;
	   if (c >= '[' && c <= '`')
	   ¦       return true;
	   if (c >= 'a' && c <= 'z')
	   ¦       return true;
	   if (c >= '{' && c <= '~')
	   ¦       return true;
	   return false;
	   */
}

void here_do(int file_id, int line, const char *message, bool wait_key)
{
	char const *file_name = file_id_name(file_id);

	char dst[64];
	int len = sprintf(dst, "[%s#%d][here: %s]\r\n", file_name, line, message);
        dbg_send_queue_push((uint8_t *)dst, len);
	if (wait_key) 
		wait_any_key();
}
void var_do(int file_id, int line, const char* var_name, long n, int size, const char *func, bool wait_key)
{
	const char *file_name = file_id_name(file_id);

	char dst[128];
	// 数值放在前面，是因为有些名字很长
	char ch = qisprint(n) ? (char)n : '.';
	int len = 0;
	if (size <= sizeof(int))
		len = sprintf(dst, "[%s#%d@%s()][var: %s = %d(D), 0x%x][char=%c][size=%d]\r\n", 
				file_name, line, func, var_name, (int)n, (unsigned int)n, ch, size);
	else if (size <= sizeof(long))
		len = sprintf(dst, "[%s#%d@%s()][var: %s = %ld(D), 0x%lx][char=%c][size=%d]\r\n", 
				file_name, line, func, var_name, n, (unsigned long)n, ch, size);
	else
		len = sprintf(dst, "[%s#%d@%s()][var: %s = %ld(D), 0x%lx][char=%c][size=%d]\r\n", 
				file_name, line, func, var_name, n, (unsigned long)n, ch, size);

        dbg_send_queue_push((uint8_t *)dst, len);

	if(wait_key)
		wait_any_key();
}

void dstr_do(int file_id, int line, const char *str_name, const char *str, bool wait_key) 
{
	char dst[512];
	const char *s = "[NULL]";
	if (NULL == str)
		str = s;

	const char *file_name = file_id_name(file_id);

	int len = strlen(str);

	len = sprintf(dst, "[%s#%d][dstr:%s(len=%d)]\r\n", file_name, line, str_name, len);

	// 字符串格式
	{
		// str可能不是'\0'结束，所以转存到堆栈内存上
		len += sprintf(&dst[len], "[str=%s]", str);
	}

	// 16进制格式
	{
		len += sprintf(&dst[len], "[hex=");
		int i;
		for (i = 0; i < len; i++)
			len += sprintf(&dst[len], " %02x", (unsigned char)str[i]);
	}
	len += sprintf(&dst[len], " ]\r\n");

        dbg_send_queue_push((uint8_t *)dst, len);

	if( wait_key ){
		wait_any_key();
	}
}

void dmem_do(int file_id, int line, const char* mem_name, const char* mem, int len, bool wait_key)
{
	const char* file_name = file_id_name(file_id);

	char dst[512];
	len = sprintf("[%s#%d][mem:%s(len=%d)]\r\n", file_name, line, mem_name, len);

	// 字符串格式
	{
		// str可能不是'\0'结束，所以一个个字符打印
		len += sprintf(&dst[len], "\t[dot_style=");
		const char *p = mem;
		int i = 0;
		for (; i < len; i++, p++ ) {
			if (qisprint(*p))
				len += sprintf(&dst[len], "%c", *p);
			else
				len += sprintf(&dst[len], "%c", '.');
		}
		len += sprintf(&dst[len], "]\r\n");

	}
	// c style
	{
		len += sprintf(&dst[len], "\t[c_style=");
		const char *p = mem;
		int i = 0;
		for (; i < len; i++, p++) {
			unsigned char uc = *p;
			if (qisprint(*p))
				len += sprintf(&dst[len], "%c", uc);
			else
				len += sprintf(&dst[len], "\\x%02x", uc);

		}
		len += sprintf(&dst[len], " ]\r\n");
	}

	// 16进制格式
	{
		len += sprintf(&dst[len], "\t[hex_style=");
		const char *p = mem;
		int i = 0;
		for (; i < len; i++, p++) {
			unsigned char uc = *p;
			len += sprintf(&dst[len], " %02x", uc);
		}
		len += sprintf(&dst[len], " ]\r\n");
	}

        dbg_send_queue_push((uint8_t *)dst, len);

	if (wait_key) {
		wait_any_key();
	}
}

