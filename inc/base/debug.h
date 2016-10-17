#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdint.h>


#define UNUSED(x) ((void)(x))


// 调试相关
// usuage:
// here( 1 )
// here( 2 )
#define here(n)         do { here_do(this_file_id, __LINE__, #n, false); } while(0)
#define herek(n)        do { here_do(this_file_id, __LINE__, #n, true); } while(0)

#define dstr(s)         do { dstr_do(this_file_id, __LINE__, #s, (char*)s, false); } while(0)
#define dstrk(s)        do { dstr_do(this_file_id, __LINE__, #s, (char*)s, true ); } while(0)

#define dmem(m, len)    do { dmem_do(this_file_id, __LINE__, #m, (char*)m, len, false); } while(0)
#define dmemk(m, len)   do { dmem_do(this_file_id, __LINE__, #m, (char*)m, len, true); } while(0)

// var
#define var(n)          do { var_do(this_file_id, __LINE__, #n, (long)(n), sizeof(typeof(n)), __func__, false ); } while(0)
#define vark(n)         do { var_do(this_file_id, __LINE__, #n, (long)(n), sizeof(typeof(n)), __func__, true ); } while(0)

void dv_do(int fild_id, int line, const char* message);
void var_do(int file_id, int line, const char* var_name, long n, int size, const char *func, bool wait_key);
void here_do(int file_id, int line, const char* message, bool wait_key);
void dstr_do(int file_id, int line, const char* str_name, const char* str, bool wait_key);
void dmem_do(int file_id, int line, const char* mem_name, const char* mem, int len, bool wait_key);


#endif
