#ifndef __AT_H__
#define __AT_H__

//{{{ include
#include <stdio.h>
#include "stm32f10x.h"
#include "log.h"
#include "at_utils.h"
//}}}

//{{{ define
// 减少代码量
// #define at_echo_ok()      log_at("OK\r\n")
// #define at_echo_error()	log_at("ERROR\r\n")
// #define at_echo_no_fun()  log_at("This fun is NOT supported, please check the input\r\n")


#define at_echo_ok()            echo_ok(this_file_id, __func__, __LINE__)
#define at_echo_error()	        echo_error(this_file_id, __func__, __LINE__)
#define at_echo_no_fun()        echo_no_fun(this_file_id, __func__, __LINE__)

#define at_echo(format, ...) 	log_at(format, ##__VA_ARGS__)

#define at_number_valid(p)       number_valid(this_file_id, __func__, __LINE__, p)
//}}}

// typedef enum{
        // at_statIdle,
        // at_statRecving,
        // at_statProcess,
        // at_statIpSending,
        // at_statIpSended,
        // at_statIpTraning
// }at_state_t;

// typedef enum{
        // m_init,
        // m_wact,
        // m_gotip,
        // m_linked,
        // m_unlink,
        // m_wdact
// }at_mdState_t;

typedef struct
{
        char *cmd_name;
        int8_t cmd_len;
        void (*test_cmd)(uint8_t id);
        void (*query_cmd)(uint8_t id);
        void (*set_cmd)(uint8_t id, uint8_t *pPara);
        void (*exec_cmd)(uint8_t id);
}at_function_t;

void at_init(void);
void at_cmd_process(uint8_t *pAtRcvData);

#endif
