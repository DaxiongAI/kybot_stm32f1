#ifndef __EVENT_H__
#define __EVENT_H__

#include "stm32f10x.h"

#define EVENT_MAX 5

// 返回值意义
// -1: 异常
// 0: 执行成功
// 1: 继续执行
typedef int32_t (*event_fun_t)(void *param);

typedef struct event{
        uint32_t PERIOD;// 隔多久执行一次函数
        uint32_t time;// 记录剩余时间
        uint32_t repeat;// 重复多少次
        void* param;// 执行函数的参数
        event_fun_t fun;// 执行函数
} event_t;

extern event_t event[EVENT_MAX];

void event_task(void);
int32_t event_add(uint32_t period, uint32_t repeat, void *param, event_fun_t fun);
void event_init(void);
#endif
