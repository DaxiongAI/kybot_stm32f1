//{{{ include
#include "workdata.h"
//}}}

//{{{ declare
queue_buffer_t ros_recv_buffer;
queue_buffer_t dbg_recv_buffer;

/*u8  ResetFlag,KeyFlag,PreKeyFlag;*/
 
u16 at_task_delay;

u32 os_time_ticks; 

os_t os[MAX_TASK_NUM];

u16 warn_flag = 0;

u16 log_flag = 0;
//}}}
