#ifndef  __WORKDATA_H__
#define  __WORKDATA_H__

#include "stm32f10x.h"
#include "user_conf.h"

// from the ros server
#define ID_BASE_CONTROL         1
#define ID_SOUND_SEQ            4
#define ID_REQUEST_EXTRA        9
#define ID_GENERAL_OUTPUT       12
#define ID_SET_PID_GAIN         13
#define ID_GET_PID_GAIN         14
// to the ros server
#define ID_CONTROL_INFO         21
#define ID_HW_VERSION           10
#define ID_FW_VERSION           11
#define ID_UDID 	        19
#define ID_BASIC_SENSOR         1
#define ID_DOCKING_IR           3
#define ID_INERTIAL_SENSOR      4
#define ID_CLIFF                5
#define ID_CURRENT_WHEEL        6
#define ID_RAW_GYRO             13
#define ID_GEN_INPUT            16
typedef struct {
  u8 Attrib;
  u8 Step;
  u8 Mode;
  u8 Timer;
} os_t;

extern os_t os[MAX_TASK_NUM];

typedef struct {
  uint16_t pHead;
  uint16_t pTail;
  uint8_t data[PRO_BUFFER_SIZE];//循环队列缓冲区
} queue_buffer_t;
//时间
//typedef struct struct_rtc 
//{
    //uint8_t second;
    //uint8_t minute;
    //uint8_t hour;
    //uint8_t day; 
    //uint8_t week;
    //uint8_t month;
    //uint8_t year;
    //uint8_t century;
//}rtc_t;

//extern rtc_t rtc;

extern queue_buffer_t ros_recv_buffer;
extern queue_buffer_t dbg_recv_buffer;

//extern uint8_t ResetFlag,KeyFlag,PreKeyFlag;

extern u16 at_task_delay;
extern u32 os_time_ticks; 
extern u16 warn_flag;
extern u16 log_flag;

#endif 	

