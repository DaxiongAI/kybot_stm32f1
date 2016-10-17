#ifndef __USER_CONF_H__
#define __USER_CONF_H__

#define OS_TICK_PER_SECOND        100	 

// 任务类型
// 定义之间不能空行?
enum task_e{
        EVENT_TASK,
        DEFRAME_TASK,
        UPDATE_PID_TASK,
        IMU_TASK,
        FEEDBACK_TASK, 	        
        HALF_SECOND_TASK,
        BUZZER_TASK,
        AT_TASK,
        DEBUG_TASK,
        ERROR_TASK,
        SHELL_TASK,
        MAX_TASK_NUM
};
// 任务当前状态
#define OSREQUSTED              0x80
#define OSBUSY                  0x40

#define PRO_BUFFER_SIZE         256
// 警告、错误类型
// 与ROS通信缓冲区溢出
#define WARN_ROS_BUF_OF         0x0001
// AT命令缓冲区溢出
#define WARN_SHELL_BUF_OF	0x0002
// 电机驱动错误
#define ERROR_DRIVER_OV	        0x0004
// 0x0008
// 0x0010
// 0x0020
// 0x0040
// 0x0080
#define LOG_BATTERY         	0x0001


#define DMA_RX_BUFFER_SIZE      (256)

#define COMM_USE_PORT 1

#if(COMM_USE_PORT==1)
	#define DEBUG_PORT USART3
	#define COMM_PORT USART1
#elif(COMM_USE_PORT==3)
	#define DEBUG_PORT USART1
	#define COMM_PORT USART3
#endif

// 硬件版本
#define HW_VER_MAJOR            2
#define HW_VER_MINOR            2
#define HW_VER_PATCH            2
// 软件版本
#define SW_VER_MAJOR            1
#define SW_VER_MINOR            2
#define SW_VER_PATCH            0

#define	LEFT_ENC_TIM 	        TIM1 
#define RIGHT_ENC_TIM 		TIM2
#define DELAY_TIM	        TIM4
#define MOTOR_PWM_TIM		TIM3

//#define SILENCE
//#define SET_BT
// 两个开其中一个,先开name,再开baud
//#define SET_BT_NAME
//#define SET_BT_BAUD             

#endif



