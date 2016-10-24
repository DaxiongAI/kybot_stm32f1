// See some resources in http://files.yujinrobot.com/
// http://kobuki.yujinrobot.com
// TODO:用keil编译的代码量明显要小于用arm-none-eabi-gcc编译的
// keil编译时勾选了microlib，microlib极度精简，详见http://www.keil.com/arm/microlib.asp
// keil不勾选microlib编译出的代码量也小于arm-none-eabi-gcc编译的，需要跑操作系统的最好不要勾选microlib
// serial problem
//https://github.com/yujinrobot/kobuki/issues/174
//http://answers.ros.org/question/52203/kobuki-malformed-subpayload/
//https://github.com/yujinrobot/kobuki/issues/82
//{{{ include
#include <stdio.h>
#include <string.h>
#include "controller.h"
#include "halfsecond.h"
#include "user_conf.h"
#include "feedback.h"
#include "workdata.h"
#include "protocol.h"
#include "adc.h"
#include "i2c1_hw.h"
#include "imu.h"
#include "encrypt.h"
#include "encoder.h"
#include "motor.h"
#include "buzzer.h"
#include "led.h"
#include "usart.h"
#include "delay.h"
#include "shell.h"
#include "tim.h"
#include "param.h"
#include "stm32f10x.h"
#include "log.h"
#include "event.h"
#include "allan.h"
#include "kalman.h"
#include "file_id.h"
#include "at_port.h"

#define this_file_id file_id_main

//}}}

//{{{ define

/*#define GET_TASK_EXEC_TIME*/

/*#define GET_IMU_TASK_EXEC_TIME*/
/*#define GET_AT_TASK_EXEC_TIME*/
/*#define GET_SHELL_TASK_EXEC_TIME*/
//}}}

//{{{ function declare
void do_os(u8 OSNo);
static void iwdg_init(void);
/*static void reset(void);*/
void nvic_init(void);
void RCC_Configuration(void);
//}}}

//{{{ data_init
void data_init(void)
{
        // 常开
        os[DEFRAME_TASK].Attrib |= OSREQUSTED;
        event_init();
}
//}}}

//{{{ bt 
#ifdef SET_BT_BAUD
static uint8_t set_bt_baudrate(uint8_t which_bt)
{
        int i = 1;
        int success_flag = 0;
        while(i--) {
                if (which_bt) {
                        set_ros_bt_baudrate(115200);
                        delay_ms(1000);
                        if( strstr((char *)ros_recv_buffer.data, "+BAUD=8") != NULL ) {
                                success_flag++;
                                led_set(led1, red, 0);
                        }
                }
                else {
                        set_dbg_bt_baudrate(115200);
                        delay_ms(1000);
                        if( strstr((char *)dbg_recv_buffer.data, "+BAUD=8") != NULL ) {
                                success_flag++;
                                led_set(led2, red, 0);
                        }
                }

                if(success_flag)
                        break;
        }

        return success_flag;
}
#endif
// which_bt: 0 debug bt, 1 ros bt
static uint8_t set_bt_name(uint8_t which_bt, uint8_t *name)
{
        int i = 1;
        int success_flag = 0;
        char tmp[32] = {'\0'};

        /*char _name[7] = {'\0'};*/
        /*int name_len = strlen((const char *)name);*/
        /*if ((name == NULL) || (name_len < 6)) {*/
                /*memcpy(_name, name, name_len);*/
                /*memset(&_name[name_len], ' ', 6-name_len);*/
                /*name = _name;*/
        /*}*/

        if (which_bt) 
                strcat(tmp, "+NAME=ros-");
        else
                strcat(tmp, "+NAME=dbg-");

        strcat(tmp, (const char*)name);

        while(i--) {
                if (which_bt) {
                        set_ros_bt_name(name);
                        delay_ms(1000);
                        if( strstr((char *)ros_recv_buffer.data, tmp) != NULL ) {
                                success_flag++;
                                led_set(led1, red, 0);
                        }

                }
                else {
                        set_dbg_bt_name(name);
                        delay_ms(1000);
                        if( strstr((char *)dbg_recv_buffer.data, tmp) != NULL ) {
                                success_flag++;
                                led_set(led2, red, 0);
                        }
                }

                if(success_flag)
                        break;
        }

        return success_flag;
}
void check_set_bt_name(void)
{
        if (is_need_rename_dbg_bt()) {
                led_set(led1, green, 0);
                uint8_t name[7]; 
                name[6] = '\0';
                read_dbg_bt_name(name);
                if (set_bt_name(0, name)) {
                        clean_rename_dbg_bt_flag();
                }
        }

        if (is_need_rename_ros_bt()) {
                led_set(led2, green, 0);
                uint8_t name[7]; 
                name[6] = '\0';
                read_ros_bt_name(name);
                if (set_bt_name(0, name)) {
                        clean_rename_ros_bt_flag();
                }
        }
}
//}}}

//{{{ main
int main(void)
{	
        u8  OSNo;
        RCC_Configuration();
        system_tick_init();	
        nvic_init();

        read_serial_number0();

#ifndef GET_TASK_EXEC_TIME
	delay_tim_init(DELAY_TIM_PERIOD);// x 1us = 周期
#else
        //for time test
	delay_tim_init(65535);// x 1us = 周期
#endif
        led_init();
        
        read_serial_number1();

        buzzer_init();

        uart1_rx_dma_config();
        uart1_tx_dma_config();

        read_serial_number2();

        uart3_rx_dma_config();
        uart3_tx_dma_config();

        /*************************************************************************/


/////////////////////////////////////////////////////////////////////////////////
#ifdef SET_BT
        uart1_init(9600);	//串口初始化
        uart3_init(9600);
//------------------------------------------------------------------------------
#if defined(SET_BT_NAME)
	set_bt_name(0, "");
	delay_ms(1000);
	set_bt_name(1, "");
	delay_ms(1000);
#elif defined(SET_BT_BAUD)
	set_bt_baudrate(0);
	delay_ms(1000);
	set_bt_baudrate(1);
	delay_ms(1000);
#endif
//------------------------------------------------------------------------------
	while(1);
        /*************************************************************************/
#else
        uart1_init(115200);	//串口初始化
        uart3_init(115200);
#endif
/////////////////////////////////////////////////////////////////////////////////

#ifndef CALCU_ALLAN_VARI
        log_none("\r\n\r\n");
#endif
        log_none("########################################################################################\r\n");
        log_none("#####                                                                              #####\r\n");
        log_none("#####                                                                              #####\r\n");
        log_none("#####                                     kybot                                    #####\r\n");
        log_none("#####                                                                              #####\r\n");
        log_none("#####                               ROS based platform                             #####\r\n");
        log_none("#####                                                                              #####\r\n");
        log_none("#####                             brought to you by Kydea                          #####\r\n");
        log_none("#####                                                                              #####\r\n");
        log_none("#####                                                                              #####\r\n");
        log_none("#####                                www.roswiki.com                               #####\r\n");
        log_none("#####                                                                              #####\r\n");
        log_none("#####                                                                              #####\r\n");
        log_none("########################################################################################\r\n");

        log_info("initialising...");	

        ee_init();
        check_set_bt_name();

        //key_init();
        gen_encryption_key0();

        motor_init();
        encoder_init();

        log_info("checking encryption key...");
        I2C1_Comm_Init(200000, 0x30);// l3g4200d max speed 400khz
        imu_init();
        adc1_init();

        setup_pid();
        data_init();

        gen_encryption_key1();

        param_load();
	iwdg_init();
        /*check_chip_serial_number();*/
        // 读保护作用不大，如果使能，升级固件需要擦除整片flash，密钥就需要用户手动下载到flash里
        /*enable_read_out_protection();*/
	for(int i = 1; i <= 3; i++){
		led_set(led12, (enum led_color)i, 1);
		delay_ms(500);
	}
        log_info("initialised");

        // 关闭所有灯
        led_set(led12, off, 1);
#ifndef SILENCE
	buzzer_toggle(1, 1000);
#endif
	kalman_init(&kalman);
        while(1) {  
                //start_delay_tim(); // 511us
                for (OSNo = 0; OSNo < MAX_TASK_NUM; OSNo++) {
                        if ((os[OSNo].Attrib & OSREQUSTED) || (os[OSNo].Attrib & OSBUSY)) {
                                do_os(OSNo);      
                        }
                } 
                //int cnt = DELAY_TIM->CNT;	
                //stop_delay_tim();
                //log_info("all task time=%d", cnt);			
                /*if(ResetFlag) {   //复位请求*/
                        /*ResetFlag = 0;*/
                        /*reset();*/
                        /*while(1);*/
                /*}*/
        }	   
}
//}}}

//{{{ debug_task
void debug_task(void)
{
        os[DEBUG_TASK].Attrib &= ~OSREQUSTED;

        // load_encoder_value();

        if(log_flag & LOG_BATTERY){
                log_flag &= ~LOG_BATTERY;
                uint8_t voltage = get_battery_vol();
                if(voltage <= KYBOT_BAT_DANGEROUS){
                        log_error("battery is dangerous");
                        buzzer_toggle(200, 50);
                }
                else if(voltage <= KYBOT_BAT_LOW) {
                        log_warn("battery is low");
                        buzzer_toggle(10, 50);
                }

                log_info("battery = %.1fv", voltage/10.0);
        }
        //start_delay_tim();

        //int cnt = DELAY_TIM->CNT;	
        //stop_delay_tim();
        //log_info("log_debug heading=%d", cnt);	
        /*log_debug("offset=%d %d %d",gyr_offset.x, gyr_offset.y, gyr_offset.z);*/
        /*log_debug("pwm: left=%d, right=%d",get_left_pwm(),get_right_pwm());*/
        //log_debug("encoder: left=%d, right=%d", encoder.left, encoder.right);
}
//}}}

//{{{ error_task
// 显示警告、错误提示
// 由于日志输出比较占cpu时间，中断执行时间应该尽量少，不要放置打印函数
void error_task(void)
{
        os[ERROR_TASK].Attrib &= ~OSREQUSTED;

        if(warn_flag & WARN_ROS_BUF_OF) {
                warn_flag &= ~WARN_ROS_BUF_OF;
                log_warn("ros buffer overflow");
                buzzer_toggle(10, 50);
        }

        if(warn_flag & WARN_SHELL_BUF_OF) {
                warn_flag &= ~WARN_SHELL_BUF_OF;
                log_warn("at command buffer overflow");
                buzzer_toggle(10, 50);
        }

        if(warn_flag & ERROR_DRIVER_OV){
                warn_flag &= ~ERROR_DRIVER_OV;
                log_error("the motor may over current or the driver is overheated");
        }
}
//}}}

//{{{ do_os
void do_os(u8 OSNo)
{
        IWDG_ReloadCounter();
        switch (OSNo) 
        {		
        case HALF_SECOND_TASK:{
                half_second_task(); 
                IWDG_ReloadCounter();      
                break;	  
        }
        case DEFRAME_TASK:{       
                deframe_task(); 
                IWDG_ReloadCounter();     
                break;
        }
        case BUZZER_TASK:{			 
                buzzer_task();	
                IWDG_ReloadCounter();					
                break;
        }
        case IMU_TASK:{//432us/450us,+kalman filter 500us	
#if defined(GET_TASK_EXEC_TIME) && defined(GET_IMU_TASK_EXEC_TIME)
		start_delay_tim();
#endif
		imu_analyze_task();
#if defined(GET_TASK_EXEC_TIME) && defined(GET_IMU_TASK_EXEC_TIME)
		int cnt = DELAY_TIM->CNT;
		stop_delay_tim();
		log_info("imu task time=%d", cnt);
#endif
                IWDG_ReloadCounter();
                break;
        }
        case FEEDBACK_TASK:{//23us 24us/63us 64us	
                //start_delay_tim();
                feedback_task();
                //int cnt = DELAY_TIM->CNT;	
                //stop_delay_tim();
                //log_info("feedback=%d", cnt);
                IWDG_ReloadCounter();		
                break; 
        }
        case UPDATE_PID_TASK:{//14us 15us	
                //PIDmode = 1;
                //moving = 1;
                //start_delay_tim();
                update_pid_task(); 
                //int cnt = DELAY_TIM->CNT;	
                //stop_delay_tim();
                //log_info("update_pid=%d", cnt);		
                IWDG_ReloadCounter();	
                break;
        }
        case AT_TASK:{
                // at+xxx=?: 5ms-7ms
                // at: 2ms
                // at+unlock: 3ms
#if defined(GET_TASK_EXEC_TIME) && defined(GET_AT_TASK_EXEC_TIME)
		start_delay_tim();
#endif
                at_process_task();
#if defined(GET_TASK_EXEC_TIME) && defined(GET_AT_TASK_EXEC_TIME)
		int cnt = DELAY_TIM->CNT;
		stop_delay_tim();
		log_info("at task time=%d", cnt);
#endif
                IWDG_ReloadCounter();
                break;
        }
        case SHELL_TASK:{
                // at+xxx=?: 5ms-7ms
                // at: 2ms
                // at+unlock: 3ms
#if defined(GET_TASK_EXEC_TIME) && defined(GET_SHELL_TASK_EXEC_TIME)
		start_delay_tim();
#endif
                shell_task();
#if defined(GET_TASK_EXEC_TIME) && defined(GET_SHELL_TASK_EXEC_TIME)
		int cnt = DELAY_TIM->CNT;
		stop_delay_tim();
		log_info("shell task time=%d", cnt);
#endif
                IWDG_ReloadCounter();
                break;
        }
        case DEBUG_TASK:{//450us
                //start_delay_tim();
                debug_task(); 
                //int cnt = DELAY_TIM->CNT;	
                //stop_delay_tim();
                //log_info("debug task time=%d", cnt);  
                IWDG_ReloadCounter();
                break;
        }      
        case ERROR_TASK:{
                //start_delay_tim();
                error_task();
                //int cnt = DELAY_TIM->CNT;	
                //stop_delay_tim();
                //log_info("error task time=%d", cnt);  
                IWDG_ReloadCounter();
                break;
        }
        case EVENT_TASK:{//log 450us
                //start_delay_tim();
                event_task();
                //int cnt = DELAY_TIM->CNT;	
                //stop_delay_tim();
                //log_info("event task time=%d", cnt);  
                IWDG_ReloadCounter();     
                break;
        }
        default:
                os[OSNo].Attrib = 0;   
                IWDG_ReloadCounter();     
                break;
        }
        IWDG_ReloadCounter();
        return;
} 
//}}}

//{{{ rcc_configuration
/*******************************************************************************
 * 函数名	: RCC_Configuration
 * 函数描述  : 设置系统各部分时钟
 * 输入参数  : 无
 * 输出结果  : 无
 * 返回值    : 无
 *******************************************************************************/
void RCC_Configuration(void)
{
        /* 定义枚举类型变量 HSEStartUpStatus */
        ErrorStatus HSEStartUpStatus;

        /* 复位系统时钟设置*/
        RCC_DeInit();
        /* 开启HSE*/
        RCC_HSEConfig(RCC_HSE_ON);
        /* 等待HSE起振并稳定*/
        HSEStartUpStatus = RCC_WaitForHSEStartUp();
        /* 判断HSE起是否振成功，是则进入if()内部 */
        if(HSEStartUpStatus == SUCCESS)
        {
                /* 选择HCLK（AHB）时钟源为SYSCLK 1分频 */
                RCC_HCLKConfig(RCC_SYSCLK_Div1); 
                /* 选择PCLK2时钟源为 HCLK（AHB） 1分频 */
                RCC_PCLK2Config(RCC_HCLK_Div1); 
                /* 选择PCLK1时钟源为 HCLK（AHB） 2分频 */
                RCC_PCLK1Config(RCC_HCLK_Div2);
                /* 设置FLASH延时周期数为2 */
                FLASH_SetLatency(FLASH_Latency_2);
                /* 使能FLASH预取缓存 */
                FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
                /* 选择锁相环（PLL）时钟源为HSE 1分频，倍频数为9，则PLL输出频率为 8MHz * 9 = 72MHz */
                RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);
                /* 使能PLL */ 
                RCC_PLLCmd(ENABLE);
                /* 等待PLL输出稳定 */
                while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
                /* 选择SYSCLK时钟源为PLL */
                RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
                /* 等待PLL成为SYSCLK时钟源 */
                while(RCC_GetSYSCLKSource() != 0x08);
        }
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);// JTAG-DP Disabled and SW-DP Enable
	// 两个都禁用，挂在jtag的io引脚的led才能控制
	/*GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE); // JTAG-DP and SW-DP Disabled*/
        //TIM1
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
        //TIM2
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//motor
        //TIM3
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
        //TIM4
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//
        //DMA1 for ADC1
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        //ADC1
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
        //GPIOA
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
        //GPIOB
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
        //GPIOC
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
        /* Enable UART clock */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        /* Enable UART clock */
        //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        /* Enable UART clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        //GPIOD
        //GPIOE

}
//}}}

//{{{ nvic_init
void nvic_init(void)
{
        NVIC_InitTypeDef NVIC_InitStructure;

        // NVIC_PriorityGroup
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

        NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn ;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        // usart1 tx, ros comm
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;  
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
        NVIC_Init(&NVIC_InitStructure);

        NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));

        NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        // usart1 rx, ros comm
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        // usart2 tx, esp8266 comm
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;  
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
        NVIC_Init(&NVIC_InitStructure);
        // usart2 rx, esp8266 comm
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        // usart3 tx, debug port
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;  
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
        NVIC_Init(&NVIC_InitStructure);
        // usart3 rx, debug port
        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        // TIM3
        // NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
        // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
        // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        // NVIC_Init(&NVIC_InitStructure);

        // NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;   
        // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; 
        // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      
        // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                
        // NVIC_Init(&NVIC_InitStructure);

        NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;  	
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     
        NVIC_Init(&NVIC_InitStructure);

        // SysTick_IRQn不能用下面的配置
        // NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;  	
        // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
        // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
        // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     
        // NVIC_Init(&NVIC_InitStructure);

}
//}}}

//{{{ watchdog init
/**
 * @brief  WWDG configuration
 * @param  None
 * @retval None
 */
static void iwdg_init(void)
{
        //uint32_t LsiFreq = 40000;
        /* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
           dispersion) */
        /* Enable write access to IWDG_PR and IWDG_RLR registers */
        IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

        /* IWDG counter clock: LSI/32 */
        IWDG_SetPrescaler(IWDG_Prescaler_32);

        /* Set counter reload value to obtain 250ms IWDG TimeOut.
           Counter Reload Value = 250ms/IWDG counter clock period
           = 250ms / (LSI/32)
           = 0.25s / (LsiFreq/32)
           = LsiFreq/(32 * 4)
           = LsiFreq/128
           */
        IWDG_SetReload(0x0fff);//LsiFreq/128

        /* Reload IWDG counter */
        IWDG_ReloadCounter();

        /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
        IWDG_Enable();

}
//}}}

//{{{ reset
#if 0
static void reset(void)
{
        /* Enable WWDG clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

        /* WWDG clock counter = (PCLK1 (48MHz)/4096)/8 = 1464Hz (~683 us)  */
        WWDG_SetPrescaler(WWDG_Prescaler_8);

        /* Set Window value to 80; WWDG counter should be refreshed only when the counter
           is below 80 (and greater than 64) otherwise a reset will be generated */
        WWDG_SetWindowValue(80);

        /* Enable WWDG and set counter value to 127, WWDG timeout = ~683 us * 64 = 43.7 ms 
           In this case the refresh window is: ~683 * (127-80)= 32.1ms < refresh window < ~683 * 64 = 43.7ms
           */
        WWDG_Enable(127);
}
#endif
//}}}

//{{{ assert
#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
        /* User can add his own implementation to report the file name and line number,
ex: log_info("Wrong parameters value: file %s on line %d", file, line) */
        log_info("assert_failed@%s line %d", file, (int)line);
        /* Infinite loop */
        while (1)
        {

        }
}
#endif
//}}}
