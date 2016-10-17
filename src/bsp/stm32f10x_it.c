/**
 ******************************************************************************
 * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
 * @author  MCD Application Team
 * @version V3.5.0
 * @date    08-April-2011
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x_it.h"
#include "encoder.h"
#include "tim.h"
#include "motor.h"
#include "workdata.h"
#include "usart.h"
#include "buffer.h"
#include "user_conf.h"
#include "controller.h"
#include "tim.h"
#include "buzzer.h"
#include "adc.h"
#include "feedback.h"
#include "delay.h"
#include "at_port.h"
#include "file_id.h"
#include "i2c1_hw.h"
#define this_file_id file_id_stm32f10x_it

/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/
/**
 * @brief   This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
        /* Go to infinite loop when Memory Manage exception occurs */
        while (1)
        {
        }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
        /* Go to infinite loop when Bus Fault exception occurs */
        while (1)
        {
        }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
        /* Go to infinite loop when Usage Fault exception occurs */
        while (1)
        {
        }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

void DMA1_Channel4_IRQHandler(void)  
{   
        uart1_dma_has_txed = 1;
        DMA_ClearFlag(DMA1_FLAG_TC4);   
        DMA_Cmd(USART1_TX_DMA_CHANNEL, DISABLE);      
}  
void DMA1_Channel7_IRQHandler(void)  
{   
        uart2_dma_has_txed = 1;
        DMA_ClearFlag(DMA1_FLAG_TC7);   
        DMA_Cmd(USART2_TX_DMA_CHANNEL, DISABLE);      
}  
void DMA1_Channel2_IRQHandler(void)  
{   
        uart3_dma_has_txed = 1;
        DMA_ClearFlag(DMA1_FLAG_TC2);   
        DMA_Cmd(USART3_TX_DMA_CHANNEL, DISABLE);      
	if (is_pushing) {
		return;	
	}
	dbg_buffer_send();
}  
/*
   if(USART_GetITStatus(USART1 , USART_IT_RXNE) != RESET)//接收到数据
   {	 
   res=USART_ReceiveData(USART1);
   pTail_inc = (ros_recv_buffer.pTail+1)%PRO_BUFFER_SIZE;
#if(COMM_USE_PORT==1)
U1RxDlyCtr = 1;
if(ros_recv_buffer.pHead != pTail_inc)//如果队列没满
{
ros_recv_buffer.data[ros_recv_buffer.pTail] = res;
ros_recv_buffer.pTail = pTail_inc;
}
#elif(COMM_USE_PORT==3)
U1RxDlyCtr = 5;//debug口需要延时以接收完字符，comm口由于数据50HZ，不需要延时处理，延时有影响数
dbg_coframe_buffer.data[debug_buffer.pTop++] = res;
#endif

USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}
if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
{
// clear interrupt
USART_ClearITPendingBit(USART1, USART_IT_TC);
}
*/
void USART1_IRQHandler(void)
{
        uint8_t pTail_inc;

        uint32_t temp = 0;  
        uint16_t i = 0;  
        if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) {  
                //USART_ClearFlag(USART1,USART_IT_IDLE);  
                temp = USART1->SR;  
                temp = USART1->DR; //清除USART_IT_IDLE标志  

                DMA_Cmd(USART1_RX_DMA_CHANNEL, DISABLE);  

                temp = DMA_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(USART1_RX_DMA_CHANNEL);

                for (i = 0;i < temp; i++) {  
                        pTail_inc = (ros_recv_buffer.pTail + 1) % PRO_BUFFER_SIZE;
                        if(ros_recv_buffer.pHead != pTail_inc) {//如果队列没满
                                ros_recv_buffer.data[ros_recv_buffer.pTail] = uart1_dma_rx_buf[i];
                                ros_recv_buffer.pTail = pTail_inc;
                        }
                        else { 
                                // 与ros通信的缓冲区溢出

                                warn_flag |= WARN_ROS_BUF_OF;

                                break;
                        }
                }  
                DMA_SetCurrDataCounter(USART1_RX_DMA_CHANNEL, DMA_RX_BUFFER_SIZE);  
                DMA_Cmd(USART1_RX_DMA_CHANNEL,ENABLE);  
        }   
}
void USART2_IRQHandler(void)
{
        //uint32_t temp = 0;  
        //uint16_t i = 0;  
        if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {  
                // USART_ClearFlag(USART2, USART_IT_IDLE);  
                //temp = USART2->SR;  
                //temp = USART2->DR; //清除USART_IT_IDLE标志  

                //DMA_Cmd(USART2_RX_DMA_CHANNEL,DISABLE);  

                //temp = DMA_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(USART2_RX_DMA_CHANNEL);  

                /*if(temp > AT_CMD_LEN_MAX) {*/
                /*// AT命令缓冲区溢出*/
                /*warn_flag |= WARN_AT_BUF_OF;*/
                /*}*/
                /*else {*/

                /*for (i = 0;i < temp; i++)  */
                /*{  */
                /*at_cmd_line.data[at_cmd_line.ix] = uart3_dma_rx_buf[i];*/
                /*at_cmd_line.ix = (at_cmd_line.ix + 1)% AT_CMD_LEN_MAX;*/
                /*}  */
                /*at_task_delay = 3;*/
                /*}*/

                DMA_SetCurrDataCounter(USART2_RX_DMA_CHANNEL, DMA_RX_BUFFER_SIZE);  
                DMA_Cmd(USART2_RX_DMA_CHANNEL, ENABLE);  
        }   
}
//注意,读取USARTx->SR能避免莫名其妙的错误   	
//u8 USART_RX_BUF[USART_REC_LEN] __attribute__ ((at(0X20001000)));//接收缓冲,最大USART_REC_LEN个字节,起始地址为0X20001000.    
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
void USART3_IRQHandler(void)
{
        uint8_t pTail_inc;
        uint32_t temp = 0;  
        uint16_t i = 0;  
        if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) {  
                //USART_ClearFlag(USART1,USART_IT_IDLE);  
                temp = USART3->SR;  
                temp = USART3->DR; //清除USART_IT_IDLE标志  

                DMA_Cmd(USART3_RX_DMA_CHANNEL,DISABLE);  

                temp = DMA_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(USART3_RX_DMA_CHANNEL);  
                for (i = 0;i < temp; i++) {  
                        pTail_inc = (dbg_recv_buffer.pTail + 1) % PRO_BUFFER_SIZE;
                        if(dbg_recv_buffer.pHead != pTail_inc) {//如果队列没满 
                                dbg_recv_buffer.data[dbg_recv_buffer.pTail] = uart3_dma_rx_buf[i];
                                dbg_recv_buffer.pTail = pTail_inc;
                        }
                        else {
                                if(temp > AT_CMD_LEN_MAX) {
                                        // AT命令缓冲区溢出
                                        warn_flag |= WARN_SHELL_BUF_OF;
                                }

                                break;
                        }
                }  

                /*if(temp > AT_CMD_LEN_MAX) {*/
                        /*// AT命令缓冲区溢出*/
                        /*warn_flag |= WARN_AT_BUF_OF;*/
                /*}*/
                /*else {*/

                        /*for (i = 0; i < temp; i++)  {  */
                                /*at_cmd_line.data[at_cmd_line.ix] = uart3_dma_rx_buf[i];*/
                                /*at_cmd_line.ix = (at_cmd_line.ix + 1)% AT_CMD_LEN_MAX;*/
                        /*}  */
                        /*[>at_task_delay = 3;<]*/
                /*}*/

                DMA_SetCurrDataCounter(USART3_RX_DMA_CHANNEL, DMA_RX_BUFFER_SIZE);  
                DMA_Cmd(USART3_RX_DMA_CHANNEL, ENABLE);  
        }   
}
void SysTick_Handler(void)
{		
        static u8 pid_time = 0;
        static u8 feedback_time = 0;
        //1s
        static u8 time_1s = 0;
        //5s
        static u16 time_5s;
        //60s
        static u16 time_60s;

        os_time_ticks++;
        pid_time++;
        feedback_time++;
        time_1s++;
        time_5s++;
        time_60s++;
        //必须10ms去读一次数据，详见 kobuki-indigo/kobuki_node/src/library/slot_callbacks.cpp中方法
        //void KobukiRos::publishRawInertia()
        os[IMU_TASK].Attrib |= OSREQUSTED;
        os[EVENT_TASK].Attrib |= OSREQUSTED;
        os[SHELL_TASK].Attrib |= OSREQUSTED;
        // os[DEFRAME_TASK].Attrib |= OSREQUSTED;

        //if(feedback_time == (1000/GYR_READ_FREQ)/(1000/OS_TICK_PER_SECOND)){
        //send_core_sensor_data = 0;//发送gyr raw数据
        os[FEEDBACK_TASK].Attrib |= OSREQUSTED;

        //}
        //else 
        // 50HZ,20ms
        if(feedback_time == (1000/FEEDBACK_FREQ)/(1000/OS_TICK_PER_SECOND)){
                feedback_time = 0;
                send_core_sensor_data = 1;//发送gyr raw数据和core sensor 数据
        }

        if(time_1s == 100){
                time_1s = 0;
                os[DEBUG_TASK].Attrib |= OSREQUSTED;
        }
        if(time_5s == 500){
                time_5s = 0;
                os[ERROR_TASK].Attrib |= OSREQUSTED;
        }
        if(time_60s == 6000) {
                time_60s = 0;
                log_flag |= LOG_BATTERY;
        }
        if(pid_time == (PID_TIME/(1000/OS_TICK_PER_SECOND))){	
                pid_time = 0;
                os[UPDATE_PID_TASK].Attrib |= OSREQUSTED;
        }		
        // 0.5s led
        if(os_time_ticks%(500/(1000/OS_TICK_PER_SECOND))==0){  
                os[HALF_SECOND_TASK].Attrib |= OSREQUSTED;			
        }
        // at命令解析任务
        if(at_task_delay){
                at_task_delay --;    
                if(at_task_delay == 0) { 
                        os[AT_TASK].Attrib |= OSREQUSTED;
                }
        }

        // 蜂鸣器任务
        if(buzzer_toggle_time > 0){
                buzzer_toggle_time--;
                if(buzzer_toggle_time == 0) {
                        os[BUZZER_TASK].Attrib |= OSREQUSTED;
                }
        }
}
void TIM4_IRQHandler(void)		//1ms中断一次
{	
        // if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
        if(TIM4->SR & TIM_IT_Update) {     
                TimingDelay_Decrement();
                TIM4->SR = ~TIM_FLAG_Update;
                //TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);   //清除中断标志
        }
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
void EXTI3_IRQHandler(void)
{

}
void EXTI15_10_IRQHandler(void)
{  
        if(EXTI_GetITStatus(EXTI_Line11) != RESET) {
                if(!GPIO_ReadInputDataBit(MOTOR_FAULT_GPIO, MOTOR_FAULT_PIN)){
                        delay_ms(10);
                        if(!GPIO_ReadInputDataBit(MOTOR_FAULT_GPIO, MOTOR_FAULT_PIN)){
                                warn_flag |= ERROR_DRIVER_OV;
                                buzzer_toggle(10, 100);
                                EXTI_ClearITPendingBit(EXTI_Line11);
                        }
                }
        }
}
void I2C1_EV_IRQHandler( void )
{
        //	i2c1_evt_isr();
        i2cInterruptHandlerI2c1();
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void I2C1_ER_IRQHandler( void )
{
        //	i2c1_err_isr();
        i2cErrorInterruptHandlerI2c1();
}
/**
 * @}
 */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
