//{{{ include
#include <stdlib.h>
#include "user_conf.h"
#include "l3g4200.h"
#include "i2c1_hw.h"
#include "param.h"
#include "delay.h"
#include "workdata.h"
#include "log.h"
#include "file_id.h"
#include "tim.h"
#include "buzzer.h"
#include "event.h"
#include "allan.h"
#include "kalman.h"

#define this_file_id file_id_l3g4200
//}}}

//{{{ define
//定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

uint8_t gyr_buf_cnt=0;
int16_imu_t gyr_buf[FEEDBACK_BUFFER_NUM];
uint8_t flag_gyr_offset_ok = 0;
int16_imu_t gyr_raw_offset;//零漂
float_imu_t gyr_vel_offset;//rad/s
int16_imu_t gyr_latest;//最新一次读取值
/*uint8_t temperature;*/

//}}}

static void caculate_raw_offset(void)
{
	static int32_t temp_raw_z=0;
	static int32_t filter_cnt=0;

	if(filter_cnt==0) {
		gyr_raw_offset.z = 0;
		temp_raw_z = 0;
	}
	temp_raw_z += gyr_latest.z;
	filter_cnt++;

	if(filter_cnt == MAX_FILTER_CNT) {
		gyr_raw_offset.z = temp_raw_z / filter_cnt;
		log_debug("gyr offset: z=%f(no kalman)", gyr_raw_offset.z * 0.00875);
		filter_cnt = 0;
	}
}
static void caculate_vel_offset(void)
{
	static float temp_vel_z=0;
	static int32_t filter_cnt=0;

	double raw_angular_vel = gyr_latest.z * 0.00875;
	double kalman_vel = kalman_filter(&kalman, raw_angular_vel);
	/*log_none("%f,%f\r\n", raw_angular_vel, kalman_vel);*/
	if(filter_cnt==0) {
		gyr_vel_offset.z = 0.0;
		temp_vel_z = 0.0;
		log_info("start calibrate the gyr");
	}
	temp_vel_z += kalman_vel;
	filter_cnt++;

	if(filter_cnt == MAX_FILTER_CNT) {
		gyr_vel_offset.z = temp_vel_z / filter_cnt;
		log_info("gyr calibration is done");
		log_debug("gyr offset: z=%f", gyr_vel_offset.z);
#ifndef SILENCE
		buzzer_toggle(3, 100);
#endif
		/*event_add(1000, ~0, NULL, log_heading_callback);*/
		filter_cnt = 0;
		flag_gyr_offset_ok = 1;
	}
}
//{{{ read
// 数据读取频率必须和feedback ID_RAW_GYRO保持一致
void l3g4200_read(void)
{
        uint8_t read_buffer[6];//iic读取后存放数据

	uint32_t success = 0;
        // 最高位置1，多字节读时寄存器才会自动加1
        success = i2c_read(L3G4200_ADDR, OUT_X_L | 0x80, 6, read_buffer);
	/*success |= i2c_read(L3G4200_ADDR, OUT_TEMP, 1, &temperature);*/
        /*success |= (i2c_read_byte(L3G4200_ADDR, OUT_X_L, &read_buffer[0]);*/
        /*success |= (i2c_read_byte(L3G4200_ADDR, OUT_X_H, &read_buffer[1]);*/
        /*success |= (i2c_read_byte(L3G4200_ADDR, OUT_Y_L, &read_buffer[2]);*/
        /*success |= (i2c_read_byte(L3G4200_ADDR, OUT_Y_H, &read_buffer[3]);*/
        /*success |= (i2c_read_byte(L3G4200_ADDR, OUT_Z_L, &read_buffer[4]);*/
        /*success |= (i2c_read_byte(L3G4200_ADDR, OUT_Z_H, &read_buffer[5]);*/

	if(success)
		log_error("l3g4200 read FAILED !");

#if defined(CALCU_ALLAN_VARI) || defined(CALCU_CALIBRATED_PARAM)
	gyr_latest.x = ((((int16_t)read_buffer[1]) << 8) | read_buffer[0]);
	gyr_latest.y = ((((int16_t)read_buffer[3]) << 8) | read_buffer[2]);
	gyr_latest.z = ((((int16_t)read_buffer[5]) << 8) | read_buffer[4]);
#else
	gyr_latest.x = ((((int16_t)read_buffer[1]) << 8) | read_buffer[0]);
	gyr_latest.y = ((((int16_t)read_buffer[3]) << 8) | read_buffer[2]);
	gyr_latest.z = ((((int16_t)read_buffer[5]) << 8) | read_buffer[4]);

	/*gyr_latest.x = ((((int16_t)read_buffer[1]) << 8) | read_buffer[0]) - gyr_offset.x;*/
	/*gyr_latest.y = ((((int16_t)read_buffer[3]) << 8) | read_buffer[2]) - gyr_offset.y;*/
	/*gyr_latest.z = ((((int16_t)read_buffer[5]) << 8) | read_buffer[4]) - gyr_offset.z;*/
#endif

        if(!flag_gyr_offset_ok)//校正零偏
        {
		caculate_raw_offset();
		caculate_vel_offset();
        }
				
        //l3g4200正前方是y轴,右边是x轴,向上是z轴
        //ROS中的xyz标准方向是脸正前方x轴，左手方向是y轴，z垂直向上
        //但是kobuki-indigo/kobuki_node/src/library/slot_callbacks.cpp中方法
        //void KobukiRos::publishRawInertia()却做了变换
        // 当前板子布局不需要坐标转换
        gyr_buf[gyr_buf_cnt].x = gyr_latest.x;
        gyr_buf[gyr_buf_cnt].y = gyr_latest.y;
        gyr_buf[gyr_buf_cnt].z = gyr_latest.z;
        gyr_buf_cnt++;
        if(gyr_buf_cnt == FEEDBACK_BUFFER_NUM)	
                gyr_buf_cnt = 0;	
}

//{{{ init
/**************************实现函数********************************************
*函数原型:		void l3g4200_initialize(void)
*功　　能:	    初始化 	l3g4200 以进入可用状态。
*******************************************************************************/
void l3g4200_init(void)
{
	uint32_t success = 0;

        success |= i2c_write_byte(L3G4200_ADDR, CTRL_REG2, 0x00);
	success |= i2c_write_byte(L3G4200_ADDR, CTRL_REG3, 0x08);
        success |= i2c_write_byte(L3G4200_ADDR, CTRL_REG4, 0x00);	//+-250dps
        success |= i2c_write_byte(L3G4200_ADDR, CTRL_REG5, 0x00);
	/*success |= i2c_write_byte(L3G4200_ADDR, CTRL_REG1, 0x0f);*/
	success |= i2c_write_byte(L3G4200_ADDR, CTRL_REG1, 0xbf);// 400HZ, cut-off:110
	/*success |= i2c_write_byte(L3G4200_ADDR,CTRL_REG1, 0xff);// 800HZ, cut-off:110*/

	if(success)
		log_error("l3g4200 init FAILED !");
}
//}}}
