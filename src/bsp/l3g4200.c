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
//����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�

uint8_t gyr_buf_cnt=0;
int16_imu_t gyr_buf[FEEDBACK_BUFFER_NUM];
uint8_t flag_gyr_offset_ok = 0;
int16_imu_t gyr_raw_offset;//��Ư
float_imu_t gyr_vel_offset;//rad/s
int16_imu_t gyr_latest;//����һ�ζ�ȡֵ
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
// ���ݶ�ȡƵ�ʱ����feedback ID_RAW_GYRO����һ��
void l3g4200_read(void)
{
        uint8_t read_buffer[6];//iic��ȡ��������

	uint32_t success = 0;
        // ���λ��1�����ֽڶ�ʱ�Ĵ����Ż��Զ���1
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

        if(!flag_gyr_offset_ok)//У����ƫ
        {
		caculate_raw_offset();
		caculate_vel_offset();
        }
				
        //l3g4200��ǰ����y��,�ұ���x��,������z��
        //ROS�е�xyz��׼����������ǰ��x�ᣬ���ַ�����y�ᣬz��ֱ����
        //����kobuki-indigo/kobuki_node/src/library/slot_callbacks.cpp�з���
        //void KobukiRos::publishRawInertia()ȴ���˱任
        // ��ǰ���Ӳ��ֲ���Ҫ����ת��
        gyr_buf[gyr_buf_cnt].x = gyr_latest.x;
        gyr_buf[gyr_buf_cnt].y = gyr_latest.y;
        gyr_buf[gyr_buf_cnt].z = gyr_latest.z;
        gyr_buf_cnt++;
        if(gyr_buf_cnt == FEEDBACK_BUFFER_NUM)	
                gyr_buf_cnt = 0;	
}

//{{{ init
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void l3g4200_initialize(void)
*��������:	    ��ʼ�� 	l3g4200 �Խ������״̬��
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
