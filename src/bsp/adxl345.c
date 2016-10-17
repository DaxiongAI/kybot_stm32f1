//{{{ include
#include <stdlib.h>
#include "adxl345.h"
#include "i2c1_hw.h"
#include "param.h"
#include "delay.h"
#include "workdata.h"
#include "log.h"
#include "file_id.h"
#include "tim.h"
#include "buzzer.h"
#include "event.h"

#define this_file_id file_id_adxl345
//}}}

//{{{ define

uint8_t flag_acc_offset_ok = 0;
int16_imu_t acc_offset;//��Ư
int16_imu_t acc_latest;//����һ�ζ�ȡֵ
//}}}

//{{{ read
// ���ݶ�ȡƵ�ʱ����feedback ID_RAW_accO����һ��
void adxl345_read(void)
{
        uint8_t read_buffer[6];//iic��ȡ��������
	uint32_t success = 0;
        success = i2c_read(ADXL345_ADDR, DATA_X0, 6, read_buffer);
        /*success |= (i2c_read_byte(ADXL345_ADDR, DATA_X0, &read_buffer[0]);*/
        /*success |= (i2c_read_byte(ADXL345_ADDR, DATA_X1, &read_buffer[1]);*/
        /*success |= (i2c_read_byte(ADXL345_ADDR, DATA_Y0, &read_buffer[2]);*/
        /*success |= (i2c_read_byte(ADXL345_ADDR, DATA_Y1, &read_buffer[3]);*/
        /*success |= (i2c_read_byte(ADXL345_ADDR, DATA_Z0, &read_buffer[4]);*/
        /*success |= (i2c_read_byte(ADXL345_ADDR, DATA_Z1, &read_buffer[5]);*/

	if(success)
		log_error("adxl345 read FAILED !");

#if defined(CALCU_ALLAN_VARI) || defined(CALCU_CALIBRATED_PARAM)
	acc_latest.x = ((((int16_t)read_buffer[1]) << 8) | read_buffer[0]);
	acc_latest.y = ((((int16_t)read_buffer[3]) << 8) | read_buffer[2]);
	acc_latest.z = ((((int16_t)read_buffer[5]) << 8) | read_buffer[4]);
#else
	acc_latest.x = ((((int16_t)read_buffer[1]) << 8) | read_buffer[0]) - acc_offset.x;
	acc_latest.y = ((((int16_t)read_buffer[3]) << 8) | read_buffer[2]) - acc_offset.y;
	acc_latest.z = ((((int16_t)read_buffer[5]) << 8) | read_buffer[4]) - acc_offset.z;
#endif

        if(!flag_acc_offset_ok)//У����ƫ
        {
                static int32_t temp_acc_x=0, temp_acc_y=0, temp_acc_z=0;
                static int32_t filter_cnt=0;
                if(filter_cnt==0) {
                        acc_offset.x = 0;
                        acc_offset.y = 0;
                        acc_offset.z = 0;
                        temp_acc_x = 0;
                        temp_acc_y = 0;
                        temp_acc_z = 0;
                        log_info("start calibrate the acc");
                }
                temp_acc_x += acc_latest.x;
                temp_acc_y += acc_latest.y;
                temp_acc_z += acc_latest.z;
                filter_cnt++;
                if(filter_cnt == MAX_FILTER_CNT) {
                        acc_offset.x = temp_acc_x/filter_cnt;
                        acc_offset.y = temp_acc_y/filter_cnt;
                        acc_offset.z = temp_acc_z/filter_cnt;
                        log_info("acc calibration is done");
                        log_debug("acc offset: x=%d y=%d z=%d", acc_offset.x, acc_offset.y, acc_offset.z);
#ifdef CALCU_ALLAN_VARI
                        log_none("acc offset: x=%d y=%d z=%d\r\n", acc_offset.x, acc_offset.y, acc_offset.z);
#endif
                        buzzer_toggle(3, 100);
                        /*event_add(1000, 10, NULL, log_heading_callback);*/
                        filter_cnt = 0;
                        flag_acc_offset_ok = 1;
                }
                return;
        }
				
        //adxl345��ǰ����x��,�����y��,������z��
        //ROS�е�xyz��׼����������ǰ��x�ᣬ���ַ�����y�ᣬz��ֱ����
}

//{{{ init
void adxl345_init(void)
{
	uint32_t success = 0;

        success |= i2c_write_byte(ADXL345_ADDR, DATA_FORMAT, 0x2B);   //������Χ,����16g��13λģʽ
        success |= i2c_write_byte(ADXL345_ADDR, BW_RATE, 0x0A);   //100HZ, �ο�pdf13ҳ
        success |= i2c_write_byte(ADXL345_ADDR, POWER_CTL, 0x28);   //ѡ���Դģʽ   �ο�pdf24ҳ
        success |= i2c_write_byte(ADXL345_ADDR, INT_ENABLE, 0x00);   //��ʹ���ж�
        success |= i2c_write_byte(ADXL345_ADDR, OFSX, 0x00);   //X ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ
        success |= i2c_write_byte(ADXL345_ADDR, OFSY, 0x00);   //Y ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ
        success |= i2c_write_byte(ADXL345_ADDR, OFSZ, 0x00);   //Z ƫ���� ���ݲ��Դ�������״̬д��pdf29ҳ

	if(success)
		log_error("adxl345 init FAILED !");
}
//}}}
