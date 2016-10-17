//{{{ include
#include <stdlib.h>
#include <math.h>
#include "imu.h"
#include "i2c1_hw.h"
#include "param.h"
#include "delay.h"
#include "workdata.h"
#include "log.h"
#include "file_id.h"
#include "tim.h"
#include "event.h"
#include "buzzer.h"
#include "controller.h"
#include "motor.h"
#include "l3g4200.h"
#include "adxl345.h"
#include "allan.h"
#include "kalman.h"
#include "encoder.h"
#define this_file_id file_id_imu

//}}}

//{{{ define
#define i2c_write   i2cWrite
#define i2c_write_byte   i2cWriteByte
#define i2c_read    i2cRead
#define i2c_read_byte   i2cReadByte

/*#define ACC_AVG_NUM  16*/
// 两电机pwm持续1s为0，就判断为静止状态
#define MOVING_MAX_FILT_CNT  100
/*#define ERR_FACTOR  0.001*/

// heading值=实际值计算的角度值(float)x100
double heading = 0.0;
double fheading = 0.0;
double heading_factor = 1.0;
/*uint8_t enable_imu_when_motor_static = 0;*/
//}}}

//{{{ imu_task
// 数据读取频率必须和feedback ID_RAW_GYRO保持一致
// read time: 427us
// TODO：feedback任务要在读imu任务之后
void imu_analyze_task(void)
{
	os[IMU_TASK].Attrib &= ~OSREQUSTED;
	static uint16_t calibration_delay_cnt = STATIC_TIME * 100;// 静止3s再开始校正
	static uint16_t moving_filt_cnt = 0;
	static uint8_t is_moving = 0;

	if(calibration_delay_cnt > 0){
		calibration_delay_cnt--;
		return;
	}
	//start_delay_tim();
	/*mpu6050_read();*/
	/*hmc5883_read();*/
	l3g4200_read();
	//adxl345_read();

#ifndef IMU_NORMAL_MODE
	output_acc_gyr(&acc_latest, &gyr_latest);
#endif
	//int cnt = DELAY_TIM->CNT;	
	//stop_delay_tim();
	//log_info("imu read time=%dus", cnt);
	if (flag_gyr_offset_ok) {
		if((motor.l_pwm == 0) && (motor.r_pwm == 0)) {
			if (moving_filt_cnt == MOVING_MAX_FILT_CNT)
				is_moving = 0;
			else
				moving_filt_cnt++;
		}
		else {
			moving_filt_cnt = 0;
			is_moving = 1;
		}

		double raw_angular_z_vel = gyr_latest.z * 0.00875;
		double delta_angle = (gyr_latest.z - gyr_raw_offset.z) * 0.00875 / GYR_READ_FREQ;
		if (is_moving) 
		{
			/*heading += FACTOR / GYR_READ_FREQ * gyr_latest.z * GYR_SENSITIVITY;//单位:度*/
			// GYR_READ_FREQ = 100,简化计算
			/*heading += gyr_latest.z * 875 / (100000*GYR_READ_FREQ/FACTOR);//单位:度*/
			/*heading %= FACTOR * 360;*/
			if (left_speed != right_speed) 
			{
				double kalman_z_vel_no_offset = kalman_filter(&kalman, raw_angular_z_vel) - gyr_vel_offset.z;
				fheading += kalman_z_vel_no_offset / GYR_READ_FREQ;
				fheading *= heading_factor;
				fheading = fmod(fheading, 360.0);
			}
		}
		heading += delta_angle;
		heading *= heading_factor;
		heading = fmod(heading, 360.0);
	}
}
int32_t log_heading_callback(void *param)
{
        /*log_debug("heading = %f", (float)heading/FACTOR);*/
        /*log_debug("heading = %f", fheading);*/
        log_debug("heading = %f, kalman fheading = %f", heading, fheading);
        return 1;
}
//}}}

//{{{ imu_init
/**************************实现函数********************************************
 *函数原型:		void l3g4200_initialize(void)
 *功　　能:	    初始化 	l3g4200 以进入可用状态。
 *******************************************************************************/
void imu_init(void)
{
        /*mpu6050_init();*/
        /*hmc5883_init();*/
        l3g4200_init();
        //adxl345_init();
}
//}}}

#if 0
//{{{ imu_update
// 快速算“平方根的倒数”。
// http://zh.wikipedia.org/wiki/%E5%B9%B3%E6%96%B9%E6%A0%B9%E5%80%92%E6%95%B0%E9%80%9F%E7%AE%97%E6%B3%95
float reciprocal_of_sqrt(float number)
{
        long i;
        float x2, y;
        const float threehalfs = 1.5F;

        x2 = number * 0.5F;
        y  = number;
        i  = * ( long * ) &y;                       //对浮点数的邪恶位级hack
        i  = 0x5f3759df - ( i >> 1 );              
        y  = * ( float * ) &i;
        y  = y * ( threehalfs - ( x2 * y * y ) );   //第一次牛顿迭代
        y  = y * ( threehalfs - ( x2 * y * y ) );   //第二次迭代，可以删除

        return y;
}
int16_imu_t acc_history[ACC_AVG_NUM];
void imu_update_with_acc_gyr(const int16_imu_t *filted_acc, const int16_imu_t *filted_gyr)
{
        float dt= 1.0 / GYR_READ_FREQ;
        float delta_2 = 0.0;
        float_imu_t angle_vel;
        float_imu_t acc_norm;
        float_imu_t acc;
        float_imu_t acc_sum;
        static int32_t acc_ix = 0;

        acc_history[acc_ix].x = filted_acc->x * 9.7883 / 4096.0;// 换算成m/s^2, 8g量程
        acc_history[acc_ix].y = filted_acc->y * 9.7883 / 4096.0;
        acc_history[acc_ix].z = filted_acc->z * 9.7883 / 4096.0;
        acc_ix ++;
        if(acc_ix >= ACC_AVG_NUM)
                acc_ix = 0;

        for(int i = 0; i < ACC_AVG_NUM; i++){
                acc_sum.x += acc_history[i].x;
                acc_sum.y += acc_history[i].y;
                acc_sum.z += acc_history[i].z;
        }

        acc.x = acc_sum.x / ACC_AVG_NUM;
        acc.y = acc_sum.y / ACC_AVG_NUM;
        acc.z = acc_sum.z / ACC_AVG_NUM;

        angle_vel.x = (float)filted_gyr->x * 0.00875 / 180 * 3.1415926;// 换算成rad/s
        angle_vel.y = (float)filted_gyr->y * 0.00875 / 180 * 3.1415926;// 换算成rad/s
        angle_vel.z = (float)filted_gyr->z * 0.00875 / 180 * 3.1415926;// 换算成rad/s

        /*if(dt > 1)*/
        /*return;*/

        float norm = 0.0f;
        float_imu_t gav_est;
        float_imu_t err;

        float ww = attitude.w * attitude.w;
        float wx = attitude.w * attitude.x;
        float wy = attitude.w * attitude.y;
        //float wz = w*z;
        float xx = attitude.x * attitude.x;
        //float xy = x*y;
        float xz = attitude.x * attitude.z;
        float yy = attitude.y * attitude.y;
        float yz = attitude.y * attitude.z;
        float zz = attitude.z * attitude.z;

        norm = reciprocal_of_sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z); // 测量正常化 把加速度计的三维向量转成单位向量。
        acc_norm.x = acc.x * norm;
        acc_norm.y = acc.y * norm;
        acc_norm.z = acc.z * norm;

        //这是把四元数换算成"方向余弦矩阵"中的第三列的三个元素。
        //根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
        //所以这里的gav_est_x\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
        gav_est.x = 2 * (xz - wy);
        gav_est.y = 2 * (wx + yz);
        gav_est.z = ww - xx - yy + zz;

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        err.x = acc_norm.y * gav_est.z - acc_norm.z * gav_est.y;                         					
        err.y = acc_norm.z * gav_est.x - acc_norm.x * gav_est.z; 
        err.z = acc_norm.x * gav_est.y - acc_norm.y * gav_est.x; 
        //acc_norm x,y,z是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
        //acc_norm x,y,z是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
        //那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
        //向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，err x,y,z就是两个重力向量的叉积。
        //这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，
        //而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，
        //所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。

        /*  
            exInt = exInt + ex * Ki;//计算和应用积分反馈			 
            eyInt = eyInt + ey * Ki;
            ezInt = ezInt + ez * Ki;
            gx = gx + Kp*ex + exInt;//校正陀螺仪测量值	   用叉积误差来做PI修正陀螺零偏							
            gy = gy + Kp*ey + eyInt;
            gz = gz + Kp*ez + ezInt;	
            */ 

        /*gyr[0] = gyr[0] + err[0] * ERR_FACTOR / dt / 2.0;//校正陀螺仪测量值，用叉积误差来做PI修正陀螺零偏							*/
        /*gyr[1] = gyr[1] + err[1] * ERR_FACTOR / dt / 2.0; */
        /*gyr[2] = gyr[2] + err[2] * ERR_FACTOR / dt / 2.0;	 */

        float delta_x = angle_vel.x * dt + err.x * ERR_FACTOR;
        float delta_y = angle_vel.y * dt + err.y * ERR_FACTOR;
        float delta_z = angle_vel.z * dt + err.z * ERR_FACTOR;

        delta_2 = delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
        /*(dt * gyr[0]) * (dt * gyr[0]) + (dt * gyr[1]) * (dt * gyr[1]) + (dt * gyr[2]) * (dt * gyr[2]);*/
        /* 
           delta_4=delta_2*delta_2;
        // 整合四元数率，解四元数微分方程（四阶毕卡法）
        w = (1-delta_2/8+delta_4/384)*w + (-x*gx - y*gy - z*gz)*(0.5-delta_2/48);		
        x = (1-delta_2/8+delta_4/384)*x + ( w*gx + y*gz - z*gy)*(0.5-delta_2/48);
        y = (1-delta_2/8+delta_4/384)*y + ( w*gy - x*gz + z*gx)*(0.5-delta_2/48);
        z = (1-delta_2/8+delta_4/384)*z + ( w*gz + x*gy - y*gx)*(0.5-delta_2/48);	
        */		 
        // 参考秦永元的惯性导航P302
        // 整合四元数率，解四元数微分方程（二阶毕卡法）
        attitude.w = (1-delta_2/8) * attitude.w + (-attitude.x * delta_x - attitude.y * delta_y - attitude.z * delta_z)/2.0;	
        attitude.x = (1-delta_2/8) * attitude.x + ( attitude.w * delta_x + attitude.y * delta_z - attitude.z * delta_y)/2.0;
        attitude.y = (1-delta_2/8) * attitude.y + ( attitude.w * delta_y - attitude.x * delta_z + attitude.z * delta_x)/2.0;
        attitude.z = (1-delta_2/8) * attitude.z + ( attitude.w * delta_z + attitude.x * delta_y - attitude.y * delta_x)/2.0;

        // 整合四元数率，解四元数微分方程(一阶龙库法,与一阶毕卡法等价)
        //attitude.w = attitude.w + (-attitude.x * delta_x - attitude.y * delta_y - attitude.z * delta_z)/2.0;
        //attitude.x = attitude.x + ( attitude.w * delta_x + attitude.y * delta_z - attitude.z * delta_y)/2.0;
        //attitude.y = attitude.y + ( attitude.w * delta_y - attitude.x * delta_z + attitude.z * delta_x)/2.0;
        //attitude.z = attitude.z + ( attitude.w * delta_z + attitude.x * delta_y - attitude.y * delta_x)/2.0;

        // 正常化四元
        norm = reciprocal_of_sqrt(attitude.w * attitude.w + attitude.x * attitude.x + attitude.y * attitude.y + attitude.z * attitude.z);
        attitude.w *= norm;
        attitude.x *= norm;
        attitude.y *= norm;
        attitude.z *= norm;
        // -2*q2*q2 - 2*q3*q3 + 1
        float t11 = -2 * attitude.y * attitude.y - 2 * attitude.z * attitude.z + 1;
        // 2*q1*q2 + 2*q0*q3
        float t12 =  2 * attitude.x * attitude.y + 2 * attitude.w * attitude.z;
        // -2*q1*q3 + 2*q0*q2
        float t13 = -2 * attitude.x * attitude.z + 2 * attitude.w * attitude.y;
        // 2*q2*q3 + 2*q0*q1
        float t23 =  2 * attitude.y * attitude.z + 2 * attitude.w * attitude.x;
        // -2*q1*q1 -2*q2*q2 + 1
        float t33 = -2 * attitude.x * attitude.x - 2 * attitude.y * attitude.y + 1;
        //转换为欧拉角
        //偏航角
        double yaw    =  atan2(t12, t11) * 57.3; // yaw
        //俯仰角
        double pitch  = asin(t13) * 57.3; 	 // pitch
        //滚转角
        double roll   =  -atan2(t23, t33) * 57.3; // roll
        /*double roll   =  atan2(t23,t33) * 57.3; // roll*/
        /*if(roll > 90 || roll < -90) {*/
        /*if(pitch > 0)*/
        /*pitch = 180 - pitch;*/
        /*if(pitch < 0)*/
        /*pitch = -(180 + pitch);*/
        /*}*/

        static int flag_debug_cnt = 0;
        flag_debug_cnt++;
        if (flag_debug_cnt == 100) {
                flag_debug_cnt = 0;
                log_debug("heading=%f, roll=%f, pitch=%f, yaw=%f", (float)heading/FACTOR, roll, pitch, yaw);
        }
}
const static float MIX_MAG_Y = 0.743144f;/*cos(42)*/
const static float MIX_MAG_Z = -0.669130f;/*sin42*/
void imu_update_with_acc_gyr_mag(const int16_imu_t *filted_acc, const int16_imu_t *filted_gyr, const int16_imu_t *filted_mag)
{
        float dt= 1.0 / GYR_READ_FREQ;
        float acc[3], gyr[3], mag[3];
        float norm = 0.0f;
        static int32_t acc_ix = 0;
        float_imu_t acc_sum;

        acc_history[acc_ix].x = filted_acc->x * 9.7883 / 4096.0;// 换算成m/s^2, 8g量程
        acc_history[acc_ix].y = filted_acc->y * 9.7883 / 4096.0;
        acc_history[acc_ix].z = filted_acc->z * 9.7883 / 4096.0;
        acc_ix ++;
        if(acc_ix >= ACC_AVG_NUM)
                acc_ix = 0;

        for(int i = 0; i < ACC_AVG_NUM; i++){
                acc_sum.x += acc_history[i].x;
                acc_sum.y += acc_history[i].y;
                acc_sum.z += acc_history[i].z;
        }

        acc[0] = acc_sum.x / ACC_AVG_NUM;
        acc[1] = acc_sum.y / ACC_AVG_NUM;
        acc[2] = acc_sum.z / ACC_AVG_NUM;

        gyr[0] = (float)filted_gyr->x * 0.00875 / 180 * 3.1415926;// 换算成rad/s
        gyr[1] = (float)filted_gyr->y * 0.00875 / 180 * 3.1415926;// 换算成rad/s
        gyr[2] = (float)filted_gyr->z * 0.00875 / 180 * 3.1415926;// 换算成rad/s

        // 绕Z旋转-45°（安装问题）。
        mag[0] = 0.7071067/* 根号2除以2 */ * ( filted_mag->x + filted_mag->y);
        mag[1] = 0.7071067/* 根号2除以2 */ * (-filted_mag->x + filted_mag->y);
        mag[2] = filted_mag->z;

        float w_q = attitude.w;
        float x_q = attitude.x;
        float y_q = attitude.y;
        float z_q = attitude.z;
        float x_q_2 = x_q * 2;
        float y_q_2 = y_q * 2;
        float z_q_2 = z_q * 2;

        //
        // 单位化加速度计的读数。
        float a_rsqrt = reciprocal_of_sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
        float x_aa = acc[0] * a_rsqrt;
        float y_aa = acc[1] * a_rsqrt;
        float z_aa = acc[2] * a_rsqrt;
        //
        // 单位化罗盘的读数。
        float h_rsqrt = reciprocal_of_sqrt(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]);
        float x_hh = mag[0] * h_rsqrt;
        float y_hh = mag[1] * h_rsqrt;
        float z_hh = mag[2] * h_rsqrt;
        //
        // 载体坐标下的重力加速度常量，已单位化。
        float x_ac = x_q*z_q_2 - w_q*y_q_2;
        float y_ac = y_q*z_q_2 + w_q*x_q_2;
        float z_ac = 1 - x_q*x_q_2 - y_q*y_q_2;
        //
        // 载体坐标下的地磁场常量，已单位化。
        float x_hc = MIX_MAG_Y*(x_q*y_q_2 + w_q*z_q_2)     + MIX_MAG_Z*(x_q*z_q_2 - w_q*y_q_2)    ;
        float y_hc = MIX_MAG_Y*(1 - x_q*x_q_2 - z_q*z_q_2) + MIX_MAG_Z*(y_q*z_q_2 + w_q*x_q_2)    ;
        float z_hc = MIX_MAG_Y*(y_q*z_q_2 - w_q*x_q_2)     + MIX_MAG_Z*(1 - x_q*x_q_2 - y_q*y_q_2);
        //
        // 测量值与常量的叉积。
        float x_ca = y_aa * z_ac - z_aa * y_ac;
        float y_ca = z_aa * x_ac - x_aa * z_ac;
        float z_ca = x_aa * y_ac - y_aa * x_ac;
        float x_ch = y_hh * z_hc - z_hh * y_hc;
        float y_ch = z_hh * x_hc - x_hh * z_hc;
        float z_ch = x_hh * y_hc - y_hh * x_hc;
        //
        // 构造增量旋转。
        float delta_x = gyr[0] * dt / 2 + (x_ca + x_ch) * ERR_FACTOR;
        float delta_y = gyr[1] * dt / 2 + (y_ca + y_ch) * ERR_FACTOR;
        float delta_z = gyr[2] * dt / 2 + (z_ca + z_ch) * ERR_FACTOR;
        //
        // 融合，四元数乘法。
        attitude.w = w_q         - x_q*delta_x - y_q*delta_y - z_q*delta_z;
        attitude.x = w_q*delta_x + x_q         + y_q*delta_z - z_q*delta_y;
        attitude.y = w_q*delta_y - x_q*delta_z + y_q         + z_q*delta_x;
        attitude.z = w_q*delta_z + x_q*delta_y - y_q*delta_x + z_q;
        // 正常化四元
        norm = reciprocal_of_sqrt(attitude.w * attitude.w + attitude.x * attitude.x + attitude.y * attitude.y + attitude.z * attitude.z);
        attitude.w *= norm;
        attitude.x *= norm;
        attitude.y *= norm;
        attitude.z *= norm;

        // -2*q2*q2 - 2*q3*q3 + 1
        float t11 = -2 * attitude.y * attitude.y - 2 * attitude.z * attitude.z + 1;
        // 2*q1*q2 + 2*q0*q3
        float t12 =  2 * attitude.x * attitude.y + 2 * attitude.w * attitude.z;
        // -2*q1*q3 + 2*q0*q2
        float t13 = -2 * attitude.x * attitude.z + 2 * attitude.w * attitude.y;
        // 2*q2*q3 + 2*q0*q1
        float t23 =  2 * attitude.y * attitude.z + 2 * attitude.w * attitude.x;
        // -2*q1*q1 -2*q2*q2 + 1
        float t33 = -2 * attitude.x * attitude.x - 2 * attitude.y * attitude.y + 1;
        //转换为欧拉角
        //偏航角
        double yaw    =  atan2(t12, t11) * 57.3; // yaw
        //俯仰角
        double pitch  = asin(t13) * 57.3; 	 // pitch
        //滚转角
        double roll   =  -atan2(t23, t33) * 57.3; // roll
        /*double roll   =  atan2(t23,t33) * 57.3; // roll*/
        /*if(roll > 90 || roll < -90) {*/
        /*if(pitch > 0)*/
        /*pitch = 180 - pitch;*/
        /*if(pitch < 0)*/
        /*pitch = -(180 + pitch);*/
        /*}*/

        static int flag_debug_cnt = 0;
        flag_debug_cnt++;
        if (flag_debug_cnt == 100) {
                flag_debug_cnt = 0;
                log_debug("heading=%f, roll=%f, pitch=%f, yaw=%f", (float)heading/FACTOR, roll, pitch, yaw);
        }
}
//}}}
#endif
