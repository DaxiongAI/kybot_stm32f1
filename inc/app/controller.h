#ifndef __DIFF_CONTROLLER_H__
#define __DIFF_CONTROLLER_H__

#include "stm32f10x.h" 
/* Register Storage */
extern int8_t left_pwm;
extern int8_t right_pwm;
extern int8_t pid_type;

extern int16_t left_speed;
extern int16_t right_speed;
extern int16_t speed;
extern int16_t radius;
/* PID Parameters */
extern int32_t Kp;
extern int32_t Ki;
extern int32_t Kd;

/* PID modes */
extern uint32_t PIDmode;

#define PID_NONE        0
#define PID_SPEED       1

//����Ƶ��Խ�ߣ��Ա���������Ҳ��Խ�ߣ���С�ٶ�(Vmin=FRAMES*1000/TICKS_PER_METER[mm/s])Ҳ��ҪԽ��
#define PID_TIME        (20)	

// ��������pwm��������Ĳ�ֵ���ֵ�������ٶ�ƽ�����ƣ���ֹ�ٶȹ���ͻ�䣬��ɵ�Դ��ѹ����
#define MAX_DELTA_OUTPUT       20  
// max motor pwm
#define MAX_OUTPUT       127  


extern uint32_t f_time;   // last frame

extern uint8_t moving;    // base in motion
extern uint8_t paused;    // base was in motion, can resume
extern uint8_t enable_motor_sw_protection;

// http://yujinrobot.github.io/kobuki/doxygen/enAppendixKobukiParameters.html
// ��������,����תһȦ���̶��ٸ�����,kobuki�����2578.316
//#define ENCODER_TICKS (50*45) 
//#define ENCODER_TICKS (2360) 
// �Ŵ�10^7��,����ʹ�ø���������
// #define PI 3.1415926
#define PI 31415926

//��ֱ��,mm
//#define WHEEL_DIAMETER (62) 
//�ְ뾶,mm
//#define WHEEL_RADIUS  (WHEEL_DIAMETER/2) 
//mm
//#define WHEEL_BIAS  (226)	
// #define TICKS_PER_METER (ENCODER_TICKS/( PI * WHEEL_RADIUS / 1000 ))
// ����ǰ��1m�����������
// PI�Ŵ���10^7��������ҲҪ�Ŵ�
// #define TICKS_PER_METER (ENCODER_TICKS*1000*10000000/( PI * WHEEL_DIAMETER ))
// ����ǰ��1mm�����������
// #define TICKS_PER_MM (ENCODER_TICKS*10000000/( PI * WHEEL_DIAMETER ))

/* Setpoint Info For a Motor */
typedef struct{
  int16_t Velocity;              // desired actual speed (count/frame)
  uint16_t Encoder;              // actual reading
  uint16_t PrevEnc;              // last reading
  int16_t PrevErr;
  int16_t Ierror;   
  int8_t output;                // last motor setting
} SetPointInfo;

extern uint16_t wheel_bias;
extern uint16_t wheel_diameter;
extern uint16_t wheel_ticks;
extern SetPointInfo left,right;

void setup_pid(void);
void controller_reset(void);
void update_pid_task(void);
void controller_clean(void);
void set_pid(int p, int i, int d);
void robot_turn(uint16_t angle, int16_t angle_velocity);
void robot_forward(uint16_t distance, int16_t velocity);
void robot_forward_ticks(uint16_t ticks, int16_t velocity);
void set_base(int ticks, int diameter, int bias);
#endif
