//{{{ include
#include "motor.h"
#include "workdata.h"
#include "encoder.h"
#include "controller.h"
//}}}

//{{{ declare
motor_t motor;

uint32_t emergency = 0;
static uint16_t get_ccr_val(int8_t pwm);
//}}}

//{{{ init
/* Constructor: sets up microprocessor for PWM control of motors */
void motor_init(void){

        // Init structures
        GPIO_InitTypeDef GPIO_InitStructure;
        TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
        TIM_OCInitTypeDef  TIM_OCInitStructure;

        // Configure the GPIO for the timer output
        GPIO_InitStructure.GPIO_Pin = (LEFT_CTR1_PIN | LEFT_CTR2_PIN );
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(LEFT_CTR_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = (RIGHT_CTR1_PIN | RIGHT_CTR2_PIN );
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(RIGHT_CTR_GPIO, &GPIO_InitStructure);
        // Remap 
        // GPIO_PinRemapConfig(M_REMAP , ENABLE);

        GPIO_InitStructure.GPIO_Pin = MOTOR_FAULT_PIN;
	// 中断必须配置为浮空输入
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(MOTOR_FAULT_GPIO, &GPIO_InitStructure);
 
        EXTI_InitTypeDef EXTI_InitStructure;

        EXTI_ClearITPendingBit(EXTI_Line11);

        GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);

        EXTI_InitStructure.EXTI_Line = EXTI_Line11;   
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);

        // Timer configuration
        TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;//pwm周期
        TIM_TimeBaseStructure.TIM_Prescaler = PWM_PRESCALE;
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(MOTOR_PWM_TIM, &TIM_TimeBaseStructure);

        // PWM channels configuration (All identical!)
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 0;//当计数器计数小于这个值(CCR1)时，输出有效电平
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //当计数器计数值小于CCR1时，低电平为有效电平

        TIM_OC1Init(MOTOR_PWM_TIM,&TIM_OCInitStructure);
        TIM_OC2Init(MOTOR_PWM_TIM,&TIM_OCInitStructure);
        TIM_OC3Init(MOTOR_PWM_TIM,&TIM_OCInitStructure);
        TIM_OC4Init(MOTOR_PWM_TIM,&TIM_OCInitStructure);

        TIM_OC1PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
        TIM_OC2PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
        TIM_OC3PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
        TIM_OC4PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);

        TIM_SetCompare1(MOTOR_PWM_TIM, 0);
        TIM_SetCompare2(MOTOR_PWM_TIM, 0);
        TIM_SetCompare3(MOTOR_PWM_TIM, 0);
        TIM_SetCompare4(MOTOR_PWM_TIM, 0);

        TIM_ARRPreloadConfig(MOTOR_PWM_TIM, ENABLE);	
        // Enable the timer
        TIM_Cmd(MOTOR_PWM_TIM, ENABLE);			

        // Halt timer during debug halt.
        // DBGMCU_Config(M_TIMn_DBG, ENABLE);

}
//}}}

//{{{ setter
/* pwm values can range from -100 (full-speed reverse) to 100 (full-speed forward), with 0 indicating a stop */
void set_left_pwm(int8_t pwm){

        motor.l_pwm = pwm;

        if(pwm == 0){
                TIM_SetCompare1(MOTOR_PWM_TIM, get_ccr_val(0));
                TIM_SetCompare2(MOTOR_PWM_TIM, get_ccr_val(0));
        }
        else if(pwm > 0){
                TIM_SetCompare1(MOTOR_PWM_TIM, get_ccr_val(pwm));
                TIM_SetCompare2(MOTOR_PWM_TIM, get_ccr_val(0));
        }
        else{
                TIM_SetCompare1(MOTOR_PWM_TIM, get_ccr_val(0));
                TIM_SetCompare2(MOTOR_PWM_TIM, get_ccr_val(pwm));
        }  
}

/* lock the wheel in place */
void brake_left(void){
        
        TIM_SetCompare1(MOTOR_PWM_TIM, get_ccr_val(0));
        TIM_SetCompare2(MOTOR_PWM_TIM, get_ccr_val(0));

        motor.l_pwm = 0;
}

/* pwm values can range from -127 (full-speed reverse) to 127 (full-speed forward), with 0 indicating a stop */
void set_right_pwm(int8_t pwm){

        motor.r_pwm = pwm;

        if(pwm == 0){
                TIM_SetCompare3(MOTOR_PWM_TIM, get_ccr_val(0));
                TIM_SetCompare4(MOTOR_PWM_TIM, get_ccr_val(0));
        }
        else if(pwm > 0){
                TIM_SetCompare3(MOTOR_PWM_TIM, get_ccr_val(pwm));
                TIM_SetCompare4(MOTOR_PWM_TIM, get_ccr_val(0));
        }
        else{
                TIM_SetCompare3(MOTOR_PWM_TIM, get_ccr_val(0));
                TIM_SetCompare4(MOTOR_PWM_TIM, get_ccr_val(pwm));
        }
}

/* lock the wheel in place */
void brake_right(void){

        TIM_SetCompare3(MOTOR_PWM_TIM, get_ccr_val(0));
        TIM_SetCompare4(MOTOR_PWM_TIM, get_ccr_val(0));

        motor.r_pwm = 0;
}
void emergency_stop(void){
        emergency = 1;
        set_pwm(0, 0);
        controller_reset();
}
void set_pwm(int8_t lpwm, int8_t rpwm){
        // load_encoder_value();
        set_left_pwm(lpwm);
        set_right_pwm(rpwm);
}
//}}}

//{{{ getter
int8_t get_left_pwm(void){

        return motor.l_pwm;
}

int8_t get_right_pwm(void){

        return motor.r_pwm;
}

static uint16_t get_ccr_val(int8_t pwm)
{
        uint16_t tmp_ccr;

        if(pwm < -MAX_OUTPUT)
                pwm = -MAX_OUTPUT;
        else if(pwm > MAX_OUTPUT)
                pwm = MAX_OUTPUT;

        if(pwm < 0)
                pwm = -pwm;

        tmp_ccr = (uint16_t)pwm * (PWM_PERIOD + 1)/(MAX_OUTPUT);

        return tmp_ccr;
}
//}}}
