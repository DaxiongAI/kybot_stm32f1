#ifndef __AT_CMD_H__
#define __AT_CMD_H__

//{{{ include
#include <stdio.h>
#include "at.h"
#include "at_base_cmd.h"
#include "at_motor_cmd.h"
#include "at_buzzer_cmd.h"
#include "at_encoder_cmd.h"
#include "at_log_cmd.h"
#include "at_pid_cmd.h"
#include "at_encrypt_cmd.h"
#include "at_led_cmd.h"
#include "at_wheel_cmd.h"
#include "at_imu_cmd.h"
#include "at_utils.h"
//}}}

#define AT_CMD_NUM   (30)


extern at_function_t at_fun[AT_CMD_NUM];

#endif
