//{{{ include
#include <string.h>
#include "stm32f10x.h"
#include "param.h"
#include "encrypt.h"
#include "log.h"
#include "imu.h"
#include "controller.h"
#include "file_id.h"

#define this_file_id            file_id_param
//}}}

//{{{ define
#define EE_PID_P_HADDR	0
#define EE_PID_P_LADDR	1

#define EE_PID_I_HADDR	2
#define EE_PID_I_LADDR	3

#define EE_PID_D_HADDR	4
#define EE_PID_D_LADDR	5

#define EE_LOGLV_ADDR	6

#define EE_SWPROT_ADDR	7

#define EE_KEY0H_ADDR	8
#define EE_KEY0L_ADDR	9
#define EE_KEY1H_ADDR	10
#define EE_KEY1L_ADDR	11

// 升级固件标志
#define EE_FIRST_ON_ADDR	12
// 底盘参数
#define EE_WHEEL_BIAS_ADDR	13
#define EE_WHEEL_TICKS_ADDR	14
#define EE_WHEEL_DIAMETER_ADDR	15

#define EE_HEADING_FACTOR_LADDR	16
#define EE_HEADING_FACTOR_HADDR	17

#define EE_LOGCLR_ADDR	18
#define EE_IMUACT_ADDR	19

#define EE_RENAME_DBGBT_ADDR	20
#define EE_DBGBT_NAME_ADDR0	21
#define EE_DBGBT_NAME_ADDR1	22
#define EE_DBGBT_NAME_ADDR2	23

#define EE_RENAME_ROSBT_ADDR	24
#define EE_ROSBT_NAME_ADDR0	25
#define EE_ROSBT_NAME_ADDR1	26
#define EE_ROSBT_NAME_ADDR2	27

// 最后2K用于模拟eeprom
// 0xF800 = 62K
uint16_t VirtAddVarTab[NumbOfVar] = {	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
					0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
					0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
					0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
                                        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
					0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
					0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
					0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f};

//}}}

//{{{ init
void ee_init(void)
{

        FLASH_Unlock();
        EE_Init();
}
//}}}

//{{{ load
void param_load(void)
{
        ee_read_logclr();
        ee_read_pid();
        ee_read_loglv();
        ee_read_encrypt_key();
        ee_read_wheel();
        ee_read_heading_factor();
        /*ee_read_imuact();*/
        /*ee_read_swprot();*/
}
//}}}

//{{{ motor
// 电机软保护
void ee_save_swprot(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_SWPROT_ADDR], enable_motor_sw_protection);
}
void ee_read_swprot(void)
{
	// 应该初始化为0xffff
        uint16_t v = 0xffff;
        EE_ReadVariable(VirtAddVarTab[EE_SWPROT_ADDR], &v);

        if((v != 0) && (v != 1))
                return;
	
	log_debug("read motor sw protection = %s", (v==1) ? "enable" : "disable");
        enable_motor_sw_protection = v;
}
//}}}

//{{{ log
// 注意log level 是int 类型
void ee_save_loglv(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_LOGLV_ADDR], log_level);
}
void ee_read_loglv(void)
{
	// 应该初始化为0xffff
        uint16_t v = 0xffff;
        EE_ReadVariable(VirtAddVarTab[EE_LOGLV_ADDR], &v);

        if(v > LOG_LEVEL_NUM)
                return;
	
	log_debug("read log level = %d(%s)", v, log_level_to_str(v));
        log_level = v;
}
//}}}

//{{{ pid
void ee_save_pid(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_PID_P_HADDR], Kp >> 16);
	EE_WriteVariable(VirtAddVarTab[EE_PID_P_LADDR], Kp);
	EE_WriteVariable(VirtAddVarTab[EE_PID_I_HADDR], Ki >> 16);
	EE_WriteVariable(VirtAddVarTab[EE_PID_I_LADDR], Ki);
	EE_WriteVariable(VirtAddVarTab[EE_PID_D_HADDR], Kd >> 16);
	EE_WriteVariable(VirtAddVarTab[EE_PID_D_LADDR], Kd);
}
void ee_read_pid(void)
{
	// 应该初始化为0xffff
        uint16_t p_h = 0xffff, i_h = 0xffff, d_h = 0xffff, p_l = 0xffff, i_l = 0xffff, d_l = 0xffff;

        EE_ReadVariable(VirtAddVarTab[EE_PID_P_HADDR], &p_h);
        EE_ReadVariable(VirtAddVarTab[EE_PID_P_LADDR], &p_l);
        EE_ReadVariable(VirtAddVarTab[EE_PID_I_HADDR], &i_h);
        EE_ReadVariable(VirtAddVarTab[EE_PID_I_LADDR], &i_l);
        EE_ReadVariable(VirtAddVarTab[EE_PID_D_HADDR], &d_h);
        EE_ReadVariable(VirtAddVarTab[EE_PID_D_LADDR], &d_l);

        if((p_h == 0) && (i_h == 0) && (d_h == 0) && (p_l == 0) && (i_l == 0) && (d_l == 0)){
                /*log_debug("pid read 0");*/
                return;
        }

        if((p_h == 0xffff) && (i_h == 0xffff) && (d_h == 0xffff) && (p_l == 0xffff) && (i_l == 0xffff) && (d_l == 0xffff)){
                /*log_debug("pid read 0xffff");*/
                return;
        }

        Kp = ((int32_t)p_h << 16) | p_l;
        Ki = ((int32_t)i_h << 16) | i_l;
        Kd = ((int32_t)d_h << 16) | d_l;

        log_debug("read pid: p = %d, i = %d, d = %d", Kp, Ki, Kd);
}
//}}}

//{{{ encrypt
void ee_save_encrypt_key(uint32_t key0, uint32_t key1)
{
        read_encrypt_key0 = key0;
        read_encrypt_key1 = key1;
        EE_WriteVariable(VirtAddVarTab[EE_KEY0H_ADDR], read_encrypt_key0 >> 16);
        EE_WriteVariable(VirtAddVarTab[EE_KEY0L_ADDR], read_encrypt_key0);
        EE_WriteVariable(VirtAddVarTab[EE_KEY1H_ADDR], read_encrypt_key1 >> 16);
        EE_WriteVariable(VirtAddVarTab[EE_KEY1L_ADDR], read_encrypt_key1);
}
void ee_read_encrypt_key(void)
{
	// 应该初始化为0xffff
        uint16_t key0_h = 0xffff, key0_l = 0xffff, key1_h = 0xffff, key1_l = 0xffff;

        EE_ReadVariable(VirtAddVarTab[EE_KEY0H_ADDR], &key0_h);
        EE_ReadVariable(VirtAddVarTab[EE_KEY0L_ADDR], &key0_l);
        EE_ReadVariable(VirtAddVarTab[EE_KEY1H_ADDR], &key1_h);
        EE_ReadVariable(VirtAddVarTab[EE_KEY1L_ADDR], &key1_l);

        if((key0_h == 0xffff) && (key0_l == 0xffff) && (key1_h == 0xffff) && (key1_l == 0xffff)){
                log_fatal("encryption key is NOT set");
                return;
        }

        read_encrypt_key0 = ((int32_t)key0_h << 16) | key0_l;
        read_encrypt_key1 = ((int32_t)key1_h << 16) | key1_l;

        log_debug("read encryption key = %d,%d", read_encrypt_key0, read_encrypt_key1);
}
uint8_t is_first_on(void)
{
        uint16_t b = 0;
        EE_ReadVariable(VirtAddVarTab[EE_FIRST_ON_ADDR], &b);
        return b;
}
void set_not_first_on(void)
{
        EE_WriteVariable(VirtAddVarTab[EE_FIRST_ON_ADDR], 1);
}
//}}}

//{{{ wheel
void ee_save_wheel(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_WHEEL_BIAS_ADDR], wheel_bias);
	EE_WriteVariable(VirtAddVarTab[EE_WHEEL_TICKS_ADDR], wheel_ticks);
	EE_WriteVariable(VirtAddVarTab[EE_WHEEL_DIAMETER_ADDR], wheel_diameter);
}
void ee_save_wheel_bias(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_WHEEL_BIAS_ADDR], wheel_bias);
}
void ee_save_wheel_ticks(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_WHEEL_TICKS_ADDR], wheel_ticks);
}
void ee_save_wheel_diameter(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_WHEEL_DIAMETER_ADDR], wheel_diameter);
}
void ee_read_wheel(void)
{
	// 应该初始化为0xffff
        uint16_t bias = 0xffff, diameter = 0xffff, ticks = 0xffff;

        EE_ReadVariable(VirtAddVarTab[EE_WHEEL_BIAS_ADDR], &bias);
        EE_ReadVariable(VirtAddVarTab[EE_WHEEL_TICKS_ADDR], &ticks);
        EE_ReadVariable(VirtAddVarTab[EE_WHEEL_DIAMETER_ADDR], &diameter);

        if((bias == 0) || (ticks == 0) || (diameter == 0)){
                return;
        }

        if(bias != 0xffff){ 
                wheel_bias = bias;
                log_debug("read mobile base: bias = %d", bias);
        }
        if(diameter != 0xffff){ 
                wheel_diameter = diameter;
                log_debug("read mobile base: radius = %d", diameter / 2);
        }
        if(ticks != 0xffff){ 
                wheel_ticks = ticks;
                log_debug("read mobile base: ticks = %d", ticks);
        }
}
//}}}

//{{{ heading
void ee_save_heading_factor(uint32_t factor)
{
        EE_WriteVariable(VirtAddVarTab[EE_HEADING_FACTOR_HADDR], factor >> 16);
        EE_WriteVariable(VirtAddVarTab[EE_HEADING_FACTOR_LADDR], factor);
}
void ee_read_heading_factor(void)
{
	// 应该初始化为0xffff
        uint16_t factor_l = 0xffff, factor_h = 0xffff;
        uint32_t factor = 0;

        EE_ReadVariable(VirtAddVarTab[EE_HEADING_FACTOR_HADDR], &factor_h);
        EE_ReadVariable(VirtAddVarTab[EE_HEADING_FACTOR_LADDR], &factor_l);

        if((factor_h == 0xffff) && (factor_l == 0xffff)){
                /*log_debug("heading factor read 0xffff");*/
                return;
        }

        factor = ((uint32_t)factor_h << 16) | factor_l;
        heading_factor = (double)factor / (10^9);

        if (heading_factor > 1.1 || heading_factor < 0.9) {
                log_info("heading factor is NOT valid, it should be near 1.0");
                return;
        }
        log_debug("read heading factor = %.9f", heading_factor);
}
//}}}

//{{{ imu active when motor not work
#if 0
void ee_save_imuact(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_IMUACT_ADDR], enable_imu_when_motor_static);
}
void ee_read_imuact(void)
{
	// 应该初始化为0xffff
        uint16_t v = 0xffff;
        EE_ReadVariable(VirtAddVarTab[EE_IMUACT_ADDR], &v);

        if((v != 0) && (v != 1))
                return;
	log_debug("read imu active status = %s", (v==1) ? "enable" : "disable");
        enable_imu_when_motor_static = v;
}
//}}}
#endif

//{{{ log color
void ee_save_logclr(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_LOGCLR_ADDR], enable_log_color);
}
void ee_read_logclr(void)
{
	// 应该初始化为0xffff
        uint16_t v = 0xffff;
        EE_ReadVariable(VirtAddVarTab[EE_LOGCLR_ADDR], &v);

        if((v != 0) && (v != 1))
                return;
	log_debug("read log color status = %s", (v==1) ? "enable" : "disable");
        enable_log_color = v;
}
//}}}

//{{{ debug bt
uint8_t is_need_rename_dbg_bt(void)
{
        uint16_t b = 0;
        EE_ReadVariable(VirtAddVarTab[EE_RENAME_DBGBT_ADDR], &b);
        return b;
}
void clean_rename_dbg_bt_flag(void)
{
        EE_WriteVariable(VirtAddVarTab[EE_RENAME_DBGBT_ADDR], 0);
}
static void set_rename_dbg_bt_flag(void)
{
        EE_WriteVariable(VirtAddVarTab[EE_RENAME_DBGBT_ADDR], 1);
}
void save_dbg_bt_name(uint8_t name[7])
{
        EE_WriteVariable(VirtAddVarTab[EE_DBGBT_NAME_ADDR0], (name[1]<<8) | name[0]);
        EE_WriteVariable(VirtAddVarTab[EE_DBGBT_NAME_ADDR1], (name[3]<<8) | name[2]);
        EE_WriteVariable(VirtAddVarTab[EE_DBGBT_NAME_ADDR2], (name[5]<<8) | name[4]);
        set_rename_dbg_bt_flag();
}
void read_dbg_bt_name(uint8_t name[7])
{
        memset(name, '\0', 7);
        EE_ReadVariable(VirtAddVarTab[EE_DBGBT_NAME_ADDR0], (uint16_t *)&name[0]);
        EE_ReadVariable(VirtAddVarTab[EE_DBGBT_NAME_ADDR1], (uint16_t *)&name[2]);
        EE_ReadVariable(VirtAddVarTab[EE_DBGBT_NAME_ADDR2], (uint16_t *)&name[4]);
}
//}}}

//{{{ ros bt
uint8_t is_need_rename_ros_bt(void)
{
        uint16_t b = 0;
        EE_ReadVariable(VirtAddVarTab[EE_RENAME_ROSBT_ADDR], &b);
        return b;
}
void clean_rename_ros_bt_flag(void)
{
        EE_WriteVariable(VirtAddVarTab[EE_RENAME_ROSBT_ADDR], 0);
}
static void set_rename_ros_bt_flag(void)
{
        EE_WriteVariable(VirtAddVarTab[EE_RENAME_ROSBT_ADDR], 1);
}
void save_ros_bt_name(uint8_t name[7])
{
        EE_WriteVariable(VirtAddVarTab[EE_ROSBT_NAME_ADDR0], (name[1]<<8) | name[0]);
        EE_WriteVariable(VirtAddVarTab[EE_ROSBT_NAME_ADDR1], (name[3]<<8) | name[2]);
        EE_WriteVariable(VirtAddVarTab[EE_ROSBT_NAME_ADDR2], (name[5]<<8) | name[4]);
        set_rename_ros_bt_flag();
}
void read_ros_bt_name(uint8_t name[7])
{
        memset(name, '\0', 7);
        EE_ReadVariable(VirtAddVarTab[EE_ROSBT_NAME_ADDR0], (uint16_t *)&name[0]);
        EE_ReadVariable(VirtAddVarTab[EE_ROSBT_NAME_ADDR1], (uint16_t *)&name[2]);
        EE_ReadVariable(VirtAddVarTab[EE_ROSBT_NAME_ADDR2], (uint16_t *)&name[4]);
}
//}}}
