#ifndef __AT_BASECMD_H
#define __AT_BASECMD_H

void at_version_query(uint8_t id);
void at_fwinfo_query(uint8_t id);
void at_battery_vol_query(uint8_t id);
void at_exec_cmd_imu_calibrate(uint8_t id);
void at_exec_cmd_null(uint8_t id);
void at_exec_cmd_reset(uint8_t id);
void at_exec_cmd_flash_unlock(uint8_t id);
void at_time_query(uint8_t id);
void at_dbgbt_name_set(uint8_t id, uint8_t *pPara);
void at_rosbt_name_set(uint8_t id, uint8_t *pPara);
#endif
