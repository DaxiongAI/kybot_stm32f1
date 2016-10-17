#ifndef __FEEDBACK_H__
#define __FEEDBACK_H__
//50hz
#define FEEDBACK_FREQ 		50
void feedback_task(void);
void feedback_clean(void);
extern uint8_t send_core_sensor_data;
#endif
