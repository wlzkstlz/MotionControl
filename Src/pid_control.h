#ifndef __pid_control_H
#define __pid_control_H
#include "stm32f1xx_hal.h"

#define	PID_FULL_F	10.0

void ResetPidControler(void);

//speed loop
float g_speed_errs[3]={0,0,0};
float g_cur_F=0;
float RunSpeedControl(float err);
void ResetSpeedControl(void);
void SetSpeedControlValue(float value);


//yaw_vel loop
float g_yaw_vel_errs[3]={0,0,0};
float g_cur_delta_F=0;
float RunYawVelControl(float err);
void ResetYawVelControl(void);
void SetYawVelControlValue(float value);

//Exe
float g_cur_right_F=0;
float g_cur_left_F=0;
void ExeControl(float Vc,float Wc,float Vl,float Vr);

//Force Limit
float g_cur_right_f_limit=PID_FULL_F;
float g_cur_left_f_limit=PID_FULL_F;

#define	OVERTQ_SCALE_OFFSET	0.1
#define	OVERFULLTQ_SCALE	(3.0-OVERTQ_SCALE_OFFSET)
#define	OVER25TQ_SCALE	(2.5-OVERTQ_SCALE_OFFSET)
#define	OVER20TQ_SCALE	(2.0-OVERTQ_SCALE_OFFSET)
#define	OVER15TQ_SCALE	(1.5-OVERTQ_SCALE_OFFSET)
#define	OVER12TQ_SCALE	(1.2-OVERTQ_SCALE_OFFSET)
#define	OVER11TQ_SCALE	(1.1-OVERTQ_SCALE_OFFSET)

#define	OVERFULLTQ_F	(OVERFULLTQ_SCALE*PID_FULL_F)
#define	OVER25TQ_F	(OVER25TQ_SCALE*PID_FULL_F)
#define	OVER20TQ_F	(OVER20TQ_SCALE*PID_FULL_F)
#define	OVER15TQ_F	(OVER15TQ_SCALE*PID_FULL_F)
#define	OVER12TQ_F	(OVER12TQ_SCALE*PID_FULL_F)
#define	OVER11TQ_F	(OVER11TQ_SCALE*PID_FULL_F)


#define	OVERTQ_TIME_OFFSET	1000
#define	OVER25TQ_TIME	(5000-OVERTQ_TIME_OFFSET)
#define	OVER20TQ_TIME	(15000-OVERTQ_TIME_OFFSET)
#define	OVER15TQ_TIME	(30000-OVERTQ_TIME_OFFSET)
#define	OVER12TQ_TIME	(60000-OVERTQ_TIME_OFFSET)
#define	OVER11TQ_TIME	(120000-OVERTQ_TIME_OFFSET)


uint32_t g_left_start_heat_25tq_t=0;
uint32_t g_left_start_heat_20tq_t=0;
uint32_t g_left_start_heat_15tq_t=0;
uint32_t g_left_start_heat_12tq_t=0;
uint32_t g_left_start_heat_11tq_t=0;

uint32_t g_left_start_cool_25tq_t=0;
uint32_t g_left_start_cool_20tq_t=0;
uint32_t g_left_start_cool_15tq_t=0;
uint32_t g_left_start_cool_12tq_t=0;
uint32_t g_left_start_cool_11tq_t=0;


uint32_t g_right_start_heat_25tq_t=0;
uint32_t g_right_start_heat_20tq_t=0;
uint32_t g_right_start_heat_15tq_t=0;
uint32_t g_right_start_heat_12tq_t=0;
uint32_t g_right_start_heat_11tq_t=0;

uint32_t g_right_start_cool_25tq_t=0;
uint32_t g_right_start_cool_20tq_t=0;
uint32_t g_right_start_cool_15tq_t=0;
uint32_t g_right_start_cool_12tq_t=0;
uint32_t g_right_start_cool_11tq_t=0;


void ResetLimitPa(void);
void UpdateLimitState(void);

#endif
