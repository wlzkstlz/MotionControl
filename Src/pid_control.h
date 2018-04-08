#ifndef __pid_control_H
#define __pid_control_H
#include "stm32f1xx_hal.h"

#define	PID_FULL_F	10.0

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
void ExeControl(float Vc,float Wc,float Vl,float Vr,float* left_f_out,float *right_f_out);

//Force Limit
float g_cur_right_f_limit=PID_FULL_F;
float g_cur_left_f_limit=PID_FULL_F;


#endif
