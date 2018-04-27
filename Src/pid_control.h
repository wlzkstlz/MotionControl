#ifndef __pid_control_H
#define __pid_control_H

#include "stm32f1xx_hal.h"



void ResetPidControler(void);

//speed loop
float RunSpeedControl(float err);
void ResetSpeedControl(void);
void SetSpeedControlValue(float value);


//yaw_vel loop
float RunYawVelControl(float err);
void ResetYawVelControl(void);
void SetYawVelControlValue(float value);

//Exe

void ExeMotionControl(float Vcmd,float Wcmd,float Vfb,float Wfb);




void ResetLimitPa(void);
void UpdateLimitState(void);


void GetPidForceOut(float *f_left,float *f_right);

#endif
