#ifndef __pid_control_H
#define __pid_control_H

#include "stm32f1xx_hal.h"

#define	PID_FULL_F	10.0

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

void ExeControl(float Vc,float Wc,float Vl,float Vr);




void ResetLimitPa(void);
void UpdateLimitState(void);

#endif
