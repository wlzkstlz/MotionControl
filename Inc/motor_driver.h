#ifndef _MOTOR_DRIVER_H
#define _MOTOR_DRIVER_H
#include "stdint.h"
void initCan(void);
void initMotorDriver(void);
void setMotorSpeed(int16_t Vl,int16_t Vr);
uint8_t getMotorSpeedCmd(int16_t*vl,int16_t*vr);
#endif


