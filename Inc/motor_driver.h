#ifndef _MOTOR_DRIVER_H
#define _MOTOR_DRIVER_H
#include "stdint.h"
void initCan(void);
void initMotorDriver(void);
void setMotorSpeed(int16_t Vl,int16_t Vr);
uint8_t getMotorSpeedCmd(int16_t*vl,int16_t*vr);


uint8_t GetMotorSpeedFb(int16_t*vl,int16_t *vr);


void setMotorForceBySpeed(int16_t Vl,int16_t Vr);

void setMotorForceByVolt(float Vl,float Vr);

void cvtMotorSpeed(int16_t Vl,int16_t Vr,float *v,float *w);


//伺服使能与报警
void SetMotorEn(uint8_t id,uint8_t en);


//电机控制模式
typedef enum
{
  
  FORCE_CLOSE_MODE,
  FORCE_OPEN_MODE,
  SPEED_CMD_MODE,
  EMERGENCY_MODE
}MotorControlMode;


void SetControlMode(MotorControlMode mode);
MotorControlMode GetControlMode();

void RunMotorControlMachine(MotorControlMode mode);

void funOpenForceMode();
void funCloseForceMode();
void funSpeedCmdMode();
void funEmergencyMode();
#endif


