#include "motor_driver.h"
#include "dac8562.h"
#include "stdlib.h"
#include "can.h"
#include "string.h"

#define MOTOR_DRIVER_MIDVALUE	32767
#define MOTOR_DRIVER_FULLVALUE	65535
#define	MOTOR_FULL_SPEED	1500.0f//	unit:r/min

#define	MOTOR_DAC_CH_L	0
#define	MOTOR_DAC_CH_R	1


static CanRxMsgTypeDef	RxMessage;
CAN_FilterConfTypeDef gCanReceiveFilter1;
volatile uint8_t gSpeedCmdNew=0;
int16_t gSpeedCmdVl=0;
int16_t gSpeedCmdVr=0;

void initCan(void)
{
	hcan.pRxMsg = &RxMessage;
	
	gCanReceiveFilter1.FilterNumber=1;
	gCanReceiveFilter1.FilterMode=CAN_FILTERMODE_IDLIST;
	gCanReceiveFilter1.FilterScale=CAN_FILTERSCALE_16BIT;
	gCanReceiveFilter1.FilterIdHigh=0x30<<5;
	gCanReceiveFilter1.FilterIdLow=0x31<<5;
	gCanReceiveFilter1.FilterMaskIdHigh=0x32<<5;
	gCanReceiveFilter1.FilterMaskIdLow=0x33<<5;
	gCanReceiveFilter1.FilterFIFOAssignment=CAN_FilterFIFO0;
	gCanReceiveFilter1.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan, &gCanReceiveFilter1);
	
	HAL_CAN_Receive_IT(&hcan,CAN_FIFO0);
}

void initMotorDriver(void)
{
	gSpeedCmdNew=0;
	
	
	initCan();
	InitDAC8562();
}
	
void setMotorSpeed(int16_t Vl,int16_t Vr)
{
	if(abs(Vl)>MOTOR_FULL_SPEED||abs(Vr)>MOTOR_FULL_SPEED)
		return;
	
	uint16_t dac_l=MOTOR_DRIVER_MIDVALUE;
	uint16_t dac_r=MOTOR_DRIVER_MIDVALUE;
	dac_l+=(int16_t)((float)Vl/MOTOR_FULL_SPEED*MOTOR_DRIVER_MIDVALUE);
	dac_r+=(int16_t)((float)Vr/MOTOR_FULL_SPEED*MOTOR_DRIVER_MIDVALUE);
	
	DAC8562_SetData(MOTOR_DAC_CH_L,dac_l);
	DAC8562_SetData(MOTOR_DAC_CH_R,dac_r);
}


void setMotorForceBySpeed(int16_t Vl,int16_t Vr)
{
	if(abs(Vl)>MOTOR_FULL_SPEED||abs(Vr)>MOTOR_FULL_SPEED)
		return;
	
	float force_scale=0.5;
	
	uint16_t dac_l=MOTOR_DRIVER_MIDVALUE;
	uint16_t dac_r=MOTOR_DRIVER_MIDVALUE;
	dac_l+=(int16_t)((float)Vl/MOTOR_FULL_SPEED*MOTOR_DRIVER_MIDVALUE*force_scale);
	dac_r+=(int16_t)((float)Vr/MOTOR_FULL_SPEED*MOTOR_DRIVER_MIDVALUE*force_scale);
	
	DAC8562_SetData(MOTOR_DAC_CH_L,dac_l);
	DAC8562_SetData(MOTOR_DAC_CH_R,dac_r);	
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan->pRxMsg->StdId==0x30)
	{
		memcpy(&gSpeedCmdVl,&hcan->pRxMsg->Data[0],2);
		memcpy(&gSpeedCmdVr,&hcan->pRxMsg->Data[2],2);
		gSpeedCmdNew=1;
	}

	HAL_CAN_Receive_IT(hcan,CAN_FIFO0);
		
}


uint8_t getMotorSpeedCmd(int16_t*vl,int16_t*vr)
{
	(*vl)=gSpeedCmdVl;
	(*vr)=gSpeedCmdVr;
	
	if(gSpeedCmdNew==0)
		return 0;

	gSpeedCmdNew=0;
	return 1;
}


#define	CAR_WIDTH	0.652
#define CAR_WHEEL_RADIUM	0.1485
void cvtMotorSpeed(int16_t Vl,int16_t Vr,float *v,float *w)
{
	if(abs(Vl)>MOTOR_FULL_SPEED||abs(Vr)>MOTOR_FULL_SPEED)
	{
		(*v)=0;
		(*w)=0;
		return;
	}
	
	float left_v=Vl*0.5*3.1415926*2*CAR_WHEEL_RADIUM/60.0;
	float right_v=Vr*0.5*3.1415926*2*CAR_WHEEL_RADIUM/60.0;
	(*v)=(left_v+right_v)*0.5;
	(*w)=(right_v-left_v)/CAR_WIDTH;
	return;
}


//伺服使能与报警
void SetMotorEn(uint8_t id,uint8_t en)
{
  if(id==0)//左电机
  {
    if(en)
    {
      HAL_GPIO_WritePin(ENA_A1_GPIO_Port, ENA_A1_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(ENA_A1_GPIO_Port, ENA_A1_Pin, GPIO_PIN_RESET);
    }
  }
  else if(id==1)//右电机
  {
    if(en)
    {
      HAL_GPIO_WritePin(ENA_A2_GPIO_Port, ENA_A2_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(ENA_A2_GPIO_Port, ENA_A2_Pin, GPIO_PIN_RESET);
    }
  }
}



