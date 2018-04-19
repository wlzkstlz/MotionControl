#include "motor_driver.h"
#include "dac8562.h"
#include "stdlib.h"
#include "can.h"
#include "usart.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "pid_control.h"
#include "crc16_modbus.h"

#define MOTOR_DRIVER_MIDVALUE	32767
#define MOTOR_DRIVER_FULLVALUE	65535
#define	MOTOR_FULL_SPEED	1500.0f//	unit:r/min

#define	MOTOR_DAC_CH_L	1//0
#define	MOTOR_DAC_CH_R	0//1


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
  
  SetMotorEn(0,1);
  SetMotorEn(1,1);
  
  ResetPidControler();
  
  initSpeedFbCommunication();
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

#define MAX_OUT_VOLT 10.0
void setMotorForceByVolt(float Vl,float Vr)
{
  if(abs(Vl)>MAX_OUT_VOLT||abs(Vr)>MAX_OUT_VOLT)
  return;
  
  uint16_t dac_l=MOTOR_DRIVER_MIDVALUE;
	uint16_t dac_r=MOTOR_DRIVER_MIDVALUE;
	dac_l+=(int16_t)((float)Vl/MAX_OUT_VOLT*MOTOR_DRIVER_MIDVALUE);
	dac_r+=(int16_t)((float)Vr/MAX_OUT_VOLT*MOTOR_DRIVER_MIDVALUE);
	
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
#define MATH_PI 3.141592654

void cvtMotorSpeed(int16_t Vl,int16_t Vr,float *v,float *w)
{
	float left_v=Vl*MATH_PI*2*CAR_WHEEL_RADIUM/60.0;
	float right_v=Vr*MATH_PI*2*CAR_WHEEL_RADIUM/60.0;
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//伺服报警中断
{
  if(GPIO_Pin==GPIO_PIN_8||GPIO_Pin==GPIO_PIN_9)
  {
    SetMotorEn(0,0);
    SetMotorEn(1,0);
    SetControlMode(EMERGENCY_MODE);
  }
}





MotorControlMode gControlMode=SPEED_CMD_MODE;
void SetControlMode(MotorControlMode mode)
{
  gControlMode=mode;
}
MotorControlMode GetControlMode()
{
  return gControlMode;
}


uint32_t led_flash_time=100;
uint32_t led_time=0;
uint8_t led_bit=0;

void RunMotorControlMachine(MotorControlMode mode)
{
  //【执行状态机行为】
   switch(mode)
  {
    case FORCE_OPEN_MODE:
      funOpenForceMode();
      led_flash_time=1500;
      break;
    case FORCE_CLOSE_MODE:
      funCloseForceMode();
      led_flash_time=100;
      break;
    case SPEED_CMD_MODE:
      funSpeedCmdMode();
      led_flash_time=500;
      break;
    case EMERGENCY_MODE:
      funEmergencyMode();
      led_flash_time=50;
      break;
    default:
      HAL_Delay(100);
      break;
  }
  
  
  //【维持串口接收循环】
  HoldUartCommunication();
  
  //【闪烁led灯】
  if(HAL_GetTick()-led_time>led_flash_time)
  {
    led_time=HAL_GetTick();
    if(led_bit)
    {
      led_bit=0;
      HAL_GPIO_WritePin(GPIOD, LED_OUT_Pin, GPIO_PIN_SET);
    }
    else
    {
      led_bit=1;
      HAL_GPIO_WritePin(GPIOD, LED_OUT_Pin, GPIO_PIN_RESET);
    }
  }
  
}

void funOpenForceMode()
{
  static uint32_t motor_cmd_delay=0;
  int16_t vl_cmd,vr_cmd;
  if(getMotorSpeedCmd(&vl_cmd,&vr_cmd))
  {
    setMotorForceBySpeed(vl_cmd,vr_cmd);
    HAL_Delay(5);
    motor_cmd_delay=HAL_GetTick();
  }

  if(HAL_GetTick()-motor_cmd_delay>1000)
  {
    setMotorForceBySpeed(0,0);
    HAL_Delay(20);
  }
}


int16_t g_vl_cmd,g_vr_cmd;
void funCloseForceMode()
{
  static uint32_t speed_cmd_delay=0,speed_fb_delay=0;
  if(getMotorSpeedCmd(&g_vl_cmd,&g_vr_cmd))
  {
    speed_cmd_delay=HAL_GetTick();
  }
  
  int16_t vl_fb=0,vr_fb=0;
  if(GetMotorSpeedFb(&vl_fb,&vr_fb))//获取速度反馈
  {
    speed_fb_delay=HAL_GetTick();
  }
  
  if(HAL_GetTick()-speed_cmd_delay>1000)//如果超时没有接收到速度指令，则置零
  {
    setMotorForceBySpeed(0,0);
    ResetPidControler();
    g_vl_cmd=0;
    g_vr_cmd=0;
    HAL_Delay(20);
    return;
  }
  
  if(HAL_GetTick()-speed_fb_delay>50)//如果超时没有接收到速度反馈，则进入急停状态
  {
    setMotorForceBySpeed(0,0);
    SetControlMode(EMERGENCY_MODE);
    return;
  }
  
  
  
  float v_cmd=0,w_cmd=0,v_fb=0,w_fb=0;//换算指令速度和反馈速度
  cvtMotorSpeed(g_vl_cmd,g_vr_cmd,&v_cmd,&w_cmd);
  cvtMotorSpeed(vl_fb,vr_fb,&v_fb,&w_fb);
  
  ExeMotionControl(v_cmd,w_cmd,v_fb,w_fb);
  
  
  float f_left=0,f_right=0;//本质是模拟输出电压值
  GetPidForceOut(&f_left,&f_right);
  setMotorForceByVolt(f_left,f_right);
  
  HAL_Delay(5);
  
  //todo 未完待续
}



void funSpeedCmdMode()
{
  static uint32_t motor_cmd_delay=0;
  int16_t vl_cmd,vr_cmd;
  if(getMotorSpeedCmd(&vl_cmd,&vr_cmd))
  {
    setMotorSpeed(vl_cmd,vr_cmd);
    HAL_Delay(5);
    motor_cmd_delay=HAL_GetTick();
  }

  if(HAL_GetTick()-motor_cmd_delay>1000)
  {
    setMotorSpeed(0,0);
    HAL_Delay(20);
  }
}
void funEmergencyMode()
{
  setMotorSpeed(0,0);
  SetMotorEn(0,0);
  SetMotorEn(1,0);
  HAL_Delay(10);
}


//速度反馈
uint8_t ask_data[]={0x00,0x03,0x43,0x00,0x00,0x02,0xD0,0x5E};//查询指令
void AskForSpeedFb(uint8_t ch)
{
  if(ch==0)
  {
    HAL_UART_Transmit(&huart1,ask_data,sizeof(ask_data),50);
  }
  else 
  {
    HAL_UART_Transmit(&huart2,ask_data,sizeof(ask_data),50);
  }
}

uint8_t left_rev_byte=0;
uint8_t right_rev_byte=0;//串口接收数据

void initSpeedFbCommunication()//初始化串口接收
{
  HAL_UART_Receive_IT(&huart1,&left_rev_byte,1);
  HAL_UART_Receive_IT(&huart2,&right_rev_byte,1);
}

uint8_t uart1_need_restart=0;
uint8_t uart2_need_restart=0;
void SetNeedRestartUart(uint8_t ch,uint8_t need_restart)
{
  if(ch==0)
  {
    uart1_need_restart=need_restart;
  }
  else
  {
    uart2_need_restart=need_restart;
  }
}

uint8_t GetNeedRestartUart(uint8_t ch)
{
  if(ch==0)
    return uart1_need_restart;
  else
    return uart2_need_restart;
}

void HoldUartCommunication()
{
  if(GetNeedRestartUart(0))
  {
    HAL_StatusTypeDef ret=HAL_UART_Receive_IT(&huart1,&left_rev_byte,1);
    if(ret==HAL_OK)
    {
      SetNeedRestartUart(0,0);
    }
    else
    {
      HAL_Delay(1);
    }
  }
  
  if(GetNeedRestartUart(1))
  {
    HAL_StatusTypeDef ret=HAL_UART_Receive_IT(&huart2,&right_rev_byte,1);
    if(ret==HAL_OK)
    {
      SetNeedRestartUart(1,0);
    }
    else
    {
      HAL_Delay(1);
    }
  }
}


#define FB_RECEIVE_BUF_LENTH  9
uint8_t left_fb_data_buf[FB_RECEIVE_BUF_LENTH];
uint8_t right_fb_data_buf[FB_RECEIVE_BUF_LENTH];

uint8_t left_fb_register[FB_RECEIVE_BUF_LENTH];
uint8_t right_fb_register[FB_RECEIVE_BUF_LENTH];

uint32_t left_fb_timestamp=0;
uint32_t right_fb_timestamp=0;

uint8_t left_rev_index=0;
uint8_t right_rev_index=0;
uint8_t left_fb_data_new=0;
uint8_t right_fb_data_new=0;


//【串口回调函数】
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//左电机
	{
    if((left_rev_index==0&&left_rev_byte==0x00)||
      (left_rev_index==1&&left_rev_byte==0x03)||
    (left_rev_index==2&&left_rev_byte==0x04)||
    (left_rev_index>2&&left_rev_index<=7))
    {
      left_fb_data_buf[left_rev_index++]=left_rev_byte;
    }
    else if(left_rev_index==8)
    {
      left_fb_data_buf[left_rev_index]=left_rev_byte;
      left_rev_index=0;
      memcpy(left_fb_register,left_fb_data_buf,sizeof(left_fb_register));
      left_fb_data_new=1;
      left_fb_timestamp=HAL_GetTick();
    }
		
		HAL_StatusTypeDef ret=HAL_UART_Receive_IT(&huart1,&left_rev_byte,1);
		if(ret!=HAL_OK)
		{
			left_rev_index=0;
			SetNeedRestartUart(0,1);
		}
	}
  
  if(huart->Instance==USART2)//右电机
	{
    if((right_rev_index==0&&right_rev_byte==0x00)||
      (right_rev_index==1&&right_rev_byte==0x03)||
    (right_rev_index==2&&right_rev_byte==0x04)||
    (right_rev_index>2&&right_rev_index<=7))
    {
      right_fb_data_buf[right_rev_index++]=right_rev_byte;
    }
    else if(right_rev_index==8)
    {
      right_fb_data_buf[right_rev_index]=right_rev_byte;
      right_rev_index=0;
      memcpy(right_fb_register,right_fb_data_buf,sizeof(right_fb_register));
      right_fb_data_new=1;
      right_fb_timestamp=HAL_GetTick();
    }
		
		HAL_StatusTypeDef ret=HAL_UART_Receive_IT(&huart2,&right_rev_byte,1);
		if(ret!=HAL_OK)
		{
			right_rev_index=0;
			SetNeedRestartUart(1,1);
		}
	}
}

#define LEFT_RIGHT_FB_TIME_INTERVAL 50
uint8_t GetMotorSpeedFb(int16_t*vl,int16_t *vr)
{
  //todo
  if(left_fb_data_new==0||right_fb_data_new==0)
    return 0;
  
  if(left_fb_timestamp>right_fb_timestamp+LEFT_RIGHT_FB_TIME_INTERVAL)
  {
    right_fb_data_new=0;
    return 0;
  }
  
  if(right_fb_timestamp>left_fb_timestamp+LEFT_RIGHT_FB_TIME_INTERVAL)
  {
    left_fb_data_new=0;
    return 0;
  }
  
  unsigned short left_crc_should_be=CRC16_MODBUS(left_fb_register, 7);
  unsigned short left_crc_is=0;
  memcpy(&left_crc_is,left_fb_register+7,2);
  
  if(left_crc_should_be!=left_crc_is)//校验不通过！！！
  {
    left_fb_data_new=0;
    return 0;
  }
  
  unsigned short right_crc_should_be=CRC16_MODBUS(right_fb_register, 7);
  unsigned short right_crc_is=0;
  memcpy(&right_crc_is,right_fb_register+7,2);
  
  if(right_crc_should_be!=right_crc_is)//校验不通过！！！
  {
    right_fb_data_new=0;
    return 0;
  }
  
  int32_t left_vel_data=0,right_vel_data=0;
  left_vel_data=left_fb_register[5]<<24|left_fb_register[6]<<16|left_fb_register[3]<<8|left_fb_register[4];
  right_vel_data=right_fb_register[5]<<24|right_fb_register[6]<<16|right_fb_register[3]<<8|right_fb_register[4];
  int16_t left_vel=((left_vel_data>>12)*25/109);
  int16_t right_vel=((right_vel_data>>12)*25/109);
  (*vl)=left_vel;
  (*vr)=right_vel;
  return 1;
}









