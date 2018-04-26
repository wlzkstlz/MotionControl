#include "pid_control.h"

#include <stdlib.h>
#include <math.h>


#define SMOOTH_THRESHOLD  1.0

void ResetPidControler(void)
{
	ResetSpeedControl();
	ResetYawVelControl();
	ResetLimitPa();
}



//speed loop
#define PID_SPEED_KP 10
#define PID_SPEED_KI 0.4
#define PID_SPEED_KD 0

float g_speed_errs[3]={0,0,0};
float g_cur_F=0;
float RunSpeedControl(float err)
{
	g_speed_errs[0]=g_speed_errs[1];
	g_speed_errs[1]=g_speed_errs[2];
	g_speed_errs[2]=err;
	
	float delta=PID_SPEED_KP*(g_speed_errs[2]-g_speed_errs[1])+PID_SPEED_KI*g_speed_errs[2]+PID_SPEED_KD*(g_speed_errs[2]-2*g_speed_errs[1]+g_speed_errs[0]);
	g_cur_F+=delta;
	return g_cur_F;
}

void ResetSpeedControl(void)
{
	g_speed_errs[0]=0;
	g_speed_errs[1]=0;
	g_speed_errs[2]=0;
	g_cur_F=0;
}


void SetSpeedControlValue(float value)
{
	g_cur_F=value;
}



//yaw_vel loop
#define PID_YAW_VEL_KP 1
#define PID_YAW_VEL_KI 0.2
#define PID_YAW_VEL_KD 0

float g_yaw_vel_errs[3]={0,0,0};
float g_cur_delta_F=0;
float RunYawVelControl(float err)
{
	g_yaw_vel_errs[0]=g_yaw_vel_errs[1];
	g_yaw_vel_errs[1]=g_yaw_vel_errs[2];
	g_yaw_vel_errs[2]=err;
	float delta=PID_YAW_VEL_KP*(g_yaw_vel_errs[2]-g_yaw_vel_errs[1])+PID_YAW_VEL_KI*g_yaw_vel_errs[2]+PID_YAW_VEL_KD*(g_yaw_vel_errs[2]-2*g_yaw_vel_errs[1]+g_yaw_vel_errs[0]);
	g_cur_delta_F+=delta;
	return g_cur_delta_F;
}

void ResetYawVelControl(void)
{
	g_yaw_vel_errs[0]=0;
	g_yaw_vel_errs[1]=0;
	g_yaw_vel_errs[2]=0;
	g_cur_delta_F=0;
}


void SetYawVelControlValue(float value)
{
	g_cur_delta_F=value;
}

float g_cur_right_F=0;
float g_cur_left_F=0;

float g_cur_right_f_limit=PID_FULL_F;
float g_cur_left_f_limit=PID_FULL_F;

//Force Limit


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



float g_Pre_F_left=0;
float g_Pre_F_right=0;
void ExeMotionControl(float Vcmd,float Wcmd,float Vfb,float Wfb)
{
	//ƫ���
	float speed_err=Vcmd-Vfb;
	float yawvel_err=Wcmd-Wfb;
	
	//�ֱ�����ٶ�����ٶ�pid
	RunSpeedControl(speed_err);
	RunYawVelControl(yawvel_err);
	
	//��������
	if(abs(g_cur_F)>g_cur_right_f_limit+g_cur_left_f_limit)
	{
		g_cur_F=g_cur_F>0?(g_cur_right_f_limit+g_cur_left_f_limit):-(g_cur_right_f_limit+g_cur_left_f_limit);
	}
	
	//���ֽ�
	float left_f=0,right_f=0;
	left_f=(g_cur_F-g_cur_delta_F)/2;
	right_f=(g_cur_F+g_cur_delta_F)/2;
	
	//��������
	if(abs(left_f)>g_cur_left_f_limit&&abs(left_f)-g_cur_left_f_limit>abs(right_f)-g_cur_right_f_limit)
	{
		float adjust=abs(left_f)-g_cur_left_f_limit;
		if(left_f>0)
		{
			left_f=left_f-adjust;
			right_f=right_f+adjust;
			
		}
		else
		{
			left_f=left_f+adjust;
			right_f=right_f-adjust;
		}
	}
	else 	if(abs(right_f)>g_cur_right_f_limit&&abs(right_f)-g_cur_right_f_limit>abs(left_f)-g_cur_left_f_limit)
	{
		float adjust=abs(right_f)-g_cur_right_f_limit;
		if(right_f>0)
		{
			right_f=right_f-adjust;
			left_f=left_f+adjust;
			
		}
		else
		{
			right_f=right_f+adjust;
			left_f=left_f-adjust;
		}
	}
  
  //Բ�����仯
  if(abs(left_f-g_Pre_F_left)>SMOOTH_THRESHOLD)
  {
    left_f=left_f-g_Pre_F_left>0?g_Pre_F_left+SMOOTH_THRESHOLD:g_Pre_F_left-SMOOTH_THRESHOLD;
  }
  if(abs(right_f-g_Pre_F_right)>SMOOTH_THRESHOLD)
  {
    right_f=right_f-g_Pre_F_right>0?g_Pre_F_right+SMOOTH_THRESHOLD:g_Pre_F_right-SMOOTH_THRESHOLD;
  }
  g_Pre_F_left=left_f;
  g_Pre_F_right=right_f;
	
	//���¼�������Ͳ����
  g_cur_F=right_f+left_f;
	g_cur_delta_F=right_f-left_f;
	
	//���������
	g_cur_left_F=left_f;
	g_cur_right_F=right_f;
  
  
  UpdateLimitState();
  
  printf("l_limit=%f,r_limit=%f  \n",g_cur_left_f_limit,g_cur_right_f_limit);
}



void ResetLimitPa(void)
{
	g_cur_left_f_limit=OVERFULLTQ_F;
	g_cur_right_f_limit=OVERFULLTQ_F;
	
	uint32_t cur_time=HAL_GetTick();
	
	g_left_start_heat_25tq_t=cur_time;
	g_left_start_heat_20tq_t=cur_time;
	g_left_start_heat_15tq_t=cur_time;
	g_left_start_heat_12tq_t=cur_time;
	g_left_start_heat_11tq_t=cur_time;

	g_right_start_heat_25tq_t=cur_time;
	g_right_start_heat_20tq_t=cur_time;
	g_right_start_heat_15tq_t=cur_time;
	g_right_start_heat_12tq_t=cur_time;
	g_right_start_heat_11tq_t=cur_time;
}


void UpdateLimitState(void)
{
	uint32_t cur_time=HAL_GetTick();//��ȡ��ǰʱ��
	
	//��*����*��
	//���жϼ���ʱ���Ƿ񳬳����ޣ��Ե������������ֵ��
	if(cur_time-g_left_start_heat_25tq_t>OVER25TQ_TIME)//���2.5������ʱ���Ѿ���������ֵ
	{
		g_cur_left_f_limit=g_cur_left_f_limit>OVER25TQ_F?OVER25TQ_F:g_cur_left_f_limit;//������ֵ��2.5������
	}
	
	if(cur_time-g_left_start_heat_20tq_t>OVER20TQ_TIME)//���2.0������ʱ���Ѿ���������ֵ
	{
		g_cur_left_f_limit=g_cur_left_f_limit>OVER20TQ_F?OVER20TQ_F:g_cur_left_f_limit;//������ֵ��2.0������
	}
	
	if(cur_time-g_left_start_heat_15tq_t>OVER15TQ_TIME)//���1.5������ʱ���Ѿ���������ֵ
	{
		g_cur_left_f_limit=g_cur_left_f_limit>OVER15TQ_F?OVER15TQ_F:g_cur_left_f_limit;//������ֵ��1.5������
	}
	
	if(cur_time-g_left_start_heat_12tq_t>OVER12TQ_TIME)//���1.2������ʱ���Ѿ���������ֵ
	{
		g_cur_left_f_limit=g_cur_left_f_limit>OVER12TQ_F?OVER12TQ_F:g_cur_left_f_limit;//������ֵ��1.2������
	}
	
	if(cur_time-g_left_start_heat_11tq_t>OVER11TQ_TIME)//���1.1������ʱ���Ѿ���������ֵ
	{
		g_cur_left_f_limit=g_cur_left_f_limit>OVER11TQ_F?OVER11TQ_F:g_cur_left_f_limit;//������ֵ��1.1������
	}
	
	
	//�����¼��ȡ���ȴ��ʼʱ�䡿
	if(abs(g_cur_left_F)<OVER11TQ_F)
	{
		g_left_start_heat_25tq_t=HAL_GetTick();
		g_left_start_heat_20tq_t=HAL_GetTick();
		g_left_start_heat_15tq_t=HAL_GetTick();
		g_left_start_heat_12tq_t=HAL_GetTick();
		g_left_start_heat_11tq_t=HAL_GetTick();
	}
	else 	if(abs(g_cur_left_F)<OVER12TQ_F)
	{
		g_left_start_heat_25tq_t=HAL_GetTick();
		g_left_start_heat_20tq_t=HAL_GetTick();
		g_left_start_heat_15tq_t=HAL_GetTick();
		g_left_start_heat_12tq_t=HAL_GetTick();

		g_left_start_cool_11tq_t=HAL_GetTick();
	}
	else 	if(abs(g_cur_left_F)<OVER15TQ_F)
	{
		g_left_start_heat_25tq_t=HAL_GetTick();
		g_left_start_heat_20tq_t=HAL_GetTick();
		g_left_start_heat_15tq_t=HAL_GetTick();
		
		g_left_start_cool_12tq_t=HAL_GetTick();
		g_left_start_cool_11tq_t=HAL_GetTick();
	}
	else 	if(abs(g_cur_left_F)<OVER20TQ_F)
	{
		g_left_start_heat_25tq_t=HAL_GetTick();
		g_left_start_heat_20tq_t=HAL_GetTick();
		
		g_left_start_cool_15tq_t=HAL_GetTick();
		g_left_start_cool_12tq_t=HAL_GetTick();
		g_left_start_cool_11tq_t=HAL_GetTick();
	}
	else 	if(abs(g_cur_left_F)<OVER25TQ_F)
	{
		g_left_start_heat_25tq_t=HAL_GetTick();
		
		g_left_start_cool_20tq_t=HAL_GetTick();
		g_left_start_cool_15tq_t=HAL_GetTick();
		g_left_start_cool_12tq_t=HAL_GetTick();
		g_left_start_cool_11tq_t=HAL_GetTick();
	}
	else
	{
		g_left_start_cool_25tq_t=HAL_GetTick();
		g_left_start_cool_20tq_t=HAL_GetTick();
		g_left_start_cool_15tq_t=HAL_GetTick();
		g_left_start_cool_12tq_t=HAL_GetTick();
		g_left_start_cool_11tq_t=HAL_GetTick();
	}
	
	//���ж���ȴʱ���Ƿ���㣬�Խ����Ӧ���ơ�
	if(cur_time-g_left_start_cool_11tq_t>OVER11TQ_TIME)//���1.1����ȴʱ���Ѿ��㹻
	{
		g_cur_left_f_limit=g_cur_left_f_limit<OVER12TQ_F?OVER12TQ_F:g_cur_left_f_limit;//������ֵ�ϸ���1.1������
	}
	
	if(cur_time-g_left_start_cool_12tq_t>OVER12TQ_TIME)//���1.2����ȴʱ���Ѿ��㹻
	{
		g_cur_left_f_limit=g_cur_left_f_limit<OVER15TQ_F?OVER15TQ_F:g_cur_left_f_limit;//������ֵ�ϸ���1.2������
	}
	
	if(cur_time-g_left_start_cool_15tq_t>OVER15TQ_TIME)//���1.5����ȴʱ���Ѿ��㹻
	{
		g_cur_left_f_limit=g_cur_left_f_limit<OVER20TQ_F?OVER20TQ_F:g_cur_left_f_limit;//������ֵ�ϸ���1.5������
	}
	
	if(cur_time-g_left_start_cool_20tq_t>OVER20TQ_TIME)//���2.0����ȴʱ���Ѿ��㹻
	{
		g_cur_left_f_limit=g_cur_left_f_limit<OVER25TQ_F?OVER25TQ_F:g_cur_left_f_limit;//������ֵ�ϸ���2.0������
	}
	
	if(cur_time-g_left_start_cool_25tq_t>OVER25TQ_TIME)//���2.5����ȴʱ���Ѿ��㹻
	{
		g_cur_left_f_limit=g_cur_left_f_limit<OVERFULLTQ_F?OVERFULLTQ_F:g_cur_left_f_limit;//������ֵ�ϸ���2.5������
	}
	
	
	//��*�ҵ��*��
	//���жϼ���ʱ���Ƿ񳬳����ޣ��Ե������������ֵ��
	if(cur_time-g_right_start_heat_25tq_t>OVER25TQ_TIME)//���2.5������ʱ���Ѿ���������ֵ
	{
		g_cur_right_f_limit=g_cur_right_f_limit>OVER25TQ_F?OVER25TQ_F:g_cur_right_f_limit;//������ֵ��2.5������
	}
	
	if(cur_time-g_right_start_heat_20tq_t>OVER20TQ_TIME)//���2.0������ʱ���Ѿ���������ֵ
	{
		g_cur_right_f_limit=g_cur_right_f_limit>OVER20TQ_F?OVER20TQ_F:g_cur_right_f_limit;//������ֵ��2.0������
	}
	
	if(cur_time-g_right_start_heat_15tq_t>OVER15TQ_TIME)//���1.5������ʱ���Ѿ���������ֵ
	{
		g_cur_right_f_limit=g_cur_right_f_limit>OVER15TQ_F?OVER15TQ_F:g_cur_right_f_limit;//������ֵ��1.5������
	}
	
	if(cur_time-g_right_start_heat_12tq_t>OVER12TQ_TIME)//���1.2������ʱ���Ѿ���������ֵ
	{
		g_cur_right_f_limit=g_cur_right_f_limit>OVER12TQ_F?OVER12TQ_F:g_cur_right_f_limit;//������ֵ��1.2������
	}
	
	if(cur_time-g_right_start_heat_11tq_t>OVER11TQ_TIME)//���1.1������ʱ���Ѿ���������ֵ
	{
		g_cur_right_f_limit=g_cur_right_f_limit>OVER11TQ_F?OVER11TQ_F:g_cur_right_f_limit;//������ֵ��1.1������
	}
	
	
	//�����¼��ȡ���ȴ��ʼʱ�䡿
	if(abs(g_cur_right_F)<OVER11TQ_F)
	{
		g_right_start_heat_25tq_t=HAL_GetTick();
		g_right_start_heat_20tq_t=HAL_GetTick();
		g_right_start_heat_15tq_t=HAL_GetTick();
		g_right_start_heat_12tq_t=HAL_GetTick();
		g_right_start_heat_11tq_t=HAL_GetTick();
	}
	else 	if(abs(g_cur_right_F)<OVER12TQ_F)
	{
		g_right_start_heat_25tq_t=HAL_GetTick();
		g_right_start_heat_20tq_t=HAL_GetTick();
		g_right_start_heat_15tq_t=HAL_GetTick();
		g_right_start_heat_12tq_t=HAL_GetTick();

		g_right_start_cool_11tq_t=HAL_GetTick();
	}
	else 	if(abs(g_cur_right_F)<OVER15TQ_F)
	{
		g_right_start_heat_25tq_t=HAL_GetTick();
		g_right_start_heat_20tq_t=HAL_GetTick();
		g_right_start_heat_15tq_t=HAL_GetTick();
		
		g_right_start_cool_12tq_t=HAL_GetTick();
		g_right_start_cool_11tq_t=HAL_GetTick();
	}
	else 	if(abs(g_cur_right_F)<OVER20TQ_F)
	{
		g_right_start_heat_25tq_t=HAL_GetTick();
		g_right_start_heat_20tq_t=HAL_GetTick();
		
		g_right_start_cool_15tq_t=HAL_GetTick();
		g_right_start_cool_12tq_t=HAL_GetTick();
		g_right_start_cool_11tq_t=HAL_GetTick();
	}
	else 	if(abs(g_cur_right_F)<OVER25TQ_F)
	{
		g_right_start_heat_25tq_t=HAL_GetTick();
		
		g_right_start_cool_20tq_t=HAL_GetTick();
		g_right_start_cool_15tq_t=HAL_GetTick();
		g_right_start_cool_12tq_t=HAL_GetTick();
		g_right_start_cool_11tq_t=HAL_GetTick();
	}
	else
	{
		g_right_start_cool_25tq_t=HAL_GetTick();
		g_right_start_cool_20tq_t=HAL_GetTick();
		g_right_start_cool_15tq_t=HAL_GetTick();
		g_right_start_cool_12tq_t=HAL_GetTick();
		g_right_start_cool_11tq_t=HAL_GetTick();
	}
	
	//���ж���ȴʱ���Ƿ���㣬�Խ����Ӧ���ơ�
	if(cur_time-g_right_start_cool_11tq_t>OVER11TQ_TIME)//���1.1����ȴʱ���Ѿ��㹻
	{
		g_cur_right_f_limit=g_cur_right_f_limit<OVER12TQ_F?OVER12TQ_F:g_cur_right_f_limit;//������ֵ�ϸ���1.1������
	}
	
	if(cur_time-g_right_start_cool_12tq_t>OVER12TQ_TIME)//���1.2����ȴʱ���Ѿ��㹻
	{
		g_cur_right_f_limit=g_cur_right_f_limit<OVER15TQ_F?OVER15TQ_F:g_cur_right_f_limit;//������ֵ�ϸ���1.2������
	}
	
	if(cur_time-g_right_start_cool_15tq_t>OVER15TQ_TIME)//���1.5����ȴʱ���Ѿ��㹻
	{
		g_cur_right_f_limit=g_cur_right_f_limit<OVER20TQ_F?OVER20TQ_F:g_cur_right_f_limit;//������ֵ�ϸ���1.5������
	}
	
	if(cur_time-g_right_start_cool_20tq_t>OVER20TQ_TIME)//���2.0����ȴʱ���Ѿ��㹻
	{
		g_cur_right_f_limit=g_cur_right_f_limit<OVER25TQ_F?OVER25TQ_F:g_cur_right_f_limit;//������ֵ�ϸ���2.0������
	}
	
	if(cur_time-g_right_start_cool_25tq_t>OVER25TQ_TIME)//���2.5����ȴʱ���Ѿ��㹻
	{
		g_cur_right_f_limit=g_cur_right_f_limit<OVERFULLTQ_F?OVERFULLTQ_F:g_cur_right_f_limit;//������ֵ�ϸ���2.5������
	}

}



void GetPidForceOut(float *f_left,float *f_right)
{
  (*f_left)=g_cur_left_F;
  (*f_right)=g_cur_right_F;
}





