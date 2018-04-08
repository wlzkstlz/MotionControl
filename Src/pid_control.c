#include "pid_control.h"

#include <stdlib.h>
#include <math.h>

//speed loop
#define PID_SPEED_KP 1
#define PID_SPEED_KI 0
#define PID_SPEED_KD 0
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
#define PID_YAW_VEL_KI 0
#define PID_YAW_VEL_KD 0
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

#define CAR_WIDTH	0.8
void ExeControl(float Vc,float Wc,float Vl,float Vr,float* left_f_out,float *right_f_out)
{
	//偏差换算
	float speed_err=Vc-(Vl+Vr)*0.5;
	float yawvel_err=Wc-(Vr-Vl)/0.8;
	
	//分别进行速度与角速度pid
	RunSpeedControl(speed_err);
	RunYawVelControl(yawvel_err);
	
	//合力限制
	if(abs(g_cur_F)>g_cur_right_f_limit+g_cur_left_f_limit)
	{
		g_cur_F=g_cur_F>0?(g_cur_right_f_limit+g_cur_left_f_limit):-(g_cur_right_f_limit+g_cur_left_f_limit);
	}
	
	//力分解
	float left_f=0,right_f=0;
	left_f=(g_cur_F-g_cur_delta_F)/2;
	right_f=(g_cur_F+g_cur_delta_F)/2;
	
	//分力限制
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
	
	//重新计算差分力
	g_cur_delta_F=right_f-left_f;
	
	//输出力控制
	(*left_f_out)=left_f;
	(*right_f_out)=right_f;
}



