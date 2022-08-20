/**
 ***************************************(C) COPYRIGHT 2021 CSS***************************************
 * @file       pid.c
 * @brief      this file contains pid processing function
 * @note       
 * @Version    V1.0.0
 * @Date       Feb-15-2021
 ***************************************(C) COPYRIGHT 2021 CSS***************************************
 */

#include "pid.h"
#include "main.h"

//#define LimitMax(input, max)   	\
//{                        				\
//	if (input > max)       				\
//  {                      				\
//		input = max;       	 				\
//  }                      				\
//  else if (input < -max) 				\
//  {                      				\
//		input = -max;        				\
//  }                      				\
//}

//void PID_init(PidTypeDef *pid,uint8_t mode,const fp32 PID[3],fp32 max_out,fp32 max_iout)
//{
//	if(pid==NULL||PID==NULL)
//	{
//		return;
//	}
//	pid->mode=mode;
//	pid->Kp=PID[0];
//	pid->Ki=PID[1];
//	pid->Kd=PID[2];
//	pid->max_out=max_out;
//	pid->max_iout=max_iout;
//	pid->Dbuf[0]=pid->Dbuf[1]=pid->Dbuf[2]=0.0f;
//	pid->error[0]=pid->error[1]=pid->error[2]=pid->Pout=pid->Iout=pid->Dout=pid->out=0.0f;
//}

//fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
//{
//    if (pid == NULL)
//    {
//        return 0.0f;
//    }

//    pid->error[2] = pid->error[1];
//    pid->error[1] = pid->error[0];
//    pid->set = set;
//    pid->fdb = ref;
//    pid->error[0] = set - ref;
//		
//    if (pid->mode == PID_POSITION)
//    {
//        pid->Pout = pid->Kp * pid->error[0];
//        pid->Iout += pid->Ki * pid->error[0];
//        pid->Dbuf[2] = pid->Dbuf[1];
//        pid->Dbuf[1] = pid->Dbuf[0];
//        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
//        pid->Dout = pid->Kd * pid->Dbuf[0];
//        LimitMax(pid->Iout, pid->max_iout);
//        pid->out = pid->Pout + pid->Iout + pid->Dout;
//        LimitMax(pid->out, pid->max_out);
//    }
//    else if (pid->mode == PID_DELTA)
//    {
//        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
//        pid->Iout = pid->Ki * pid->error[0];
//        pid->Dbuf[2] = pid->Dbuf[1];
//        pid->Dbuf[1] = pid->Dbuf[0];
//        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
//        pid->Dout = pid->Kd * pid->Dbuf[0];
//        pid->out += pid->Pout + pid->Iout + pid->Dout;
//        LimitMax(pid->out, pid->max_out);
//    }
//    return pid->out;
//}
  



//#include "PID.h"

PidTypeDef CM1_speed_pid={0};
PidTypeDef CM2_speed_pid={0};
PidTypeDef CM3_speed_pid={0};
PidTypeDef CM4_speed_pid={0};
PidTypeDef CM5_speed_pid={0};
PidTypeDef CM6_speed_pid={0};

PidTypeDef Gyro_normal_pid;
PidTypeDef Yaw_speed_pid={0};
PidTypeDef Yaw_position_pid={0};
PidTypeDef Yaw_Auto_Aim_pid={0};
PidTypeDef Pitch_speed_pid={0};
PidTypeDef Pitch_position_pid={0};
PidTypeDef Pitch_Auto_Aim_pid={0};
PidTypeDef Shoot_speed_pid={0};
PidTypeDef Shoot_position_pid={0};
PidTypeDef CM1_current_pid={0};
PidTypeDef CM2_current_pid={0};
PidTypeDef CM3_current_pid={0};
PidTypeDef CM4_current_pid={0};

void PID_SET_Init(void)
{
//	PID_Init(PidTypeDef * pid,double kp,double ki,double kd,double ka,double max_out,double dead_band,double i_band,double max_input)
	PID_Init(&CM1_speed_pid,5,0,0,0,10000,0,0,10000);
	PID_Init(&CM2_speed_pid,5,0,0,0,10000,0,0,10000);
	PID_Init(&CM3_speed_pid,5,0,0,0,10000,0,0,10000);
	PID_Init(&CM4_speed_pid,5,0,0,0,10000,0,0,10000);
	PID_Init(&CM5_speed_pid,9,0.00008,0,0,13000,0,0,13000);
	PID_Init(&CM6_speed_pid,9,0.00006,0,0,13000,0,0,13000);
	
	PID_Init(&CM1_current_pid,6,0,0,0,10000,0,0,7260);
	PID_Init(&CM2_current_pid,6,0,0,0,10000,0,0,7260);
	PID_Init(&CM3_current_pid,6,0,0,0,10000,0,0,7260);
	PID_Init(&CM4_current_pid,6,0,0,0,10000,0,0,7260);

	PID_Init(&Gyro_normal_pid,1.25,0,0,0,1000000,10,0,81092);
	
	PID_Init(&Yaw_speed_pid,130,30,0,0,30000,0,0,99999999);//Yaw和Pitch的速度环和位置环  0.85 2.4
	PID_Init(&Yaw_position_pid,15,2,0,0,16000,0,0,999999);// 8
	PID_Init(&Yaw_Auto_Aim_pid,0.8,0.15,0.05,0,36000,0,0,7000);
	
	PID_Init(&Pitch_speed_pid,-51,-5,0,0,30000,0,0,36000);   
	PID_Init(&Pitch_position_pid,0.51,0.1,0,0,16000,0,0,36000);   
	PID_Init(&Pitch_Auto_Aim_pid,-1,-0.1,0,0,36000,0,0,7000);
	
	PID_Init(&Shoot_position_pid,9,0,0,0,10000,0,0,999999999);	
	PID_Init(&Shoot_speed_pid,1.8,0,0,0,30000,0,0,30000);	
}


//初始化PID
void PID_Init(PidTypeDef * pid,double kp,double ki,double kd,double ka,double max_out,double dead_band,double i_band,double max_input)
{
	pid->Kp=kp;
	pid->Ki=ki;
	pid->Kd=kd;
	pid->Ka=ka;
	pid->max_out=max_out;
	pid->dead_band=dead_band;
	pid->intergral_band=i_band;
	pid->max_input=max_input;
	pid->output=0;
	pid->e[0]=0;
	pid->e[1]=0;
	pid->e[2]=0;
	pid->d_last=0;
}

//PID计算
void PID_Calc(PidTypeDef * pid, double rel_val, double set_val,uint8_t PID_mode)
{
	double p = 0,
			i = 0,
			d = 0;
	if(set_val > pid->max_input)
		set_val = pid->max_input;
	else if(set_val < -(pid->max_input))
		set_val = -(pid->max_input);
	pid->e[2] = set_val - rel_val; 
		if(pid==&Shoot_position_pid||pid==&Gyro_normal_pid||pid==&Yaw_position_pid)
		{
			if(pid->e[2]>(pid->max_input/2))
			{
				pid->e[2]=pid->e[2]-pid->max_input;
			}
			else if((-pid->e[2])>(pid->max_input/2))
			{
				pid->e[2]=pid->max_input+pid->e[2];
			}
		}

	if(MyAbs(pid->e[2]) >=pid->dead_band)
	{
		if(MyAbs(pid->e[2]) <= pid->intergral_band)
			pid->intergral =pid->intergral+ (pid->Ki) * (pid->e[2]);	
		else
			{pid->intergral=pid->intergral*0.99;}
		if(pid->intergral > pid->max_out)
			pid->intergral= pid->max_out;
		else if(pid->intergral < -pid->max_out)
			pid->intergral= -pid->max_out;
		if(PID_mode==Positional)
		{		
  		p = pid->Kp * (pid->e[2]);
			i = pid->intergral;
			d = pid->Kd * (1-pid->Ka)*(pid->e[2]-pid->e[1])+(pid->Ka)*(pid->d_last);
			pid->d_last=d;
			pid->output =p+i+d;
		}
		else if(PID_mode==Incremental)
		{
			p = pid->Kp * ((pid->e[2]-pid->e[1]));
			i = pid->Ki*pid->e[2];
			d = pid->Kd *(pid->e[2]-pid->e[1]*2+pid->e[0]);
			pid->output +=p+i+d;
		}			
		if(pid->output > pid->max_out)
			pid->output = pid->max_out;
		if(pid->output < -(pid->max_out))
			pid->output = -(pid->max_out);
	}

	else
	{
		pid->output=0;
	}
	pid->e[0] = pid->e[1];
	pid->e[1] = pid->e[2];
}

//PID计算
void PID2_Calc(PidTypeDef * pid, double rel_val, double set_val,uint8_t PID_mode)
{
	double p = 0,
			i = 0,
			d = 0;
//	if(set_val > pid->max_input)
//		set_val = pid->max_input;
//	else if(set_val < -(pid->max_input))
//		set_val = -(pid->max_input);
	pid->e[2] = set_val - rel_val; 
		if(pid==&Shoot_position_pid||pid==&Gyro_normal_pid||pid==&Yaw_position_pid)
		{
			if(pid->e[2]>(pid->max_input/2))
			{
				pid->e[2]=pid->e[2]-pid->max_input;
			}
			else if((-pid->e[2])>(pid->max_input/2))
			{
				pid->e[2]=pid->max_input+pid->e[2];
			}
		}

	if(MyAbs(pid->e[2]) >=pid->dead_band)
	{
		if(MyAbs(pid->e[2]) <= pid->intergral_band)
			pid->intergral =pid->intergral+ (pid->Ki) * (pid->e[2]);	
		else
			{pid->intergral=pid->intergral*0.99;}
		if(pid->intergral > pid->max_out)
			pid->intergral= pid->max_out;
		else if(pid->intergral < -pid->max_out)
			pid->intergral= -pid->max_out;
		if(PID_mode==Positional)
		{		
  		p = pid->Kp * (pid->e[2]);
			i = pid->intergral;
			d = pid->Kd * (1-pid->Ka)*(pid->e[2]-pid->e[1])+(pid->Ka)*(pid->d_last);
			pid->d_last=d;
			pid->output =p+i+d;
		}
		else if(PID_mode==Incremental)
		{
			p = pid->Kp * ((pid->e[2]-pid->e[1]));
			i = pid->Ki*pid->e[2];
			d = pid->Kd *(pid->e[2]-pid->e[1]*2+pid->e[0]);
			pid->output +=p+i+d;
		}			
//		if(pid->output > pid->max_out)
//			pid->output = pid->max_out;
//		if(pid->output < -(pid->max_out))
//			pid->output = -(pid->max_out);
	}

	else
	{
		pid->output=0;
	}
	pid->e[0] = pid->e[1];
	pid->e[1] = pid->e[2];
}

float MyAbs(float num)
{
	if(num>=0)
		return num;
	else 
		return -num;	
}


void PID_Calc_Position(PidTypeDef * pid, double ret_val, double set_val)//单环位置式皮带
{
	double p = 0,i = 0,d = 0,ep=0;
	if(set_val > pid->max_input)
		set_val = pid->max_input;
	else if(set_val < -(pid->max_input))
		set_val = -(pid->max_input);
	
	pid->e[1]=set_val-ret_val;
	

		ep+=pid->e[1];
		p=pid->Kp*pid->e[1];
		i=pid->Ki*ep;
		d=pid->Kd*(pid->e[1]-pid->e[0]);

				pid->output=p+i+d;
	
		
		if(pid->output > pid->max_out)
			pid->output = pid->max_out;
		if(pid->output < -(pid->max_out))
			pid->output = -(pid->max_out);


	

	pid->e[0] = pid->e[1];

}


void PID_Calc_Speed(PidTypeDef * pid, double ret_val, double set_val)//单环式皮带
{
	double p = 0,i = 0,d = 0;
	if(set_val > pid->max_input)
		set_val = pid->max_input;
	else if(set_val < -(pid->max_input))
		set_val = -(pid->max_input);
	
	pid->e[2]=set_val-ret_val;
	
	

		
		
		p=pid->Kp*(pid->e[2]-pid->e[1]);
		i=pid->Ki*pid->e[2];
		d=pid->Kd*(pid->e[2]-2*pid->e[1]+pid->e[0]);

//			if(pid->output> pid->max_out)//遇限消积分
//				if(i<0)
//					pid->output+=p+i+d;
//			if(pid->output< pid->max_out)//遇限消积分
//				if(i>0)
//					pid->output+=p+i+d;
//			if(pid->output> (-pid->max_out) && pid->output< pid->max_out)
				pid->output+=p+i+d;

		if(pid->output > pid->max_out)
			pid->output = pid->max_out;
		if(pid->output < -(pid->max_out))
			pid->output = -(pid->max_out);



	pid->e[0] = pid->e[1];
	pid->e[1] = pid->e[2];

}

void PID_Calc1(PidTypeDef * pid, double rel_val, double set_val,uint8_t PID_mode)   //位置式 PID 控制设计  
{
	double p = 0,i = 0,d = 0;				
	//PID_Calc(&CM1_speed_pid,CM1_Feedback.real[0],CM1Speed,Incremental);
	if(set_val > pid->max_input)
		set_val = pid->max_input;
	else if(set_val < -(pid->max_input))
		set_val = -(pid->max_input);  //不能超过最大输入
	
	if (PID_mode ==Positional )			//位置式
	{
	pid->e[2] = set_val - rel_val; //更新误差
	pid->sum_e+=pid->e[2];
	pid->d_band=pid->e[2]-pid->e[1];  //微分项Incremental
	pid->e[1] = pid->e[2];
		p=pid->Kp*pid->e[2];    
		i=pid->Ki*pid->sum_e;
		d=pid->Kd*pid->d_band;    //比例项限幅50%
		
	abs_limit(i, pid->integral_limit);//250  积分项限幅5%调节行程
	abs_limit(p, pid->p_limit);       //比例项限幅30%-40&
	pid->output=p+i+d;
	}
	
	if (PID_mode ==Incremental )			//位置式
	{
	pid->e[2] = set_val - rel_val; //更新误差
	pid->sum_e+=pid->e[2];
	pid->d_band=pid->e[2]-pid->e[1];  //微分项Incremental
	pid->e[1] = pid->e[2];
		p=pid->Kp*pid->e[2];    
		i=pid->Ki*pid->sum_e;
		d=pid->Kd*pid->d_band;    //比例项限幅50%
		
//	abs_limit(i, pid->integral_limit);//250  积分项限幅5%调节行程
//	abs_limit(p, pid->p_limit);       //比例项限幅30%-40&
	pid->output=p+i+d;
	}
	if(pid->output > pid->max_out)
			pid->output = pid->max_out;
		if(pid->output < -(pid->max_out))
			pid->output = -(pid->max_out);  //不能超过最大输出
//经检验，此算法没问题；			
	pid->e[0] = pid->e[1];
	pid->e[1] = pid->e[2];	
}

void abs_limit(float a, float ABS_MAX)
{
    if (a > ABS_MAX)
        a = ABS_MAX;
    if (a < -ABS_MAX)
        a = -ABS_MAX;
}

