/*
******************* (C) COPYRIGHT 2015 Air Nano Team ***************************
 * 模块名称 : 控制程序
 * 文件名   ：
 * 描述     ：    
 * 实验平台 ：HT_Hawk
 * 库版本   ：ST3.5.0
 * 论坛    ：http://www.airnano.cn 
*********************************************************************************
*/
#include "include.h"
#include "MultiRotor_control.h"


struct _ctrl ctrl;
struct _target Target;
int16_t Moto_duty[4];
s16 *motor_array = Moto_duty;

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Calculate_target
**功能 : 计算目标量
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Calculate_Target(void) 
{
	int16_t ftemp=0;
	static u8 al_flag=0;
	
	Target.Pitch = (1500-RC_Data.PITCH)/(20 + 7*RC_Data.SENSITIVITY);
	Target.Roll = (RC_Data.ROLL-1500)/(20 + 7*RC_Data.SENSITIVITY);

  //目标航向控制。当油门大于最小检查值时，认为用户希望起飞。那么此时的航向做为目标航向
   if(RC_Data.THROTTLE > RC_MINCHECK ) 
   {
      if(flag.LockYaw != 1){  
				 flag.LockYaw = 1;
	       Target.Yaw = AngE.Yaw; //将当前的航向做为目标航向
      }
   }
   else 
   {
		 flag.LockYaw = 0;	
		 Target.Yaw = AngE.Yaw;
    } 
	//航向在中点设置一个死区
	if((RC_Data.YAW > 1600)||(RC_Data.YAW < 1400)){
		ftemp = 1500 - RC_Data.YAW; 
	  Target.Yaw += (ftemp / 200.0f)*0.1f; 
		
		//转[-180.0,+180.0]
	  if(Target.Yaw >180.0f) Target.Yaw -= 360.0f;	
	  else if(Target.Yaw <-180.0f)Target.Yaw += 360.0f;
	}
	if(flag.FlightMode){
    if(al_flag !=1){
		  al_flag =1;
			Target.Altiude = US100_Alt;//20;
		}
		if((RC_Data.THROTTLE > 1600)||(RC_Data.THROTTLE < 1400))
		{
			if(RC_Data.THROTTLE > 1600)		Target.Altiude +=  (float)(RC_Data.THROTTLE -1600)/500;
			if(RC_Data.THROTTLE < 1400)		Target.Altiude +=  (float)(RC_Data.THROTTLE -1400)/500;
			if(Target.Altiude>50) Target.Altiude=50;
			if(Target.Altiude<10) Target.Altiude=10;
		}
	}
	else al_flag = 0;
}

/***************************************************/
/*void CONTROL(float rol, float pit, float yaw)    */
/*输入：rol   横滚角                               */
/*      pit   俯仰角                               */
/*			yaw   航向                                 */
/*输出：                                           */
/*备注：串级PID 控制   外环（角度环）采用PID调节    */
/*                     内环（角速度环）采用PD调节  */
/***************************************************/
void CONTROL(struct _target Goal)   
{
	float  deviation_pitch,deviation_roll,deviation_yaw;
	
	if(ctrl.ctrlRate >= 2)
	{
		// 计算角度误差 
		deviation_pitch = Goal.Pitch - AngE.Pitch;
		deviation_roll = Goal.Roll - AngE.Roll;
		
		//航向计算////////////
		if((Goal.Yaw - AngE.Yaw)>180 || (Goal.Yaw - AngE.Yaw)<-180)
		{
			if(Goal.Yaw>0 && AngE.Yaw<0)  deviation_yaw= (-180 - AngE.Yaw) +(Goal.Yaw - 180);
			if(Goal.Yaw<0 && AngE.Yaw>0)  deviation_yaw= (180 - AngE.Yaw) +(Goal.Yaw + 180);
		}
    else  deviation_yaw = Goal.Yaw - AngE.Yaw;
		
		// 角度误差积分
		ctrl.pitch.shell.increment += deviation_pitch;
		ctrl.roll.shell.increment  += deviation_roll;
		
		// 误差积分限幅
		ctrl.pitch.shell.increment = data_limit(ctrl.pitch.shell.increment,ctrl.pitch.shell.increment_max,-ctrl.pitch.shell.increment_max);
		ctrl.roll.shell.increment  = data_limit(ctrl.roll.shell.increment,ctrl.pitch.shell.increment_max,-ctrl.pitch.shell.increment_max);
		//= data_limit()


		// 角度PID输出->作为内环角速度目标量
		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * deviation_pitch + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment;
		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * deviation_roll + ctrl.roll.shell.ki * ctrl.roll.shell.increment;
    ctrl.yaw.shell.pid_out   = ctrl.yaw.shell.kp * deviation_yaw;
		
    ctrl.ctrlRate = 0; 
	}
	ctrl.ctrlRate ++;
	Attitude_RatePID(0.002f);
//	if(baixu == 0)
	Motor_Conter();
//	else
//	Motor_Conter_1();
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Attitude_RatePID
**功能 : 角速率控制PID
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Attitude_RatePID(float T)
{
  fp32 E_pitch,E_roll,E_yaw;
	
	// 计算偏差  
	E_pitch = ctrl.pitch.shell.pid_out - sensor.gyro.averag.y * Gyro_G;
	E_roll  = ctrl.roll.shell.pid_out  - sensor.gyro.averag.x * Gyro_G;
	E_yaw   = ctrl.yaw.shell.pid_out   - sensor.gyro.averag.z * Gyro_G;
	
  // 期望角速度限幅
	E_pitch = data_limit(E_pitch,300,-300);
	E_roll  = data_limit(E_roll,300,-300);
	E_yaw   = data_limit(E_yaw,300,-300);
	
	// 积分
	ctrl.pitch.core.increment += E_pitch * (T/0.002f);
	ctrl.roll.core.increment  += E_roll  * (T/0.002f);
	ctrl.yaw.core.increment   += E_yaw   * (T/0.002f);
	
	// 角速度误差积分限幅
	ctrl.pitch.core.increment = data_limit(ctrl.pitch.core.increment,20,-20);
	ctrl.roll.core.increment  = data_limit(ctrl.roll.core.increment,20,-20);
	ctrl.yaw.core.increment   = data_limit(ctrl.yaw.core.increment,20,-20);
	
	// 
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * E_pitch;
	ctrl.roll.core.kp_out  = ctrl.roll.core.kp  * E_roll;
	ctrl.yaw.core.kp_out   = ctrl.yaw.core.kp   * E_yaw;
	
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
	ctrl.roll.core.ki_out  = ctrl.roll.core.ki  * ctrl.roll.core.increment;
	ctrl.yaw.core.ki_out   = ctrl.yaw.core.ki   * ctrl.yaw.core.increment;
	
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (sensor.gyro.histor.y - sensor.gyro.averag.y) * ( 0.002f/T );
	ctrl.roll.core.kd_out  = ctrl.roll.core.kd  * (sensor.gyro.histor.x - sensor.gyro.averag.x) * ( 0.002f/T );
	ctrl.yaw.core.kd_out   = ctrl.yaw.core.kd   * (sensor.gyro.histor.z - sensor.gyro.averag.z) * ( 0.002f/T );
	
	// 角速度PID输出 
	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;
	ctrl.roll.core.pid_out  = ctrl.roll.core.kp_out  + ctrl.roll.core.ki_out  + ctrl.roll.core.kd_out;
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.kp_out   + ctrl.yaw.core.kd_out;
	
	sensor.gyro.histor.y = sensor.gyro.averag.y;
	sensor.gyro.histor.x = sensor.gyro.averag.x;   
	sensor.gyro.histor.z = sensor.gyro.averag.z; 
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Motor_Conter(void)
**功能 : 电机控制
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Motor_Conter(void)
{
	s16 pitch,roll,yaw;
	
	pitch = ctrl.pitch.core.pid_out;
	roll  = ctrl.roll.core.pid_out;    
	yaw   = -ctrl.yaw.core.pid_out;
	
	if(flag.FlightMode)
	{
		Moto_duty[0] = g_HightPwm - pitch - roll + yaw;
		Moto_duty[1] = g_HightPwm - pitch + roll - yaw;
		Moto_duty[2] = g_HightPwm + pitch + roll + yaw;
		Moto_duty[3] = g_HightPwm + pitch - roll - yaw;

	}
//	else if(RC_Data.THROTTLE > RC_MINCHECK) 
//	{
//		int date_throttle	= (RC_Data.THROTTLE-1000)/cos(AngE.Roll/RtA)/cos(AngE.Pitch/RtA);
//		
//		Moto_duty[0] = date_throttle - pitch - roll + yaw;
//		Moto_duty[1] = date_throttle - pitch + roll - yaw;
//		Moto_duty[2] = date_throttle + pitch + roll + yaw;
//		Moto_duty[3] = date_throttle + pitch - roll - yaw;
//	}
//	else
//	{	
//	    Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] = IDLING;	
//		Reset_Integral();		
//	}
//	if(flag.ARMED) //飞控上锁标志位(锁定飞控)
	moto_PwmRflash(&Moto_duty[0]);		
//	else            moto_STOP();	
}
//
//void Motor_Conter_1(void)
//{
//	s16 pitch,roll,yaw;
//	
//	pitch = ctrl.pitch.core.pid_out;
//	roll  = ctrl.roll.core.pid_out;    
//	yaw   = -ctrl.yaw.core.pid_out;
//	
//	if(flag.FlightMode)
//	{
//		Moto_duty[0] = HightPwm - pitch - roll + yaw;
//		Moto_duty[1] = HightPwm - pitch + roll - yaw;
//		Moto_duty[2] = HightPwm + pitch + roll + yaw;
//		Moto_duty[3] = HightPwm + pitch - roll - yaw;
//	}
//////	else if(RC_Data.THROTTLE > RC_MINCHECK) 
//////	{
//////		int date_throttle	= (RC_Data.THROTTLE-1000)/cos(AngE.Roll/RtA)/cos(AngE.Pitch/RtA);
//////		
//////		Moto_duty[0] = date_throttle - pitch - roll + yaw;
//////		Moto_duty[1] = date_throttle - pitch + roll - yaw;
//////		Moto_duty[2] = date_throttle + pitch + roll + yaw;
//////		Moto_duty[3] = date_throttle + pitch - roll - yaw;
//////	}
//////	else
//////	{	
//////	    Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] = IDLING;	
//////		Reset_Integral();		
//////	}
////	if(flag.ARMED) //飞控上锁标志位(锁定飞控)
//	moto_PwmRflash(&Moto_duty[0]);		
////	else            moto_STOP();	
//}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Reset_Integral
**功能 : 积分清零
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Reset_Integral(void)
{
	ctrl.pitch.shell.increment = 0;
	ctrl.roll.shell.increment= 0;	
	ctrl.pitch.core.increment = 0;		
	ctrl.roll.core.increment = 0;		
	ctrl.yaw.core.increment = 0;
}
