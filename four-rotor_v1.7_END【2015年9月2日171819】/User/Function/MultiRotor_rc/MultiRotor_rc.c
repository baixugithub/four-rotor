/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：RC.c
 * 描述    ：接收遥控器数据         
 * 实验平台：HT_Hawk
 * 库版本  ：ST3.5.0
 * 论坛    ：http://www.airnano.cn
**********************************************************************************/
#include "MultiRotor_rc.h"
#include "include.h"

RC_GETDATA RC_Data;
rcReadRawData rcReadRawFunc = RC_Data_Refine;


void RDAU(void)
{
	RC_directive();
	rcReadRawFunc();
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : RC_directive
**功能 : 遥控指令
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void RC_directive(void)
{
//	u8 stTmp = 0,i;
//	static u8  rcSticks;
//	static u8  rcDelayCommand;
//	static u16 seltLockCommend;	
//	
//	for (i = 0; i < 4; i++) {
//			stTmp >>= 2;
//			if (RC_Data.rc_data[i] > RC_MINCHECK)
//					stTmp |= 0x80;  // check for MIN
//			if (RC_Data.rc_data[i] < RC_MAXCHECK)
//					stTmp |= 0x40;  // check for MAX
//	}
//	if (stTmp == rcSticks) {
//			if (rcDelayCommand < 250)
//					rcDelayCommand++;
//	} else
//	rcDelayCommand = 0;
//	rcSticks = stTmp;
//	
//	if (rcDelayCommand == 150)
//	{
//		if (flag.ARMED){
//			 if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)   //上锁 
//				  flag.ARMED=0;
//		}
//		else{
//	if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)    //解锁   
//		flag.ARMED=1;
//	if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)    //加速度矫正  
//		flag.calibratingA = 1;
//	if ((rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_HI) && flag.calibratingM_pre)  //指南针矫正 
//	  flag.calibratingM = 1;
//	if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_LO)    
//		flag.calibratingM_pre = 1;
//	else flag.calibratingM_pre = 0;
//			
//    }
//	}
//	//武装之后一段时间油门保持最低  则自动解除武装
//	if (flag.ARMED)
//	{
//	   if (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_CE) {
//		    if (seltLockCommend < AUTODISARMDE_TIME)
//					 seltLockCommend++;
//				else 
//					 flag.ARMED=0;
//		 }
//		 else 
//        seltLockCommend = 0;			 
//	}
	if(RC_Data.rc_data[4]<RC_MINCHECK)flag.FlightMode=1;// 
	if(RC_Data.rc_data[4]>RC_MAXCHECK) flag.FlightMode=0;
//	
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : RcData_Refine
**功能 : 提炼遥控数据
**输入 : None
**输出 : None
**备注 : 无
**====================================================================================================*/
/*====================================================================================================*/
void RC_Data_Refine(void)
{
	u8 chan,a;	

	u16 rcDataMax[6], rcDataMin[6];
	static int16_t rcDataCache[6][4], rcDataMean[6];
	static uint8_t rcValuesIndex = 0;

	rcValuesIndex++;
	for (chan = 0; chan < 6; chan++) 
	{
		//滑动平均值滤波，4次
		if(RC_Pwm_In[chan]>2800 || RC_Pwm_In[chan]<800) 
			RC_Pwm_In[chan] = RC_Pwm_In_his[chan];
	
		rcDataCache[chan][rcValuesIndex % 4] = RC_Pwm_In[chan] ;		
		RC_Pwm_In_his[chan] = RC_Pwm_In[chan];
		
		rcDataMean[chan] = 0;
		rcDataMax[chan] = 0;
		rcDataMin[chan] = 25000;
		
		for (a = 0; a < 4; a++)
		{
			// 记录缓存中最大值 && 最小值
			if(rcDataCache[chan][a] > rcDataMax[chan])  rcDataMax[chan] = rcDataCache[chan][a];     
			if(rcDataCache[chan][a] < rcDataMin[chan])	rcDataMin[chan] = rcDataCache[chan][a]; 
			// 求和
			rcDataMean[chan] += rcDataCache[chan][a];  
		}
		// 剔除缓存中 最大值 && 最小值 
		rcDataMean[chan] = (rcDataMean[chan] - (rcDataMax[chan] + rcDataMin[chan])) / 2;
	} 

	RC_Data.ROLL  = RC_Data.rc_data[0] = rcDataMean[0];//横滚角
	RC_Data.PITCH = RC_Data.rc_data[1] = rcDataMean[1];//俯仰角
	RC_Data.YAW   = RC_Data.rc_data[2] =rcDataMean[3]; //航向角
	RC_Data.THROTTLE  = RC_Data.rc_data[3] =rcDataMean[2];//油门
	RC_Data.rc_data[4] =rcDataMean[4];
	RC_Data.rc_data[5] =rcDataMean[5];
}

