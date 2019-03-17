/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：Ultrasonic.c
 * 描述    ：Ultrasonic         
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
**********************************************************************************/
#include "include.h"

#define	TRIG_PORT      GPIOB		//TRIG       
#define	ECHO_PORT      GPIOB	    //ECHO 
#define	TRIG_PIN       GPIO_Pin_1   //TRIG       
#define	ECHO_PIN       GPIO_Pin_0	//ECHO  



float US100_Alt;
float US100_Alt_V;


/*====================================================================================================*/
/*====================================================================================================*
**函数 : Ultrasonic_Config
**功能 : 超声波配置
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Ultrasonic_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	//EXTI_InitTypeDef EXTI_InitStructure;
	
	/* config the extiline(PB0) clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = TRIG_PIN;					    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//设为推挽输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         
	GPIO_Init(TRIG_PORT, &GPIO_InitStructure);	        //初始化外设GPIO 	
	GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
	

}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Ultrasonic_Pulsing
**功能 : 启动超声波
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Ultrasonic_Pulsing(void)
{
	GPIO_SetBits(TRIG_PORT,TRIG_PIN);		  //送>10US的高电平
	delay_us(20);
	GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
}


unsigned int g_Hight=0,g_HightOld=0;
float g_Alt_Hight=0,g_Alt_HightOld=0;
float g_HightControl=0,g_HightControlold=0,g_HightPwm=0,hight_increment=0;
unsigned char RcvIndex,GLengthHigh, GLengthLow;
float g_hight_Kp=0.7,g_hight_Ki=0.01,g_hight_Kd=5;  
float hight_error=0,hight_errorold=0,hight_erroroldd,cao;


/*====================================================================================================*/
/*====================================================================================================*
**函数 : Ultrasonic_Config
**功能 : 超声波配置
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Filter_Hight(u16 set_hight)
{
	RcvIndex = 0; 
	
	g_Alt_Hight=US100_Alt;

	if(g_Alt_Hight-g_Alt_HightOld>40)		   //防测量错误
	   g_Alt_Hight=g_Alt_HightOld+40;
	else if(g_Alt_HightOld-g_Alt_Hight>40)
		g_Alt_Hight=g_Alt_HightOld-40;
	g_Alt_HightOld = g_Alt_Hight;
	
	hight_errorold=hight_error;
	hight_error=set_hight-g_Alt_Hight;

	hight_increment += hight_error;
	
	cao = hight_error/50;
	
	if(cao>=0) {
     if(cao< 0.1) cao = 0.1; 
		 if(cao>=1)   cao = 1;
	} 
	if(cao<=0){
    if(cao< -0.1) cao = -0.1; 
		 if(cao<= -1)   cao = -1;
	} 

	hight_increment = data_limit(hight_increment,cao*4000,-4000*cao);
		
	g_HightControl = g_hight_Kp*hight_error + g_hight_Ki*hight_increment +g_hight_Kd*(hight_error - hight_errorold);
	
	if(g_HightControl>500)
		g_HightControl=500;
	else if(g_HightControl<-400)
		g_HightControl=-400;
}

void Hight_PwmOut(void)//超声波的PWM输出
{
	g_HightPwm=560+g_HightControl;	  //初始值500 【调试：575（11.3V）*2015年8月14日02:31:36*】
	//[调试：1.580（11.1V）高度可以]
	if(g_HightPwm>800) g_HightPwm=800;
	if(g_HightPwm<200) g_HightPwm=200;
}
