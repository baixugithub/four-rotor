/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��Ultrasonic.c
 * ����    ��Ultrasonic         
 * ʵ��ƽ̨��Air Nano���������
 * ��汾  ��ST3.5.0
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
**���� : Ultrasonic_Config
**���� : ����������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Ultrasonic_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	//EXTI_InitTypeDef EXTI_InitStructure;
	
	/* config the extiline(PB0) clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = TRIG_PIN;					    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//��Ϊ�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         
	GPIO_Init(TRIG_PORT, &GPIO_InitStructure);	        //��ʼ������GPIO 	
	GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
	

}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Ultrasonic_Pulsing
**���� : ����������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Ultrasonic_Pulsing(void)
{
	GPIO_SetBits(TRIG_PORT,TRIG_PIN);		  //��>10US�ĸߵ�ƽ
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
**���� : Ultrasonic_Config
**���� : ����������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Filter_Hight(u16 set_hight)
{
	RcvIndex = 0; 
	
	g_Alt_Hight=US100_Alt;

	if(g_Alt_Hight-g_Alt_HightOld>40)		   //����������
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

void Hight_PwmOut(void)//��������PWM���
{
	g_HightPwm=560+g_HightControl;	  //��ʼֵ500 �����ԣ�575��11.3V��*2015��8��14��02:31:36*��
	//[���ԣ�1.580��11.1V���߶ȿ���]
	if(g_HightPwm>800) g_HightPwm=800;
	if(g_HightPwm<200) g_HightPwm=200;
}
