/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��led.c
 * ����    ��led����Ӧ��         
 * ʵ��ƽ̨��Air Nano���������
 * ��汾  ��ST3.5.0
**********************************************************************************/
#include "board_config.h"

led_Fsm LED;
LEDBuf_t LEDBuf;

/*
 * ��������LED_GPIO_Config
 * ����  ������LED�õ���I/O��
 * ����  ����
 * ���  ����
 */
 
void LED_GPIO_Config(void)
{		
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����GPIOB������ʱ��*/
	RCC_APB2PeriphClockCmd( RCC_GPIO_LED, ENABLE); 
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE); 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//����LEDʹ�õ��ùܽ�
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);

	/*ѡ��Ҫ���Ƶ�GPIOC����*/															   
  	GPIO_InitStructure.GPIO_Pin = LED_R | LED_G | LED_B;	

	/*��������ģʽΪͨ���������*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*������������Ϊ50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*���ÿ⺯������ʼ��GPIOB*/
  	GPIO_Init(GPIO_LED, &GPIO_InitStructure);		  

	/* �ر�����led��	*/
	GPIO_SetBits(GPIO_LED, LED_R | LED_G | LED_B);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;	
	
  GPIO_Init(GPIOE, &GPIO_InitStructure);	
  GPIO_SetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6);	
}
void LED_SHOW(void)
{
   LED_ALLON();
	 delay(900);
	 LED_ALLOFF();
	 delay(16000);
	 LED_ALLON();
	 delay(900);
	 LED_ALLOFF();
	 delay(16000);
	 LED_ALLON();
	 delay(900);
	 LED_ALLOFF();
	 delay(16000);
	 LED_ALLON();
}


void LEDReflash(void)
{
	if(LEDBuf.bits.R)
		Ledr_on;
	else
		Ledr_off;

	if(LEDBuf.bits.G)
		Ledg_on;
	else
		Ledg_off;

	if(LEDBuf.bits.B)
		Ledb_on;
	else
		Ledb_off;
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : LED_Fsm
**���� : LED״̬��
**���� :  
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void LED_Fsm(void)
{
 	switch(LED.event)
 	{
	  case Ht_ARMED:       
			if(++LED.cnt >= 120)  LED.cnt=0;
			if(LED.cnt<10 || (LED.cnt>20 && LED.cnt<30)) 
						LEDBuf.byte =LG|LB;
				else
						LEDBuf.byte =0;
		  break;
		case Ht_DISARMED:       
			if(++LED.cnt >= 60) LED.cnt=0;
		  if(LED.cnt<=20)
						LEDBuf.byte =LR;
			else if(LED.cnt<=40)
						LEDBuf.byte =LG;
			else  LEDBuf.byte =LB;
			break;	
		case Ht_CALIBRATA:           
         LEDBuf.byte =LB;
			break;
 		case Ht_CALIBRATM_X:         
          LEDBuf.byte =LR;
 			break;
		case Ht_CALIBRATM_Y:         
          LEDBuf.byte =LG;
 			break;
		case Ht_CALIBRATM_Z:         
          LEDBuf.byte =LB;
 			break;
 	}
	LEDReflash();
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Hto_LED_Reflash
**���� : LED״̬��
**���� :  
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void FailSafeLEDAlarm(void)
{
//	if(flag.ARMED)
//		LED.event=Ht_ARMED;
//	
//	if(!flag.ARMED)
//		LED.event=Ht_DISARMED;
	
	if(flag.calibratingA)
		LED.event=Ht_CALIBRATA;
	
	if(flag.calibratingA)		
 		 LED.event=Ht_CALIBRATA;
	
	if(flag.calibratingM){
		switch(flag.calibratingM)
 	  {
			case 1:LED.event=Ht_CALIBRATM_X; break;
			case 2:LED.event=Ht_CALIBRATM_Y; break;
			case 3:LED.event=Ht_CALIBRATM_Z; break;
		}
	}
	LED_Fsm();
}

