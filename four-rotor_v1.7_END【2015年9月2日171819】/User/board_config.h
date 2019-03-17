#ifndef __BOARD_CONFIG_H
#define	__BOARD_CONFIG_H

#include "include.h"

/*-------------������ƫ����----------------------*/
/*-------------��Ҫ  ��Ҫ��----------------------*/
#define FLASH_EXCURSION  0x20000
#define pro_FALG_ADD 0x0801FFF0

/*----------------�������----------------------*/
#define IDLING 210

/*--------------ң�ؿ��Ʒ�ʽѡ��----------------*/
#define RC_CONTROL_USE_NRF24l01

/*---------------�����ǲɼ�---------------------*/
#define GYRO_GATHER   100 

/*----------------���ż��----------------------*/
#define RC_MINCHECK   1200
#define RC_MAXCHECK   1800

/*-------------�Զ������װʱ��-----------------*/
#define AUTODISARMDE_TIME 2500  

/*-------------����ѡ�񿪹�-----------------*/
/*-����0�� KEY0 = 0 & KYE1 = 0��A����>B��*/
/*-����1�� KEY0 = 0 & KYE1 = 1����ͣ��A<����>B��ͣ����*/
/*-����2�� KEY0 = 1 & KYE1 = 0��A<����>B��ͣ����*/
/*-����3�� KEY0 = 1 & KYE1 = 1��O����>C����>D����>E����>F����O��*/

#define KEY1 PAin(0)   //PA0   TIM2����>PWM_IN1
#define KEY2 PAin(1)   //PA1  TIM2����>PWM_IN2
#define Sign_Video PAout(2)  //PA2  TIM2����>PWM_IN3

extern u8 Mark_forward;	  //2
extern u8 Mark_forward_1;//3
extern u8 Mark_forward_2;//4


typedef void (*rcReadRawData)(void);        

#endif /* __BOARD_CONFIG_H */
