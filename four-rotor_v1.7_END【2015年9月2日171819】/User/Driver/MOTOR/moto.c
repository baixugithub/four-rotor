/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��moto.c
 * ����    �������������         
 * ʵ��ƽ̨��Air Nano���������
 * ��汾  ��ST3.5.0
**********************************************************************************/
#include "moto.h"


void Tim4_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	/**********************************************************
	72 000 000/72=1M
	1000 000/2500=400Hz
	���Բ�����PWMΪ400Hz
	����Ϊ2.5ms����Ӧ2500�ļ���ֵ��1ms~2ms��Ӧ�ļ���ֵΪ1000~2000��
	**********************************************************/
	TIM_TimeBaseStructure.TIM_Period = 2499;		//��������	
	TIM_TimeBaseStructure.TIM_Prescaler = 71;	//pwmʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//���ϼ���
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

void PWM_OUT_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* GPIOA and GPIOC clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	Tim4_init();	
}


/*====================================================================================================*/
/*====================================================================================================*
**���� : pwmWriteMotor
**���� : PWMд����
**���� : 
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void pwmWriteMotor(uint8_t index, uint16_t value)
{    
	if(value > Moto_PwmMax)  value = Moto_PwmMax;
	if(value <= 0)           value = 0;
	// pwmWritePtr(index, value);
}

void writeMotors(int16_t *Moter)
{
    uint8_t i;

    for (i = 0; i < 4; i++)
        pwmWriteMotor(i, Moter[i]);
}
//������·���ת��
void moto_PwmRflash(int16_t *Moter)
{		
	for(u8 i=0;i<4;i++)
	{
		if(*(Moter+i) > Moto_PwmMax) 
			*(Moter+i) = Moto_PwmMax;
	}
	for(u8 i=0;i<4;i++)
	{
		if(*(Moter+i) <= 0 )  *(Moter+i) = 0;
	}
	TIM4->CCR1 = 1000 + *(Moter++);
	TIM4->CCR2 = 1000 + *(Moter++);
	TIM4->CCR3 = 1000 + *(Moter++);
	TIM4->CCR4 = 1000 + *Moter;
}
//���ֹͣת��
void moto_STOP(void)
{
	TIM4->CCR1 = 1000;
	TIM4->CCR2 = 1000;
	TIM4->CCR3 = 1000;
	TIM4->CCR4 = 1000;
}
