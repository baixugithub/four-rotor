/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��time.c
 * ����    ��TIM3����         
 * ʵ��ƽ̨��Air Nano���������
 * ��汾  ��ST3.5.0
**********************************************************************************/
#include "time.h"
#include "stm32f10x.h"


/**************************ʵ�ֺ���********************************************
*����ԭ��:		
*��������:2ms�ж�һ��,������Ϊ2000		
*******************************************************************************/
void TIM5_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//�������ã�ʱ���ͱȽ�������ã���������ֻ�趨ʱ�����Բ���OC�Ƚ����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_DeInit(TIM5);

	TIM_TimeBaseStructure.TIM_Period=2000;//װ��ֵ
	//prescaler is 1200,that is 72000000/72/500=2000Hz;
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;//��Ƶϵ��
	//set clock division 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	//count up
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	//clear the TIM5 overflow interrupt flag
	TIM_ClearFlag(TIM5,TIM_FLAG_Update);
	//TIM5 overflow interrupt enable
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	//enable TIM5
	TIM_Cmd(TIM5,DISABLE);
}
void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* NVIC_PriorityGroup */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the USARTy Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	 
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	

}
