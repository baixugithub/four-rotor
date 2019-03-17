/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��ms5611.c
 * ����    ��ms5611����         
 * ʵ��ƽ̨��Air Nano���������
 * ��汾  ��ST3.5.0
**********************************************************************************/
#include "include.h"
#include "math.h"

//��ѹ��״̬��
#define SCTemperature  0x01	  //��ʼ�¶�ת��
#define CTemperatureing  0x02 //����ת���¶�
#define SCPressure  0x03	    //��ʼ��ѹת��
#define SCPressureing  0x04	  //����ת����ѹ


 
/*
C1 ѹ�������� SENS|T1
C2  ѹ������  OFF|T1
C3	�¶�ѹ��������ϵ�� TCS
C4	�¶�ϵ����ѹ������ TCO
C5	�ο��¶� T|REF
C6 	�¶�ϵ�����¶� TEMPSENS
*/
uint32_t  Cal_C[7];	        //���ڴ��PROM�е�6������1-6

double OFF_;
float Aux;
/*
dT ʵ�ʺͲο��¶�֮��Ĳ���
TEMP ʵ���¶�	
*/
uint64_t dT,TEMP;

/*
OFF ʵ���¶Ȳ���
SENS ʵ���¶�������
*/
uint64_t OFf,SENS;
uint32_t D1_Pres,D2_Temp;	// ����ѹ��ֵ,�����¶�ֵ

uint32_t Pressure,Pressure_old,qqp;				//����ѹ
uint32_t TEMP2,T2,OFF2,SENS2;	//�¶�У��ֵ
uint32_t Pres_BUFFER[20];
uint32_t Temp_BUFFER[100];


#define SAMPLE_NUM 10
int32_t Acc_speed_Filterbuffer[SAMPLE_NUM],acc_speedz;
int32_t SpeedZ,SpeedZ_old;


/*====================================================================================================*/
/*====================================================================================================*
**���� : MS5611_Reset
**���� : ��λMS5611
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void MS5611_Reset(void)
{
	Single_Write(MS5611_ADDR,CMD_RESET,1);
}


u8 MS5611_Read_Prom(void)
{
//	uint8_t rxbuf[2] = { 0, 0 };
	u8 check = 0;
	u8 i;

	for (i = 0; i < PROM_NB; i++)
	{
	//	check += IIC_Read_nByte(MS5611_ADDR, CMD_PROM_RD + i * 2, 2, rxbuf); // send PROM READ command
	//	ms5611_prom[i] = rxbuf[0] << 8 | rxbuf[1];
	}

	if(check==PROM_NB)
		return 1;
	else
		return 0;
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : MS5611_Init
**���� : ��λMS5611
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
u8 MS5611_Init(void)
{
	delay_ms(10);
	//��������λ
	MS5611_Reset();
	delay_ms(3);
	MS5611_Read_Prom();
	//��ʼ��ȡ�¶�
//	MS5611_Start_T();
	return 0;
}

/*******************************************************************************
  * @��������	MS561101BA_RESET
  * @����˵��   ��λMS5611
  * @�������   ��
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
void MS561101BA_RESET(void)
{
	I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress);
	I2C_WaitAck();
	I2C_SendByte(MS561101BA_RST);
	I2C_WaitAck();
	I2C_Stop();
}
/*******************************************************************************
  * @��������	MS5611_init
  * @����˵��   ��ʼ��5611
  * @�������  	��
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
u8 MS5611_init(void)
 {	 
   uint8_t d1,d2,i;
   MS561101BA_RESET();	 // Reset Device
	 delay_ms(20);  
	 for(i=1;i<=6;i++)
	 {
		I2C_Start();
		I2C_SendByte(MS561101BA_SlaveAddress);
		I2C_WaitAck();
		I2C_SendByte((MS561101BA_PROM_RD+i*2));
		I2C_WaitAck();
	  I2C_Stop();
		delay_ms(1);

		I2C_Start();
		I2C_SendByte(MS561101BA_SlaveAddress+1);
		I2C_WaitAck();
		d1=I2C_RadeByte();
		I2C_Ack();
		d2=I2C_RadeByte();
		I2C_NoAck();
		I2C_Stop();

		I2C_delay();
		Cal_C[i]=((uint16_t)d1<<8)|d2;
	  delay_ms(10);
	 }
	 
	 return !Cal_C[0];
 }



