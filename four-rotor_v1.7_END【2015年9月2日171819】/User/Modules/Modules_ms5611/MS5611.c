/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：ms5611.c
 * 描述    ：ms5611配置         
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
**********************************************************************************/
#include "include.h"
#include "math.h"

//气压计状态机
#define SCTemperature  0x01	  //开始温度转换
#define CTemperatureing  0x02 //正在转换温度
#define SCPressure  0x03	    //开始气压转换
#define SCPressureing  0x04	  //正在转换气压


 
/*
C1 压力灵敏度 SENS|T1
C2  压力补偿  OFF|T1
C3	温度压力灵敏度系数 TCS
C4	温度系数的压力补偿 TCO
C5	参考温度 T|REF
C6 	温度系数的温度 TEMPSENS
*/
uint32_t  Cal_C[7];	        //用于存放PROM中的6组数据1-6

double OFF_;
float Aux;
/*
dT 实际和参考温度之间的差异
TEMP 实际温度	
*/
uint64_t dT,TEMP;

/*
OFF 实际温度补偿
SENS 实际温度灵敏度
*/
uint64_t OFf,SENS;
uint32_t D1_Pres,D2_Temp;	// 数字压力值,数字温度值

uint32_t Pressure,Pressure_old,qqp;				//大气压
uint32_t TEMP2,T2,OFF2,SENS2;	//温度校验值
uint32_t Pres_BUFFER[20];
uint32_t Temp_BUFFER[100];


#define SAMPLE_NUM 10
int32_t Acc_speed_Filterbuffer[SAMPLE_NUM],acc_speedz;
int32_t SpeedZ,SpeedZ_old;


/*====================================================================================================*/
/*====================================================================================================*
**函数 : MS5611_Reset
**功能 : 复位MS5611
**输入 : None
**輸出 : None
**备注 : None
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
**函数 : MS5611_Init
**功能 : 复位MS5611
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
u8 MS5611_Init(void)
{
	delay_ms(10);
	//传感器复位
	MS5611_Reset();
	delay_ms(3);
	MS5611_Read_Prom();
	//开始读取温度
//	MS5611_Start_T();
	return 0;
}

/*******************************************************************************
  * @函数名称	MS561101BA_RESET
  * @函数说明   复位MS5611
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
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
  * @函数名称	MS5611_init
  * @函数说明   初始化5611
  * @输入参数  	无
  * @输出参数   无
  * @返回参数   无
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



