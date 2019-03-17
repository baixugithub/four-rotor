/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��IMU.c
 * ����    ����̬����         
 * ʵ��ƽ̨��HT_Hawk
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team 
**********************************************************************************/
#include "board_config.h"
#include "MultiRotor_ahrs.h"
#include "MPU6050.h"

#define KpDef 0.7f
#define KiDef 0.0005f
#define SampleRateHalf 0.001f  

#define  IIR_ORDER     4      //ʹ��IIR�˲����Ľ���
double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //ϵ��b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//ϵ��a
double InPut_IIR[3][IIR_ORDER+1] = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};

// /*	
// 	Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
// 	R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��	
// */
// #define KALMAN_Q        0.02
// #define KALMAN_R        8.0000

Quaternion NumQ = {1, 0, 0, 0};
EulerAngle AngE = {0};


int16_t MAG[3];



void AHRS_getValues(void)
{
	static float x,y,z;
	
	MPU6050_Dataanl();
	
	HMC5883lRead(MAG);
	
	// ���ٶȼ�IIR�˲�
	sensor.acc.averag.x = IIR_I_Filter(sensor.acc.origin.x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	sensor.acc.averag.y = IIR_I_Filter(sensor.acc.origin.y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	sensor.acc.averag.z = IIR_I_Filter(sensor.acc.origin.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	
	// ������һ�׵�ͨ�˲�
 	sensor.gyro.averag.x = LPF_1st(x,sensor.gyro.radian.x,0.386f);	x = sensor.gyro.averag.x;
 	sensor.gyro.averag.y = LPF_1st(y,sensor.gyro.radian.y,0.386f);	y = sensor.gyro.averag.y;
 	sensor.gyro.averag.z = LPF_1st(z,sensor.gyro.radian.z,0.386f);	z = sensor.gyro.averag.z;
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : AHRS_Update
**���� : AHRS
**���� : None
**��� : None
**ʹ�� : AHRS_Update();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_GetQ( Quaternion *pNumQ )
{
  fp32 ErrX, ErrY, ErrZ;
  fp32 AccX, AccY, AccZ;
  fp32 GyrX, GyrY, GyrZ;
	fp32 Normalize;
  static fp32 exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
	Gravity V;
	
	// ���ٶȹ�һ��
	Normalize = Q_rsqrt(squa(sensor.acc.averag.x)+ squa(sensor.acc.averag.y) +squa(sensor.acc.averag.z));
	AccX = sensor.acc.averag.x*Normalize;
	AccY = sensor.acc.averag.y*Normalize;
	AccZ = sensor.acc.averag.z*Normalize;

	// ��ȡ��������
	V = Quaternion_vectorGravity(&NumQ);
	
	// �������
	ErrX = (AccY*V.z - AccZ*V.y);
	ErrY = (AccZ*V.x - AccX*V.z);
	ErrZ = (AccX*V.y - AccY*V.x);
 	
	exInt = exInt + ErrX * KiDef;
	eyInt = eyInt + ErrY * KiDef;
	ezInt = ezInt + ErrZ * KiDef;
	
	// �����޷�
	exInt = data_limit(exInt,0.035,-0.035);
	eyInt = data_limit(eyInt,0.035,-0.035);
	ezInt = data_limit(ezInt,0.035,-0.035);
	
	GyrX = sensor.gyro.averag.x * Gyro_Gr + KpDef * VariableParameter(ErrX) * ErrX  +  exInt;
	GyrY = sensor.gyro.averag.y * Gyro_Gr + KpDef * VariableParameter(ErrY) * ErrY  +  eyInt;
	GyrZ = sensor.gyro.averag.z * Gyro_Gr + KpDef * VariableParameter(ErrZ) * ErrZ  +  ezInt;
	
	
	// һ�����������, ������Ԫ��
	Quaternion_RungeKutta(&NumQ, GyrX, GyrY, GyrZ, SampleRateHalf);
	
	// ��Ԫ����һ��
	Quaternion_Normalize(&NumQ);
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : AHRS_Geteuler
**���� : AHRS��ȡŷ����
**���� : None
**��� : None
**ʹ�� : AHRS_Geteuler();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_Geteuler(void)
{
	fp32 sin_pitch,sin_roll,cos_roll,cos_pitch;
	
	AHRS_getValues();
	
	// ��ȡ��Ԫ��
  AHRS_GetQ(&NumQ);
	
  // ��Ԫ��תŷ����
	Quaternion_ToAngE(&NumQ, &AngE);
	
  // ����ŷ���ǵ����Ǻ���ֵ
  sin_roll  = sin(AngE.Roll);
  sin_pitch = sin(AngE.Pitch);
  cos_roll  = cos(AngE.Roll);
  cos_pitch = cos(AngE.Pitch);
	
	//  �شŲ����ڻ�ش����ݲ�������ͣ�õش�����
	if(!flag.MagIssue && flag.MagExist){
		// �ش���ǲ���
		fp32 hx = MAG[0]*cos_pitch + MAG[1]*sin_pitch*sin_roll - MAG[2]*cos_roll*sin_pitch; 
		fp32 hy = MAG[1]*cos_roll + MAG[2]*sin_roll;
		
		// ���õشŽ��㺽���
		fp32 mag_yaw = -Degree(atan2((fp64)hy,(fp64)hx));
		 
		// �����ǻ��ֽ��㺽���
		AngE.Yaw += Degree(sensor.gyro.averag.z * Gyro_Gr * 2 * SampleRateHalf);
		
		// �شŽ���ĺ�����������ǻ��ֽ���ĺ���ǽ��л����ں� 
		if((mag_yaw>90 && AngE.Yaw<-90) || (mag_yaw<-90 && AngE.Yaw>90)) 
				AngE.Yaw = -AngE.Yaw * 0.998f + mag_yaw * 0.002f;
		else AngE.Yaw = AngE.Yaw * 0.998f + mag_yaw * 0.002f;
	}
	else 
		AngE.Yaw = Degree(AngE.Yaw);
		
		AngE.Roll = Degree(AngE.Roll);  // roll
		AngE.Pitch = Degree(AngE.Pitch); // pitch
}
