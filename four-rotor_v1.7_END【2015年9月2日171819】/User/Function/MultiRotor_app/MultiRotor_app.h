#ifndef _MultiRotor_app_H_
#define _MultiRotor_app_H_
/* Includes ------------------------------------------------------------------*/
#include "include.h"


typedef struct {
	      u8 MpuExist;      // MPU����
	      u8 MagExist;      // MAG����
	      u8 NrfExist;      // NRF����
	      u8 MagIssue;      // MAG������
//        u8 ARMED;         // �������
	      u8 LockYaw;       // ��������       
		  u8 calibratingA;  // ���ٶȲɼ�
	      u8 calibratingM;  // �����Ʋɼ�
	      u8 calibratingM_pre; //������Ԥ�ɼ�
	      
	      u8 ParamSave;     // ���������־
	      u8 FlightMode;	//����ģʽ
	
	      u8 Loop_250Hz;	//4ms
	      u8 Loop_100Hz;	//10ms
	      u8 Loop_10Hz;		//50ms
	      u8 Loop_3s;		//3s
         }Flag_t;	//��־1
typedef struct {
	      u8 Loop1_250Hz;	//4ms
	      u8 Loop1_100Hz;	//10ms
	      u8 Loop1_10Hz;		//50ms
	      u8 Loop1_3s;		//3s
		  u8 Loop1_3_5s;		//3.5s
		  u8 Loop1_7s;	//7s
         }Flag1_t;	//��־2
typedef struct {
	      u8 Loop2_250Hz;	//4ms
	      u8 Loop2_100Hz;	//10ms
	      u8 Loop2_10Hz;		//50ms
	      u8 Loop2_3s;		//3s
		  u8 Loop2_3_5s;		//3.5s
		  u8 Loop2_7s;	//7s
         }Flag2_t;
typedef struct {
		  u8 Loop3_250Hz;	//4ms
		  u8 Loop3_100Hz;	//10ms
		  u8 Loop3_10Hz;		//50ms
		  u8 Loop3_1s;	//1.1s
	      u8 Loop3_2s;	//1.2s
	 	  u8 Loop3_3s;	//1.3s
		  u8 Loop3_4s;	//1.4s
	 	  u8 Loop3_5s;	//1.5s

		}Flag3_t;
//��������
typedef struct {
	      u8 Mark_0;	//ִ�е�0������A����>B����
		  u8 Mark_1;	//ִ�е�1������ (A<����>B��)
	      u8 Mark_2;	//ִ�е�2������
		  u8 Mark_3;	//ִ�е�3������

         }Mark_t;

extern Flag_t flag;	   		//�ж��жϱ�־λ
extern Flag1_t flag1;	    //�ж��жϱ�־λ
extern Flag2_t flag2;	   //�ж��жϱ�־λ
extern Flag3_t flag3;	   //�ж��жϱ�־λ

extern Mark_t mark;	  //����ִ�б�־λ

void Bootloader_Set(void);
void InitBoard(void);
void Sensor_Init(void);
void Screen_Update(void);
void Motor_Conter_1(void);//�Զ���������
void loop(void);
void Time_slice(void);	  //ʱ��Ƭ0

void loop_1(void);
void Time_slice_1(void);  //ʱ��Ƭ1

void loop_2(void);
void Time_slice_2(void);  //ʱ��Ƭ2

void loop_3(void);
void Time_slice_3(void);  //ʱ��Ƭ3

#endif /* __MultiRotor_app_H */



